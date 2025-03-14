from openvino.runtime import Core
import numpy as np
import cv2
import subprocess

# Paths to model files
MODEL_XML = "/home/matthewthomasbeck/Projects/Robot_Dog/model/person-detection-0200.xml"
MODEL_BIN = "/home/matthewthomasbeck/Projects/Robot_Dog/model/person-detection-0200.bin"

# Initialize OpenVINO Runtime
ie = Core()

# Load and compile the model
try:
    model = ie.read_model(model=MODEL_XML)
    compiled_model = ie.compile_model(model=model, device_name="MYRIAD")
    input_layer = compiled_model.input(0)
    output_layer = compiled_model.output(0)
    print(f"Model input shape: {input_layer.shape}")
except Exception as e:
    print(f"Failed to load and compile model: {e}")
    exit(1)

# Dummy image testing to validate model
try:
    dummy_input_shape = input_layer.shape  # Expected input shape
    dummy_input = np.ones(dummy_input_shape, dtype=np.float32)  # Create dummy input with ones
    results = compiled_model([dummy_input])[output_layer]  # Perform inference
    print("Dummy input test passed!")
except Exception as dummy_error:
    print(f"Error with dummy input: {dummy_error}")
    exit(1)

# Start the rpicam-vid process
camera_process = subprocess.Popen(
    [
        "rpicam-vid", "--width", "640", "--height", "480", "--framerate", "30",
        "--timeout", "0", "--output", "-", "--codec", "mjpeg", "--nopreview"
    ],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    bufsize=0
)

mjpeg_buffer = b''

try:
    while True:
        # Read MJPEG data
        chunk = camera_process.stdout.read(4096)
        if not chunk:
            print("Camera process stopped sending data.")
            break

        mjpeg_buffer += chunk

        # Find JPEG frame markers
        start_idx = mjpeg_buffer.find(b'\xff\xd8')
        end_idx = mjpeg_buffer.find(b'\xff\xd9')

        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            # Extract and decode frame
            frame_data = mjpeg_buffer[start_idx:end_idx + 2]
            mjpeg_buffer = mjpeg_buffer[end_idx + 2:]
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

            if frame is not None:
                try:
                    # Convert frame to RGB if required by the model
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                    # Resize frame to the model's expected input size (e.g., 300x300)
                    input_blob = cv2.resize(frame, (256, 256))  # Update size as needed

                    # Transpose to match the model's expected layout
                    input_blob = input_blob.transpose(2, 0, 1)  # (H, W, C) -> (C, H, W)

                    # Add batch dimension
                    input_blob = np.expand_dims(input_blob, axis=0)  # (C, H, W) -> (1, C, H, W)

                    # Ensure the data type is correct
                    input_blob = input_blob.astype(np.float32)

                    print(f"Inputs: {compiled_model.inputs}")
                    print(f"Outputs: {compiled_model.outputs}")


                    print(f"Input blob shape: {input_blob.shape}, dtype: {input_blob.dtype}")

                    # Perform inference
                    results = compiled_model([input_blob])[output_layer]

                    # Draw detections on the frame
                    for detection in results[0][0]:
                        confidence = detection[2]
                        if confidence > 0.5:  # Confidence threshold
                            xmin, ymin, xmax, ymax = map(
                                int,
                                detection[3:7] * [
                                    frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]
                                ]
                            )
                            label = f"ID {int(detection[1])}: {confidence:.2f}"
                            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                            cv2.putText(
                                frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                            )

                    # Display the frame
                    cv2.imshow("OpenVINO Inference", frame)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except Exception as inference_error:
                    print(f"Error during inference: {inference_error}")
            else:
                print("Failed to decode frame, skipping...")

finally:
    # Cleanup
    cv2.destroyAllWindows()
    camera_process.terminate()
    camera_process.wait()
