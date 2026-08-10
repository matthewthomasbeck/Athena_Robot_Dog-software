##################################################################################
# Copyright (c) 2025 Matthew Thomas Beck                                         #
#                                                                                #
# Licensed under the Creative Commons Attribution-NonCommercial 4.0              #
# International (CC BY-NC 4.0). Personal and educational use is permitted.       #
# Commercial use by companies or for-profit entities is prohibited.              #
##################################################################################

"""LAN teleop server: desktop sends WASD-style command strings → on-robot RL.

Protocol (same framing as website backend):
  [4-byte big-endian length][UTF-8 command]
Commands: w, s, a, d, w+a, arrowleft, arrowright, n, ...
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
from queue import Queue

import utilities.config as config


_server_sock: socket.socket | None = None
_stop_event = threading.Event()
COMMAND_QUEUE: Queue | None = None


def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _handle_client(conn: socket.socket, addr, command_queue: Queue) -> None:
    logging.info(f"(lan_teleop.py): Desktop connected from {addr}\n")
    try:
        while not _stop_event.is_set():
            length_bytes = _recv_exact(conn, 4)
            if length_bytes is None:
                logging.warning("(lan_teleop.py): Client closed connection.\n")
                break
            length = struct.unpack(">I", length_bytes)[0]
            if length <= 0 or length > 10_000:
                logging.error(f"(lan_teleop.py): Invalid frame length {length}\n")
                break
            payload = _recv_exact(conn, length)
            if payload is None:
                logging.warning("(lan_teleop.py): Incomplete payload; client gone.\n")
                break
            command = payload.decode("utf-8", errors="replace").strip()
            if not command:
                continue
            # Keep only the latest intent if the robot is busy.
            while not command_queue.empty():
                try:
                    command_queue.get_nowait()
                except Exception:
                    break
            command_queue.put(command)
            logging.info(f"(lan_teleop.py): Queued command '{command}'\n")
    except Exception as e:
        logging.error(f"(lan_teleop.py): Client handler error: {e}\n")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        logging.info(f"(lan_teleop.py): Desktop disconnected ({addr})\n")


def _accept_loop(server: socket.socket, command_queue: Queue) -> None:
    while not _stop_event.is_set():
        try:
            server.settimeout(1.0)
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(
                target=_handle_client, args=(conn, addr, command_queue), daemon=True
            ).start()
        except socket.timeout:
            continue
        except Exception as e:
            if not _stop_event.is_set():
                logging.error(f"(lan_teleop.py): Accept error: {e}\n")
            break


def start_lan_teleop_server() -> Queue | None:
    """Bind TCP server and return a command queue. Desktop connects TO the robot."""
    global _server_sock, COMMAND_QUEUE
    cfg = config.LAN_TELEOP_CONFIG
    host = cfg["BIND_HOST"]
    port = int(cfg["PORT"])
    _stop_event.clear()
    command_queue: Queue = Queue()
    COMMAND_QUEUE = command_queue

    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        _server_sock = server
        threading.Thread(target=_accept_loop, args=(server, command_queue), daemon=True).start()
        logging.info(f"(lan_teleop.py): Listening for desktop teleop on {host}:{port}\n")
        return command_queue
    except Exception as e:
        logging.error(f"(lan_teleop.py): Failed to start server on {host}:{port}: {e}\n")
        _server_sock = None
        COMMAND_QUEUE = None
        return None
