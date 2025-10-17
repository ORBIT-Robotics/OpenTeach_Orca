"""Minimal Quest hand-stream logger.

Listens for the Quest APK's ZeroMQ PUSH streams and decodes the right-hand
joint payload so you can confirm data flow before wiring it into a visualiser.
"""
from __future__ import annotations

import argparse
import datetime as dt
from socket import socket
import time
from typing import List, Tuple

import zmq


# Oculus right-hand bone order (matches OVRSkeleton.Bones for the APK)
RIGHT_HAND_BONES = [
    "Hand_Start",
    "Hand_WristRoot",
    "Hand_ForearmStub",
    "Hand_Thumb0",
    "Hand_Thumb1",
    "Hand_Thumb2",
    "Hand_Thumb3",
    "Hand_Index1",
    "Hand_Index2",
    "Hand_Index3",
    "Hand_Middle1",
    "Hand_Middle2",
    "Hand_Middle3",
    "Hand_Ring1",
    "Hand_Ring2",
    "Hand_Ring3",
    "Hand_Pinky0",
    "Hand_Pinky1",
    "Hand_Pinky2",
    "Hand_Pinky3",
    "Hand_ThumbTip",
    "Hand_IndexTip",
    "Hand_MiddleTip",
    "Hand_RingTip",
    "Hand_PinkyTip",
]


def bind_pull(context: zmq.Context, port: int) -> zmq.Socket:
    """Create a PULL socket bound on all local interfaces."""
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)  # keep only the freshest payload
    socket.bind(f"tcp://*:{port}")  #tells the PC to listen on all interfaces connected to the network
    return socket


def decode_hand_payload(payload: bytes) -> Tuple[str, List[Tuple[float, float, float]]]:
    """Turn the raw Quest string into a mode tag and xyz triples."""
    token = payload.decode("utf-8", errors="replace").strip()
    if ":" not in token:
        return token, []

    mode, encoded = token.split(":", maxsplit=1)
    joints: List[Tuple[float, float, float]] = []
    for part in encoded.split("|"):
        try:
            x_str, y_str, z_str = part.split(",", maxsplit=2)
            joints.append((float(x_str), float(y_str), float(z_str)))
        except ValueError:
            # Skip malformed chunks; payloads sometimes have a trailing colon.
            continue
    return mode, joints


def format_joint_preview(joints: List[Tuple[float, float, float]], limit: int = 5) -> str:
    """Return a condensed string showing the first few joints with bone names."""
    if not joints:
        return "[]"

    preview = []
    for idx, coords in enumerate(joints[:limit]):
        bone = RIGHT_HAND_BONES[idx] if idx < len(RIGHT_HAND_BONES) else f"joint_{idx}"
        preview.append(f"{bone}=({coords[0]:+.3f},{coords[1]:+.3f},{coords[2]:+.3f})")

    if len(joints) > limit:
        preview.append("…")
    return "[" + ", ".join(preview) + "]"


def main() -> None:
    parser = argparse.ArgumentParser(description="Quest hand stream logger")
    parser.add_argument("--keypoint-port", type=int, default=8087, help="Hand data port")
    parser.add_argument("--resolution-port", type=int, default=8095, help="Resolution button port")
    parser.add_argument("--pause-port", type=int, default=8100, help="Pause/continue port")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between log lines")
    args = parser.parse_args()

    ctx = zmq.Context()
    raw_socket = bind_pull(ctx, args.keypoint_port)
    resolution_socket = bind_pull(ctx, args.resolution_port)
    pause_socket = bind_pull(ctx, args.pause_port)

    poller = zmq.Poller()
    poller.register(raw_socket, zmq.POLLIN)
    poller.register(resolution_socket, zmq.POLLIN)
    poller.register(pause_socket, zmq.POLLIN)

    print("Listening on:")
    print(f"  hand data      tcp://<your-ip>:{args.keypoint_port}")
    print(f"  resolution btn tcp://<your-ip>:{args.resolution_port}")
    print(f"  pause status   tcp://<your-ip>:{args.pause_port}")
    print("Use your PC's Wi-Fi IP on the headset menu. Ctrl+C to quit.\n")

    last_log = 0.0
    last_hand_at = None
    warn_after = 0.5

    try:
        while True:
            events = dict(poller.poll(timeout=100))
            now = time.time()

            if raw_socket in events:
                payload = raw_socket.recv()
                mode, joints = decode_hand_payload(payload)
                last_hand_at = now

                if now - last_log >= args.interval:
                    ts = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
                    preview = format_joint_preview(joints)
                    print(f"[{ts}] HAND mode={mode} joints={len(joints)} {preview}")
                    last_log = now

            elif last_hand_at and now - last_hand_at > warn_after:
                ts = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
                print(f"[{ts}] hand stream idle for {now - last_hand_at:.2f}s")
                last_hand_at = None

            if resolution_socket in events:
                payload = resolution_socket.recv()
                ts = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
                print(f"[{ts}] RES {payload.decode(errors='replace')}")

            if pause_socket in events:
                payload = pause_socket.recv()
                ts = dt.datetime.fromtimestamp(now).strftime("%H:%M:%S")
                print(f"[{ts}] PAUSE {payload.decode(errors='replace')}")

    except KeyboardInterrupt:
        pass
    finally:
        raw_socket.close()
        resolution_socket.close()
        pause_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
