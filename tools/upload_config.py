#!/usr/bin/env python3
"""Upload a file (typically a config YAML) to a FluidNC board over serial via XModem.

Sends $Xmodem/Receive=<dest>, then streams the file with XModem-1K/CRC, the
same protocol fluidterm's Ctrl+X uses, but non-interactively so it can be
scripted. Optionally makes the uploaded file the active config and restarts.

Usage:
    upload_config.py -p /dev/ttyUSB0 dc-servo-v2.yaml --as config.yaml --activate --restart
"""
import argparse
import binascii
import sys
import time
from pathlib import Path

import serial
from serial.tools.list_ports import comports

STX = 0x02
EOT = 0x04
ACK = 0x06
NAK = 0x15
CAN = 0x18
CTRLZ = 0x1A

PACKET_SIZE = 1024
MAX_RETRIES = 10
# The receiver writes each packet to flash before ACKing, which can stall on LittleFS.
ACK_TIMEOUT = 10.0


class UploadError(Exception):
    pass


def log(msg, end="\n"):
    sys.stderr.write(msg + end)
    sys.stderr.flush()


def pick_port():
    ports = comports()
    if len(ports) == 1:
        return ports[0].device
    names = ", ".join(p.device for p in ports) or "none found"
    raise UploadError(f"specify the serial port with -p (available: {names})")


def open_port(port, baud):
    # Keep pyserial's default (DTR and RTS both asserted), which matches what the
    # Linux kernel raises on open. Clearing them makes pyserial drop DTR before RTS,
    # and that transient RTS-only state pulls EN low on ESP32 auto-reset circuits.
    return serial.Serial(port, baud, timeout=0.1)


def sync(ser, interval=0.5, max_wait=15.0):
    # Some adapters still reset the board on open, and commands sent during boot
    # are lost, so keep poking until a reply arrives. Waiting for silence instead
    # never ends when the board emits periodic logs or status reports.
    deadline = time.monotonic() + max_wait
    while time.monotonic() < deadline:
        ser.write(b"\n")
        poke_deadline = min(time.monotonic() + interval, deadline)
        while time.monotonic() < poke_deadline:
            if ser.readline().decode(errors="replace").strip() == "ok":
                return
    raise UploadError("board did not answer")


def read_byte(ser, timeout, accept=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if b and (accept is None or b[0] in accept):
            return b[0]
    return None


def wait_for_crc_request(ser, timeout=10.0):
    # The board may still emit text (e.g. [MSG:...]) containing 'C' before it
    # starts the handshake. The real request is a lone 'C' followed by silence,
    # since the receiver then waits up to 2 s for the first packet.
    deadline = time.monotonic() + timeout
    text = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == CAN:
            raise UploadError(f"board refused the upload: {text.decode(errors='replace').strip()}")
        if b[0] == ord("C"):
            nxt = read_byte(ser, 0.15)
            if nxt is None:
                return
            text += b
            b = bytes([nxt])
        text += b
        if b == b"\n":
            line = text.decode(errors="replace").strip()
            if line:
                log(f"< {line}")
            if line.startswith("error"):
                raise UploadError("board rejected $Xmodem/Receive")
            text.clear()
    raise UploadError("timed out waiting for the board to start XModem")


def build_packet(seq, chunk):
    chunk = chunk.ljust(PACKET_SIZE, bytes([CTRLZ]))
    crc = binascii.crc_hqx(chunk, 0)
    return bytes([STX, seq & 0xFF, 0xFF - (seq & 0xFF)]) + chunk + crc.to_bytes(2, "big")


def send_with_retries(ser, payload, what):
    for _ in range(MAX_RETRIES):
        ser.write(payload)
        ser.flush()
        # Log lines from other tasks keep flowing during the transfer. They are
        # plain text, so they never contain these bytes; resending on them would
        # make the board see duplicates and cancel after a few.
        reply = read_byte(ser, ACK_TIMEOUT, accept=(ACK, NAK, CAN))
        if reply == ACK:
            return
        if reply == CAN:
            raise UploadError(f"board cancelled the transfer during {what}")
    raise UploadError(f"too many retries on {what}")


def xmodem_send(ser, data):
    total = len(data)
    seq = 1
    for offset in range(0, total, PACKET_SIZE):
        send_with_retries(ser, build_packet(seq, data[offset:offset + PACKET_SIZE]), f"packet {seq}")
        seq += 1
        log(f"\r  sent {min(offset + PACKET_SIZE, total)}/{total} bytes", end="")
    if total:
        log("")
    send_with_retries(ser, bytes([EOT]), "EOT")


def wait_ok(ser, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if not line or line.startswith("<"):
            continue
        if line == "ok":
            return True
        log(f"< {line}")
        if line.startswith("error"):
            return False
    log("  timed out waiting for 'ok'")
    return False


def send_command(ser, cmd, timeout=5.0):
    log(f"> {cmd}")
    ser.write(f"{cmd}\n".encode())
    return wait_ok(ser, timeout)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", type=Path, help="local file to upload")
    parser.add_argument("-p", "--port", help="serial port (auto-detected if only one exists)")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("--as", dest="dest", help="filename on the board (default: local file name)")
    parser.add_argument("--activate", action="store_true", help="set $Config/Filename to the uploaded file")
    parser.add_argument("--restart", action="store_true", help="send $Bye after uploading to reload the config")
    args = parser.parse_args()

    try:
        data = args.file.read_bytes()
        dest = args.dest or args.file.name
        port = args.port or pick_port()

        with open_port(port, args.baud) as ser:
            sync(ser)

            log(f"> $Xmodem/Receive={dest}")
            ser.write(f"$Xmodem/Receive={dest}\n".encode())
            wait_for_crc_request(ser)
            xmodem_send(ser, data)
            if not wait_ok(ser, timeout=10.0):
                raise UploadError("upload failed")

            if args.activate and not send_command(ser, f"$Config/Filename={dest}"):
                raise UploadError("failed to set $Config/Filename")
            if args.restart:
                log("> $Bye")
                ser.write(b"$Bye\n")
                ser.flush()
    except (UploadError, OSError, serial.SerialException) as e:
        log(f"\nerror: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
