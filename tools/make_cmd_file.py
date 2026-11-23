#!/usr/bin/env python3
"""Makes the one-byte command files for mosquitto_pub -f (PowerShell cannot type raw bytes)."""
for name, data in [("start", b"\x01"), ("stop", b"\x02"), ("reset", b"\x03"),
                   ("bad0", b"\x00"), ("bad4", b"\x04"), ("two_bytes", b"\x01\x01"), ("old_text_start", b"START")]:
    open(name + ".bin", "wb").write(data)
print("made: start.bin stop.bin reset.bin (good) and bad0.bin bad4.bin two_bytes.bin old_text_start.bin (must be rejected)")