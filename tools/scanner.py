#!/usr/bin/env python3
import asyncio
import struct
from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from datetime import datetime

TARGET_NAME = "E104-BT5005A"
MANUFACTURER_ID = 0xFFFF
seen_ids = set()


def parse_frame(data: bytes) -> dict | None:
    if len(data) < 16:
        return None
    try:
        manufacturer_id, temperature, humidity, pressure, magic, battery, device_id = (
            struct.unpack_from("<HhHIIBB", data, 0)
        )
    except struct.error:
        return None
    if manufacturer_id != MANUFACTURER_ID:
        return None
    return {
        "temperature_c": temperature / 100.0,
        "humidity_pct": humidity / 100.0,
        "pressure_hpa": pressure / 10.0,
        "battery_pct": battery,
        "device_id": device_id,
        "magic": hex(magic),
    }


def callback(device: BLEDevice, adv: AdvertisementData) -> None:
    if device.name != TARGET_NAME:
        return
    mfr_data = adv.manufacturer_data.get(MANUFACTURER_ID)
    if mfr_data is None:
        return
    raw = struct.pack("<H", MANUFACTURER_ID) + mfr_data
    parsed = parse_frame(raw)
    if parsed is None:
        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] {device.address} — failed to parse frame"
        )
        return
    if parsed["device_id"] in seen_ids:
        return
    seen_ids.add(parsed["device_id"])
    print(
        f"[{datetime.now().strftime('%H:%M:%S')}] "
        f"RSSI: {adv.rssi:4d} dBm | "
        f"Temp: {parsed['temperature_c']:6.2f} °C | "
        f"Hum: {parsed['humidity_pct']:6.2f} % | "
        f"Press: {parsed['pressure_hpa']:7.1f} hPa | "
        f"Batt: {parsed['battery_pct']:3d} % | "
        f"ID: 0x{parsed['device_id']:02X}"
    )


async def main() -> None:
    print(f"Scanning for '{TARGET_NAME}'... (Ctrl+C to stop)\n")
    scanner = BleakScanner(callback)
    await scanner.start()
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        await scanner.stop()
        print("\nScan stopped.")


if __name__ == "__main__":
    asyncio.run(main())

