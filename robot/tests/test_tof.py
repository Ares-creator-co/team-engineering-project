# Thonny compatible hopefully to check the wiring of the three ToF sensors
# the wiring checks work without a driver, the live readings need vl53l4cd.py (or vl53l0x.py) on the Pico
from machine import Pin, I2C
import time

I2C_ID = 0
I2C_SDA_PIN = 0
I2C_SCL_PIN = 1
I2C_FREQ = 400_000

DEFAULT_ADDRESS = 0x29
READINGS = 40

# name, XSHUT pin, new address
SENSORS = (
    ("front", 10, 0x30),
    ("left", 11, 0x31),
    ("right", 12, 0x32),
)


def identify():
    # each chip keeps its model ID in a different register
    try:
        if i2c.readfrom_mem(DEFAULT_ADDRESS, 0xC0, 1)[0] == 0xEE:
            return "VL53L0X"
    except OSError:
        pass
    try:
        i2c.writeto(DEFAULT_ADDRESS, b"\x01\x0f")
        if i2c.readfrom(DEFAULT_ADDRESS, 2) == b"\xeb\xaa":
            return "VL53L4CD"
    except OSError:
        pass
    return "unknown"


def bring_up(name, xshut, address):
    # powers up one sensor and moves it to its own address, returns its model or None
    xshut.value(1)
    time.sleep_ms(50)
    if DEFAULT_ADDRESS not in i2c.scan():
        print("FAIL", name, "- not found, check its VIN, GND, SDA, SCL and XSHUT wires")
        xshut.value(0)
        return None
    model = identify()
    print("PASS", name, "- found, model:", model)
    if model == "unknown":
        xshut.value(0)
        return model
    # each chip keeps its address in a different register
    if model == "VL53L4CD":
        i2c.writeto_mem(DEFAULT_ADDRESS, 0x0001, bytes([address & 0x7F]), addrsize=16)
    else:
        i2c.writeto_mem(DEFAULT_ADDRESS, 0x8A, bytes([address & 0x7F]))
    time.sleep_ms(10)
    if address in i2c.scan():
        print("PASS", name, "- moved to", hex(address))
        return model
    print("FAIL", name, "- did not move to", hex(address))
    xshut.value(0)
    return None


def read_mm(device):
    # driver libraries name this differently
    if hasattr(device, "read"):
        return device.read()
    return device.ping()	


def open_device(model, address):
    if model == "VL53L4CD":
        from vl53l4cd import VL53L4CD
        return VL53L4CD(i2c, address=address)
    from vl53l0x import VL53L0X
    return VL53L0X(i2c, address=address)


def live_readings(ready):
    try:
        devices = [(name, open_device(model, address)) for name, address, model in ready]
    except ImportError as error:
        print("Wiring checks done. Save the driver on the Pico to see live readings:", error)
        return
    print("Move your hand in front of each sensor, distances are in mm, None means no target")
    for i in range(READINGS):
        print("  ".join(name + ": " + str(read_mm(device)) for name, device in devices))
        time.sleep_ms(250)


def main():
    # all sensors off first, otherwise they clash on the same address
    xshuts = [Pin(pin, Pin.OUT, value=0) for name, pin, address in SENSORS]
    time.sleep_ms(50)

    found = i2c.scan()
    print("With all sensors off, the I2C scan found:", [hex(a) for a in found])
    if found:
        print("FAIL - something answered while all sensors were off, check the XSHUT wires")
        return

    ready = []
    models = []
    for (name, pin, address), xshut in zip(SENSORS, xshuts):
        model = bring_up(name, xshut, address)
        if model:
            models.append(model)
        if model in ("VL53L0X", "VL53L4CD"):
            ready.append((name, address, model))

    print(len(models), "/", len(SENSORS), "sensors found")
    if "VL53L4CD" in models:
        print("These are VL53L4CD sensors, so mindmap.py and vl53l0x.py will not work with them")
    if ready:
        live_readings(ready)


i2c = I2C(I2C_ID, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN), freq=I2C_FREQ)
main()