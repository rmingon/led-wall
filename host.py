# host_send.py
# Raspberry Pi → LED tiles (CH32V006 slave)
# Bus: DATA[0..7] on GPIO, CLK on GPIO (rising edge latch)
# Frame: A5 5A ADDR CMD LEN PAY[LEN] CRC (CRC8 Dallas/Maxim)

import time
import RPi.GPIO as GPIO

# ---------- Pin mapping (BCM) ----------
DATA_PINS = [5, 6, 13, 19, 26, 16, 20, 21]  # D0..D7
CLK_PIN   = 18                               # CLK (rising edge)

# ---------- Protocol constants ----------
SOF0, SOF1 = 0xA5, 0x5A
CMD_SET_ALL_RGB = 0x01
CMD_SET_ONE_RGB = 0x02
CMD_FILL_RGB    = 0x03
CMD_BOOT_TEST   = 0x04

ADDR_BCAST = 0xFF
NUM_LEDS   = 25

# ---------- Low-level bus ----------
def bus_init():
    GPIO.setmode(GPIO.BCM)
    for p in DATA_PINS:
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(CLK_PIN, GPIO.OUT, initial=GPIO.LOW)

def bus_cleanup():
    GPIO.cleanup()

def clock_byte(b: int, t_high=0.000002, t_low=0.000002):
    """Put b on DATA[7:0], then generate a CLK rising edge.
       t_high / t_low can be tuned if needed (seconds)."""
    for i, p in enumerate(DATA_PINS):   # D0..D7
        GPIO.output(p, (b >> i) & 1)
    GPIO.output(CLK_PIN, 1)
    # Short delays keep edges clean; adjust/omit if your wiring is short
    time.sleep(t_high)
    GPIO.output(CLK_PIN, 0)
    time.sleep(t_low)

# ---------- CRC8 Dallas/Maxim (reflected, poly 0x31 → 0x8C) ----------
def crc8_maxim(buf: bytes) -> int:
    crc = 0x00
    for x in buf:
        cur = x
        for _ in range(8):
            mix = (crc ^ cur) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            cur >>= 1
    return crc

# ---------- Framing ----------
def send_frame(addr: int, cmd: int, payload: bytes = b""):
    """Build + transmit one framed packet (address-aware)."""
    if not (0 <= addr <= 0xFF):
        raise ValueError("addr must be 0..255")
    if not (0 <= cmd <= 0xFF):
        raise ValueError("cmd must be 0..255")
    if not (0 <= len(payload) <= 255):
        raise ValueError("payload too long")

    # header + CRC
    header = bytes([addr, cmd, len(payload)])
    c = crc8_maxim(header)
    c ^= crc8_maxim(payload)  # CRC over ADDR..CMD..LEN..PAY
    frame = bytes([SOF0, SOF1]) + header + payload + bytes([c])

    # transmit
    for b in frame:
        clock_byte(b)

# ---------- High-level helpers ----------
def fill(addr: int, r: int, g: int, b: int):
    send_frame(addr, CMD_FILL_RGB, bytes([r & 0xFF, g & 0xFF, b & 0xFF]))

def set_one(addr: int, index: int, r: int, g: int, b: int):
    if not (0 <= index < NUM_LEDS):
        raise ValueError("index 0..24")
    send_frame(addr, CMD_SET_ONE_RGB, bytes([index & 0xFF, r & 0xFF, g & 0xFF, b & 0xFF]))

def set_all(addr: int, rgb_triples):
    """rgb_triples: iterable of (r,g,b), length == NUM_LEDS"""
    if len(rgb_triples) != NUM_LEDS:
        raise ValueError(f"need {NUM_LEDS} RGB tuples")
    pay = bytearray()
    for (r, g, b) in rgb_triples:
        pay += bytes([r & 0xFF, g & 0xFF, b & 0xFF])
    send_frame(addr, CMD_SET_ALL_RGB, bytes(pay))

def boot_test(addr: int):
    send_frame(addr, CMD_BOOT_TEST, b"")

# ---------- Example usage ----------
if __name__ == "__main__":
    try:
        bus_init()

        # broadcast fill to all tiles:
        fill(ADDR_BCAST, 0, 0, 0)                   # all off
        time.sleep(0.05)
        fill(ADDR_BCAST, 20, 0, 0)                  # dim red
        time.sleep(0.2)
        fill(ADDR_BCAST, 0, 20, 0)                  # dim green
        time.sleep(0.2)
        fill(ADDR_BCAST, 0, 0, 20)                  # dim blue
        time.sleep(0.2)

        # unicast examples:
        TILE = 0x12                                  # your node address
        boot_test(TILE)                              # run the slave's test
        time.sleep(0.5)

        # set one pixel on TILE to white:
        set_one(TILE, index=7, r=60, g=60, b=60)

        # set all pixels on TILE to a gradient:
        grad = [(i*10 % 256, i*5 % 256, i*3 % 256) for i in range(NUM_LEDS)]
        set_all(TILE, grad)

    finally:
        bus_cleanup()