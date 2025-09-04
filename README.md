# LED-Wall Tile — CH32V003 Parallel-Bus WS2812B Controller (5×5)

A modular **5×5 WS2812B LED tile** driven by a **CH32V003** slave MCU.  
Tiles share a simple **8-bit parallel bus + CLK + EN** so a Raspberry Pi (or any MCU) can address many tiles and push pixel data quickly.

---

## Highlights

- **25× WS2812B** per tile (5 rows × 5 columns), daisy-chained on-board
- **Parallel control bus:** `DATA0…DATA7`, `CLK`, `EN`, optional `RST`
- **MCU:** WCH **CH32V003F…** (RISC-V, 3.3 V)
- **Power:** 5 V input for LEDs; onboard **AMS1117-3.3** LDO for the MCU
- **Connectors on four edges** for tiling/rotation; pass-through for power & bus
- **Status LED** on a GPIO for quick diagnostics
- Designed with **KiCad 9.0.3**

---

## Repo structure

```
/wall/
  wall.kicad_pro
  wall.kicad_pcb
  wall.kicad_sch
  wall.kicad_prl
/media/
  wall_top_3d.jpeg
  wall_bottom_3d.jpeg
  wall_pcb.png
  wall_schematic.png
/firmware/        # (placeholder) CH32V003 code
/tools/           # (placeholder) host examples (Raspberry Pi)
README.md
```

---

## Images

- Top 3D: `media/wall_top_3d.jpeg`
- Bottom 3D: `media/wall_bottom_3d.jpeg`
- PCB layout: `media/wall_pcb.png`
- Schematic: `media/wall_schematic.png`

---

## Electrical overview

- **LED matrix:** 25 × WS2812B (5050). Data is routed serpentine per row.
- **MCU:** `CH32V003F*P6` (labelled **U1** in the schematic).
  - Parallel data bus on **PC0…PC7**
  - `CLK` and `EN` on PDx pins (see net labels in schematic)
  - `PA1/PA2` used for the **WS2812** data output to the first LED in the chain
  - `RST` net broken out (optional external reset)
- **Power:**
  - **+5 V** for LEDs
  - **AMS1117-3.3** (U3) generates **+3V3** for the MCU; C1/C2/C3 for decoupling
- **Indicators:** D26 + R4 connected to a GPIO (status / heartbeat)

> ⚠️ **Current budget:** Each WS2812B can draw up to **40 mA** at full-white.  
> One tile worst-case ≈ **25 × 40 mA = 1.5 A @ 5 V** (plus margin).  
> Size your power rails accordingly and avoid sustained full-white if thermals are tight.

---

## Connectors / Pinout

Each edge header is a **1×14** connector carrying power and the parallel bus.

| Pin | Signal | Notes                              |
| --: | :----- | :--------------------------------- |
|   1 | DATA0  | Parallel data bit 0 (MCU **PC0**)  |
|   2 | DATA1  | (PC1)                              |
|   3 | DATA2  | (PC2)                              |
|   4 | DATA3  | (PC3)                              |
|   5 | DATA4  | (PC4)                              |
|   6 | DATA5  | (PC5)                              |
|   7 | DATA6  | (PC6)                              |
|   8 | DATA7  | (PC7)                              |
|   9 | CLK    | Rising edge latch (EXTI7_0 IRQ)    |
|  10 | EN     | Tile enable / latch (PDx)          |
|  11 | GND    | Ground                             |
|  12 | GND    | Ground (second pin for current)    |
|  13 | +5V    | LED power                          |
|  14 | +5V    | LED power (second pin for current) |

> Exact PDx assignments are labelled in the schematic nets; keep those in firmware defines so you can move them if the footprint changes.

- WS2812 data → **PC7 / TIM2_CH2 (remap)**
- Driven by **TIM2 PWM + DMA**

---

## Protocol

Each frame is transmitted as a sequence of **bytes** clocked on **PD2 rising edge** while the slave samples D0–D7.

**Frame format:**

```
SOF0 SOF1 ADDR CMD LEN PAY[LEN] CRC
```

- **SOF0** = `0xA5`
- **SOF1** = `0x5A`
- **ADDR** = 0–254 for unicast, `0xFF` for broadcast
- **CMD** = command byte
- **LEN** = number of payload bytes
- **PAY** = payload
- **CRC** = Dallas/Maxim CRC8 (poly=0x31 reflected=0x8C, init=0x00) computed over `ADDR..CMD..LEN..PAY`

### Commands

| CMD  | Name          | LEN      | Payload description             |
| ---- | ------------- | -------- | ------------------------------- |
| 0x01 | `SET_ALL_RGB` | 3×N (75) | [R,G,B] triplets for all LEDs   |
| 0x02 | `SET_ONE_RGB` | 4        | [index, R, G, B] (0–24 index)   |
| 0x03 | `FILL_RGB`    | 3        | [R, G, B] fill all LEDs         |
| 0x04 | `BOOT_TEST`   | 0        | Run built-in RGB test animation |

---

## Firmware (CH32V003)

- **Toolchain:** WCH-RISC-V (MounRiver) or Open-source (e.g., `riscv-none-elf-gcc` + `openocd` for WCH-LinkE).
- **Peripherals used:**
  - GPIO for `DATA[7:0]`, `CLK`, `EN`, `LED`
  - Timer or edge-detect to sample `CLK` while `EN=1`
  - Bit-banged or timer-driven WS2812 waveform output on `PAx`
- **WS2812 timing:** 800 kHz, 1.25 µs/bit. Use a timer and tight ISR/DMA-like loop to meet timing.

**Build steps (outline):**

1. Clone repo; open `/kicad/wall.kicad_pro` to view hardware.
2. Open `/wall_code/` in your IDE; set your tile address in `main.c`.
3. Flash with **WCH-LinkE** via **SWIO** (PD1 on CH32V003). Provide a small 3-pin tag-connector or pogo-pads on your programmer.
4. Power at 5 V and verify the heartbeat LED.

---

## Example Host Frame

Set all 25 LEDs to **red (255,0,0)**:

```
A5 5A 12 01 4B  FF 00 00 FF 00 00 ... (25 triplets) ... CRC
```

- `0xA5 0x5A` = SOF
- `0x12` = target address
- `0x01` = `SET_ALL_RGB`
- `0x4B` = 75 payload bytes
- Payload = 25× [FF,00,00]
- `CRC` = computed over ADDR..PAYLOAD

## Raspberry Pi example (Python, pseudo-code)

2. Open `host.py` in your IDE.

## Example Host Frame

Set all 25 LEDs to **red (255,0,0)**:

```
A5 5A 12 01 4B  FF 00 00 FF 00 00 ... (25 triplets) ... CRC
```

- `0xA5 0x5A` = SOF
- `0x12` = target address
- `0x01` = `SET_ALL_RGB`
- `0x4B` = 75 payload bytes
- Payload = 25× [FF,00,00]
- `CRC` = computed over ADDR..PAYLOAD

---

## Manufacturing notes

- **Layers:** 2-layer, 1.6 mm FR-4, 1 oz Cu
- **Min track/clearance:** per your fab defaults (design uses moderate widths)
- **Finish:** HASL or ENIG
- **Decoupling:** place 0.1 µF near the MCU VDD and each LED power rail segment
- **Mounting:** corner holes + through-grid holes in the LED field (for diffusion/weight).

---

## Bring-up checklist

1. Power at **5 V** and check **+3V3** out of U3 (AMS1117-3.3).
2. Flash firmware; confirm status LED blinks.
3. With a logic analyzer, verify `CLK` edges and that `DATA[7:0]` are stable at each edge while `EN=1`.
4. Send a **Fill** command to a single address → tile lights uniformly.
5. Push a full **75-byte** pixel buffer → verify 5×5 order matches your mapping.

---

## License

- Hardware: CC-BY-SA 4.0 (or specify your preference)
- Firmware/Host code: MIT (or your preference)

---

## Credits

Designed by **RMINGON**.  
KiCad project files: `kicad/wall.*` (created with KiCad **9.0.3**).

---

### Bill-of-Materials (core)

| Ref     | Part                  | Notes                                              |
| ------- | --------------------- | -------------------------------------------------- |
| U1      | **CH32V003F…P6**      | 3.3 V RISC-V MCU                                   |
| U3      | **AMS1117-3.3**       | 5 V → 3.3 V LDO                                    |
| D1–D25  | **WS2812B**           | 5 V addressable LEDs                               |
| D26, R4 | Status LED + 5 Ω–1 kΩ | (value per brightness; schematic shows **R4 5 Ω**) |
| J1–J4   | 1×14 headers          | Bus + Power                                        |
| C1–C3   | 10 µF/0.1 µF mix      | Regulator & MCU decoupling                         |

> Tweak values to match your inventory and regulator stability requirements (e.g., AMS1117 likes ≥10 µF on input/output).
