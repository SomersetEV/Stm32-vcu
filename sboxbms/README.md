# sboxbms: ZombieVerter hardware as a BMW S-Box + SimpBMS

This firmware turns a second ZombieVerter board into the battery's contactor
controller and BMS for VW MEB modules. To the real ZombieVerter it looks like
two devices:

- a **BMW PHEV S-Box**: it takes contactor commands and reports current and voltages;
- a **SimpBMS**: it reports cell voltages, temperatures, current limits and SoC.

The BMS logic (CMU decode, balancing, limits, SoC, sleep) is ported from the
`meb-simpbms` branch of STM32-canbridge.

## Wiring

| Zombie board | Connects to |
|---|---|
| CAN1 (PA11/PA12), 500k | VW MEB CMUs **and** the ISA/IVT shunt |
| CAN2 (PB5/PB6, remapped), 500k | the real ZombieVerter (its `ShuntCan`/`BMSCan` port) |
| `prec_out` (PC6) | precharge contactor |
| `dcsw_out` (PC7) | main positive contactor |
| `gp_out1` (PD15) | main negative contactor |
| 12V | permanent 12V (the board sleeps when the Zombie is quiet) |

The outputs are set in `include/bms_config.h` (`CONT_PRECHARGE`, `CONT_POSITIVE`, `CONT_NEGATIVE`).

**ISA voltage inputs.** The defaults follow the ZombieVerter's own ISA convention:

- **U1** is the inverter side of the contactors.
- **U2** is the battery side.

If your shunt is wired the other way, swap `SHUNT_ID_UOUT` and `SHUNT_ID_UBATT` in `include/bms_config.h`.

## CAN frames

### CAN2 (to and from the real Zombie)

| ID | Direction | Content |
|---|---|---|
| 0x100 | Zombie → box | Contactor command. Byte 0: 0x00 = off, 0xA6 = precharge + negative, 0xAA = run. Byte 3 is the BMW CRC8, which is checked. |
| 0x300 | Zombie → box | Byte 1 must be 0xFE to allow the contactors to close. |
| 0x200 | box → Zombie | Current in mA, int24 LE (the raw ISA value). |
| 0x210 | box → Zombie | Battery-side voltage in mV, int24 LE. Uses the ISA U2 if present, otherwise the sum of the cells. |
| 0x220 | box → Zombie | Inverter-side voltage in mV, int24 LE (ISA U1). Sent as 0 without the ISA, so the Zombie's precharge fails safe. |
| 0x351, 0x355, 0x356, 0x373 | box → Zombie | SimpBMS frames. They stop if any CMU goes quiet for 3 s. |

- **Rates:** the S-Box frames are sent every 20 ms and the SimpBMS frames every 100 ms.
- **Only while the Zombie talks:** they are sent only while the Zombie has been heard in the last second, so the box never keeps a sleeping Zombie awake.
- **Web/SDO access:** the box answers CANopen SDO on CAN2 as node **5**. The Zombie is node 3.

### CAN1

| ID | Content |
|---|---|
| 0x1C0–0x1DF | MEB cell voltages |
| 0x1A5555F4–FB (extended) | MEB module temperatures |
| 0x1A5554xx (extended) | Balancing commands, sent by the box |
| 0x521–0x528 | ISA shunt results (little-endian) |
| 0x411 | ISA configuration, sent by the box when `IsaInit` is set |

## Contactor behaviour

The contactors follow the real Zombie. The box never opens them because of a BMS fault. The Zombie acts on the SimpBMS data itself.

- **Off:** 0x00, no command, or any unknown command byte. All three contactors are open.
- **Precharge:** 0xA6. The negative and precharge contactors are closed.
- **Run:** 0xAA. The positive contactor closes once both are true:
  - precharge has been on for at least `PRECHG_MIN_MS`, and
  - the inverter-side voltage is within `PRECHG_DV_MV` of a plausible battery voltage. This is the same check the Zombie makes.

  The precharge contactor then opens after `PRECHG_OVERLAP_MS`. A 0xAA that arrives while the box is off still goes through precharge first.
- **Timeout:** if no valid 0x100 and 0x300 frames arrive for `CMD_TIMEOUT_MS` (500 ms), everything opens.

## Real ZombieVerter settings

| Parameter | Value |
|---|---|
| `ShuntType` | 2 (SBOX) |
| `ShuntCan` | the port wired to this box's CAN2 |
| `BMS_Mode` | SimpBMS |
| `BMSCan` | the same port |
| `udcmin` | below the pack voltage. The Zombie only updates `udcsw` while `udc2 > udcmin`. |
| `BMS_Timeout` | as usual (SimpBMS data stops when the CMUs go quiet) |

## First boot of the box

1. Flash `stm32_sboxbms.bin` through the usual OpenInverter bootloader or web interface.
2. In the box's web interface or terminal, run `defaults` then `save`. The parameter block left by the VCU firmware does not match this firmware.
3. Run `set IsaInit 1`. This configures the ISA on CAN1 the way the ZombieVerter does: little-endian, cyclic. It takes a few seconds and sets itself back to 0. You only need to do this once per shunt.

The CAN map is never loaded from flash, so a map left behind by the VCU firmware can't send junk on CAN2.

## Sleep

- **When it sleeps:** after `SLEEP_TIMEOUT_MS` (10 s) without Zombie frames on CAN2, and with all contactors open. The box turns balancing off, stops both CAN controllers and enters STOP mode.
- **Watchdog:** an LSI RTC alarm wakes the box every 10 s to feed the watchdog.
- **Wake:**
  - Edge mode: the first CAN2 edge wakes the box.
  - Poll mode: if other nodes keep CAN2 busy while the Zombie sleeps, the box checks every 2 s instead.
  - Every wake starts with a 120 ms check of CAN2. Only a Zombie frame (0x100 or 0x300) wakes the box fully.
- **After a full wake:**
  - Module data restarts, so SimpBMS frames resume once every CMU has reported.
  - SoC keeps counting from the ISA's As counter. It re-anchors to the cell voltage curve only after a sleep of 30 minutes or more.

The board's wifi module and regulators keep drawing current during STOP. Measure the quiescent current before you rely on permanent 12V.

## Building and testing

```
make -C sboxbms          # needs arm-none-eabi-gcc; builds sboxbms/stm32_sboxbms.{bin,hex}
make -C sboxbms test     # host unit tests (any g++)
```

The host tests link the real ZombieVerter S-Box driver (`src/bmw_sbox.cpp`). They check that:

- the emulator reacts correctly to the exact frames the Zombie sends;
- the Zombie's own decoder reads the box's 0x200, 0x210 and 0x220 frames correctly.

## Spot values (web interface)

| Value | Meaning |
|---|---|
| `boxstate` | Contactor state: off, precharge or run |
| `cmdbyte`, `cmdvalid`, `crcerrors` | What the Zombie is commanding |
| `prechg_out`, `pos_out`, `neg_out` | Output states |
| `udc_out`, `udc_batt`, `idc`, `shuntok` | ISA readings |
| `soc`, `umin`, `umax`, `udelta`, `packv`, `tmin`, `tmax`, `chglim`, `dischglim`, `cmusok`, `balancing` | BMS |
| `zombieok` | The Zombie has been heard in the last second |
| `uaux` | 12V supply |

## To check on hardware

- ISA U1/U2 wiring, and the current sign (`SHUNT_SIGN`, `SBOX_CURRENT_SIGN`). Take a candump of 0x521 under a known load.
- That CAN2 receives while CAN1 is held in init mode during the wake check. This is the same open point as on the canbridge.
- Sleep current, how quickly the box wakes on Zombie traffic, and that the watchdog survives sleep.
- The real S-Box's frame rate and unused bytes are not known. The Zombie only reads bytes 0–2 and has no S-Box timeout.
