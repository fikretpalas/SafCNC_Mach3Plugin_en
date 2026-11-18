# SafCNC Mach3 Plugin

A Mach3 motion plugin that drives a CNC machine through an **Arduino Uno / Nano** over USB.
The protocol is not tied to the Arduino: an STM32 card could reach about 100 kHz (not tested, see below).
Mach3 keeps planning the trajectory; the plugin streams it to the card in 5 ms slices and the
card generates the step and direction signals with a 32 kHz timer.

Written in **Free Pascal (Lazarus)** as a plain Win32 DLL - no LCL, no external libraries.

| | |
|---|---|
| Firmware for the card | [SafCNC_Arduino](https://github.com/fikretpalas/SafCNC_Arduino) |
| Turkish edition of this plugin | [SafCNC_Mach3Plugin_tr](https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr) |

## How it works

```
 Mach3 (32-bit)                         SafCNC.dll                               Arduino Uno / Nano
+------------------+   trajectory   +--------------------------------+   USB    +-----------------------+
| G-code           |    points      | uDevice      Mach3 events, jog |  serial  | ring buffer 200 slices|
| trajectory       | -------------> | uCardLayer   slices, position  | 115200   | 32 kHz DDA step timer |
| planner, DROs    | <------------- | uSerialLink  threads, packets  | <------> | limits, probe, E-Stop |
+------------------+  position,     +--------------------------------+  packets +-----------------------+
                      inputs                                                      step / dir / enable
```

* Mach3 plans the move and fills its trajectory buffer.
* `uDevice` takes the points, `uCardLayer` converts each 5 ms slice into whole steps per axis
  (fractions are carried over, so no step is lost) and `uSerialLink` queues them as packets.
* A reader thread and a writer thread talk to the card with overlapped I/O, so Mach3 never waits for USB.
* The card answers with a status packet every 50 ms: state, buffer level, outputs and inputs.
* After any stop the card reports the slice and sub-tick where motion ended, and the plugin corrects the
  Mach3 position to the steps that were really executed.

## Features

* 3 axes (X, Y, Z); the card reports its own speed limit: 160 steps per 5 ms slice = 32 kHz per axis
  on an Arduino, at most 500 = 100 kHz
* Automatic card detection on any COM port, USB serial ports first; hot-plug and reconnect
* Jogging with acceleration and a short card buffer, so the machine stops quickly on key release
* G4 dwell, spindle on / direction, flood and mist outputs
* Limit, probe, Feed Hold, Cycle Start and E-Stop inputs mapped to the Mach3 input signals
* Status window under *PlugIn Control > SafCNC* with a link lamp, port and baud rate
* Messages on the Mach3 status line for every connection and error event
* Nothing is written to the registry; the plugin keeps no settings of its own

## Safety design

* Every packet carries a sequence counter and a 16-bit checksum.
* The card stops by itself if the link is silent for 200 ms; the plugin applies the same timeout.
* Limit switches, probe and the card's own E-Stop input are handled on the card, independently of the PC.
* Any card or link error puts Mach3 into reset and shows the reason.
* If the *Motor Tuning* velocities exceed the speed the card reports, a warning shows each axis with its
  highest allowed velocity, and the card is not linked until they are lowered. The plugin does not
  change the Mach3 settings itself.
* Jogging stops when Mach3 loses the keyboard focus, when opposite keys are pressed, or when no key or
  mouse button has been held for 30 ms.
* Clicking the Mach3 title bar, window frame or menu bar while jogging makes Windows block the Mach3
  window, and dragging would move it. While a jog key is held or the jog is still running, such a click
  is not passed on to Windows: the window does not move, the machine decelerates and stops smoothly, and
  jogging stays locked until every jog key has been released.
* If a key is pressed while the window is being dragged by its title bar, the window is put back where it
  was, the move is cancelled and jogging stays locked until all keys are released.
* If Mach3 is left in the "jogging" state with no jog running (for example after a jog key was pressed
  inside a text box), the plugin clears that state once the machine is at rest, so no reset is needed.
* At start-up the plugin verifies the Mach3 memory layout and disables itself if it does not match.

> A CNC machine can injure people and destroy tools. Always fit a hardware E-Stop that cuts motor and
> spindle power. This software is provided without any warranty.

## Source files

| File | Purpose |
|---|---|
| `SafCNC.lpr` | Library entry point, functions exported to Mach3, menu hook |
| `uDevice.pas` | Top-level controller: motion, jog, dwell, probe and reset events |
| `uCardLayer.pas` | Slices, position counting, outputs, alarms, connection state |
| `uSerialLink.pas` | COM port search, reader / writer threads, packet checks, watchdog |
| `uCardProtocol.pas` | Packet format shared with the firmware |
| `uMainProgram.pas` | Access to the Mach3 Engine / MainPlanner blocks and callbacks |
| `uCardWindow.pas` | Status window drawn with the Win32 API |
| `uMessages.pas` | All user-visible texts |
| `uWinApi.pas` | Win32 API declarations |

## Build

Mach3 is a 32-bit program, so the DLL **must** be built for `i386-win32`.

1. Install [Lazarus](https://www.lazarus-ide.org/) with Free Pascal 3.2.x. On 64-bit Lazarus add the
   `i386-win32` cross compiler, or simply use the 32-bit Lazarus installer.
2. Open `SafCNC.lpi`. The project is already set to CPU `i386`, OS `win32`, executable type *Library*.
3. *Run > Build* produces `SafCNC.dll`.

A 64-bit build is rejected at compile time by a `{$FATAL}` check.

## Install

1. Upload the firmware from [SafCNC_Arduino](https://github.com/fikretpalas/SafCNC_Arduino) to the card.
2. Copy `SafCNC.dll` into the `Mach3\PlugIns` folder.
3. Start Mach3. If it asks which motion control device to use, choose SafCNC.
4. Leave reset with the Mach3 *Reset* button; the status line shows `SafCNC card connected.`

## Mach3 Ports & Pins

The card appears to Mach3 as **port 3**.

| Input pin | Signal on the card | | Output pin | Signal on the card |
|---|---|---|---|---|
| 1 | X limit (D9) | | 1 | Spindle enable (D11) |
| 2 | Y limit (D10) | | 2 | Spindle direction (D13) |
| 3 | Z limit (D12) | | 3 | Flood coolant (A3) |
| 4 | Feed Hold (A1) | | 4 | Mist coolant (A4) |
| 5 | Cycle Start (A2) | | | |
| 6 | E-Stop (A0) | | | |
| 7 | Probe (A5) | | | |

Step and direction pins are fixed on the card (GRBL 1.1 / CNC Shield V3 layout) and need no Mach3 pin
assignment. Steps per unit, velocity and acceleration are taken from *Motor Tuning*. While no card
is connected, Mach3 is given the protocol ceiling (100 kHz), so the velocities can be set up to it;
once a card is connected, its own limit applies.

## Protocol in brief

| Direction | Packet | Size | Content |
|---|---|---|---|
| PC -> card | state (`K_RESET`, `K_READY`, `K_ACTIVE`) | 8 bytes | command, counter, reset reason or spindle + outputs, checksum |
| PC -> card | move (`K_MOVE`) | 10 bytes | command, counter, X / Y / Z steps for the next 5 ms, checksum |
| card -> PC | status | 8 bytes | state, counter, buffer level, spindle, outputs, inputs, checksum |
| card -> PC | info (`K_INFO`), at power-up and on a new connection | 8 bytes | command, counter, step limit per slice (16 bit), sub-ticks per slice (16 bit), checksum |

The checksum makes the 16-bit sum of all bytes zero. The card reports its step limit in `K_INFO`; the
other constants live in `uCardProtocol.pas` and must match the firmware.

## Other cards (STM32)

The plugin is not limited to the Arduino. It talks to any card that implements the same packet
protocol, and the design leaves room for faster hardware: an STM32 card could run at about
**100 kHz** per axis (500 steps per 5 ms slice).

* An Arduino Uno / Nano cannot go beyond about 30 kHz; the higher speed is for STM32 hardware.
* The STM32 side has **not been tested** yet.
* The card reports its working speed in the `K_INFO` packet; the plugin plans the slices for that
  speed and never goes above 500 steps per slice.

## Current limits

* Three axes; the A axis is not driven.
* The spindle output is on / off with direction; speed is not controlled from Mach3.
* Probing moves (G31) and homing are not executed by the card in this version.

## Developer

**Fikret Palas** - <safcnc@tutamail.com> - <https://github.com/fikretpalas>

## License

[MIT](LICENSE)
