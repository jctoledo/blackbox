# Build a Blackbox

The current build combines an ESP32-C3, a WT901 UART IMU, a supported u-blox GPS receiver and a status LED. Start on the bench, establish reliable sensor data, then mount it in the car. The firmware image and actual board wiring must agree.

[How the measurements work](ENGINEERING.md) · [Hardware acceptance checklist](HARDWARE_VALIDATION.md) · [Project hardware and wiring](https://github.com/jctoledo/blackbox#hardware)

## 1. Choose and identify the parts

| Part | Record before assembly |
| --- | --- |
| ESP32-C3 development board | Exact board revision, flash capacity and whether USB uses a bridge or native GPIO18/19 |
| WT901 UART IMU | Module variant, supply requirements, output rate, baud and enabled packet types |
| NEO-M9N or NEO-6M GPS | Module/antenna details and matching firmware variant; M9N target 25 Hz, 6M target 5 Hz |
| WS2812 status LED | Board-integrated or external, matching the firmware's GPIO8 assignment |
| Power, wiring and enclosure | Actual supply/logic voltages, connector retention, insulation and cable strain relief |

The current merged firmware is for **ESP32-C3**. The ESP32-S3 schematic proposal is a different hardware design and needs its own port and electrical validation. Do not treat an S3 board as an interchangeable controller.

## 2. Wire against the actual board

The current default signal mapping is:

| Signal source | Signal destination |
| --- | --- |
| GPS TX | ESP32-C3 GPIO4 / UART RX |
| ESP32-C3 GPIO5 / UART TX | GPS RX |
| WT901 TX | ESP32-C3 GPIO19 / UART RX |
| ESP32-C3 GPIO18 / UART TX | WT901 RX |
| ESP32-C3 GPIO8 | Status LED data input |

Check the project's [hardware mapping](https://github.com/jctoledo/blackbox#hardware) and your module documentation for power and ground connections. This table describes signal routing, not voltage compatibility or a complete electrical schematic.

**GPIO18/19 are also native USB pins on ESP32-C3.** The default IMU assignment must not share those signals with an active native USB connection. Establish which USB arrangement your board uses before wiring; a pin-map change must be made in both firmware and wiring. A generic board name is not sufficient to settle compatibility.

## 3. Prove the bench setup

Flash the matching GPS image using a data cable. Record the firmware version and SHA-256. Keep the sensor stationary through startup calibration, connect the phone to the device's Wi-Fi and open its dashboard. Outdoors, check that GPS fixes arrive and inspect actual delivered rates in Diagnostics.

Confirm that acceleration, gyro and attitude packets all remain fresh. Exercise a Wi-Fi disconnect and reconnect. Record a short disposable session, export it, reload the page and confirm the saved session remains available. Follow the detailed [bench checklist](HARDWARE_VALIDATION.md#1-usb-installation-and-recovery-bench).

## 4. Mount the sensor, antenna and display

Secure the sensor and document its axes before learning alignment. A moving enclosure changes the relationship between its measurements and the vehicle. Provide cable strain relief and a suitable view of the sky for the GPS antenna; document the actual installed locations so later recordings can be interpreted.

Mount the phone where it can be read without obstructing the driver's view or vehicle controls. Set up timing and recording while parked. The illustrated [sensor-to-car alignment instructions](CALIBRATION.md) explain which rotation is learned and why it differs from stationary bias calibration.

Use level ground for the initial baseline. The revised calibration arithmetic handles modest inclines in software tests, but the physical driveway comparison remains part of acceptance. An enclosure or mounting change deserves a repeat check.

## 5. Keep a build journal

This section is the home for the project's real assembly and installation photos as they are collected. Each image should explain something a builder can reproduce, rather than merely show a finished box.

| Photo | Caption should identify |
| --- | --- |
| Parts laid out | Board/module revisions, antenna and power arrangement |
| Wiring before enclosure | Pin labels, connector orientation and USB arrangement |
| Enclosure open | Mounting, insulation and strain relief |
| Installed sensor and antenna | Sensor axes, car-forward direction and antenna location |
| Driver's display | Phone/orientation and actual firmware version |
| First verified session | Conditions, exported evidence and what was checked |

Keep original images separately. Add web-sized copies under `docs/assets/build/` and link them here with descriptive captions and alternative text; the website's Build section links directly to this journal. Do not publish private routes, identifying paperwork or Wi-Fi credentials visible in the background.

## First drive

Start with an ordinary drive after the bench and axis checks. Record a parked segment before and after it, then export Session CSV, GPS CSV and a full backup. Keep notes on the hardware, mounting, phone/browser and observed problems. No performance manoeuvres are needed to collect useful initial data.

Controlled line crossings with an independent reference come later. Use [the acceptance checklist](HARDWARE_VALIDATION.md) to separate “boots and records,” “detects the intended crossings,” and “meets a measured timing-error budget.”
