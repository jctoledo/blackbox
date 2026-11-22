# Android USB serial adapter

`usb-serial.mjs` bundles the unmodified USB serial drivers from
[ESP Web Tools](https://github.com/esphome/esp-web-tools/tree/8db1c6f565e10b1dff028ba8c29dd24ce69f049b/src/util/usb-serial),
revision `8db1c6f565e10b1dff028ba8c29dd24ce69f049b`, under Apache-2.0
(see `usb-serial.LICENSE`). Drivers cover CDC-ACM (including C3 native USB),
CP210x, CH340/CH341 and FTDI. Their presence is not physical validation of
every board, Android phone or USB host implementation.

Rebuild on **lat** with `python3 scripts/vendor_usb_serial.py` (Node/npm and
network access required). The script pins upstream source and esbuild, bundles
all drivers into one local module, and retains the upstream license. There are
no runtime CDN requests for USB drivers. Re-run the flasher regressions after
updating either pin. Existing desktop flashing still uses its pinned toolkit.
