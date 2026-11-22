// ESP Web Tools USB serial adapter, Apache-2.0; upstream 8db1c6f565e10b1dff028ba8c29dd24ce69f049b. See usb-serial.LICENSE and README.md.
var __defProp = Object.defineProperty;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __defNormalProp = (obj, key, value) => key in obj ? __defProp(obj, key, { enumerable: true, configurable: true, writable: true, value }) : obj[key] = value;
var __esm = (fn, res) => function __init() {
  return fn && (res = (0, fn[__getOwnPropNames(fn)[0]])(fn = 0)), res;
};
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};
var __publicField = (obj, key, value) => __defNormalProp(obj, typeof key !== "symbol" ? key + "" : key, value);

// driver.ts
var matchesFilter, matchesDefinition, findInterface, findBulkEndpoints, controlOut, controlIn;
var init_driver = __esm({
  "driver.ts"() {
    matchesFilter = (device, filter) => {
      if (filter.vendorId !== void 0 && filter.vendorId !== device.vendorId) {
        return false;
      }
      if (filter.productId !== void 0 && filter.productId !== device.productId) {
        return false;
      }
      if (filter.classCode === void 0) {
        return true;
      }
      if (device.deviceClass === filter.classCode) {
        return true;
      }
      return device.configurations.some(
        (config) => config.interfaces.some(
          (iface) => iface.alternates.some((alt) => alt.interfaceClass === filter.classCode)
        )
      );
    };
    matchesDefinition = (device, definition) => definition.filters.some((f) => matchesFilter(device, f));
    findInterface = (device, classCode) => device.configuration?.interfaces.find(
      (iface) => iface.alternates[0]?.interfaceClass === classCode
    );
    findBulkEndpoints = (iface) => {
      const endpoints = iface.alternates[0].endpoints.filter(
        (ep) => ep.type === "bulk"
      );
      const inEp = endpoints.find((ep) => ep.direction === "in");
      const outEp = endpoints.find((ep) => ep.direction === "out");
      if (!inEp || !outEp) {
        throw new Error(
          `Interface ${iface.interfaceNumber} has no bulk IN and OUT endpoints`
        );
      }
      return { in: inEp, out: outEp };
    };
    controlOut = async (device, setup, data) => {
      const result = await device.controlTransferOut(setup, data);
      if (result.status !== "ok") {
        throw new Error(
          `USB control request 0x${setup.request.toString(16)} failed: ${result.status}`
        );
      }
    };
    controlIn = async (device, setup, length) => {
      const result = await device.controlTransferIn(setup, length);
      if (result.status !== "ok" || !result.data) {
        throw new Error(
          `USB control request 0x${setup.request.toString(16)} failed: ${result.status}`
        );
      }
      return result.data;
    };
  }
});

// cp210x.ts
var cp210x_exports = {};
__export(cp210x_exports, {
  cp210xLineControl: () => cp210xLineControl,
  cp210xModemControl: () => cp210xModemControl,
  createCp210xDriver: () => createCp210xDriver
});
var VENDOR_CLASS, IFC_ENABLE, SET_LINE_CTL, SET_BREAK, SET_MHS, SET_BAUDRATE, UART_ENABLE, UART_DISABLE, MHS_DTR, MHS_RTS, MHS_WRITE_DTR, MHS_WRITE_RTS, cp210xLineControl, cp210xModemControl, Cp210xDriver, createCp210xDriver;
var init_cp210x = __esm({
  "cp210x.ts"() {
    init_driver();
    VENDOR_CLASS = 255;
    IFC_ENABLE = 0;
    SET_LINE_CTL = 3;
    SET_BREAK = 5;
    SET_MHS = 7;
    SET_BAUDRATE = 30;
    UART_ENABLE = 1;
    UART_DISABLE = 0;
    MHS_DTR = 1;
    MHS_RTS = 2;
    MHS_WRITE_DTR = 256;
    MHS_WRITE_RTS = 512;
    cp210xLineControl = (config) => {
      const parity = { none: 0, odd: 16, even: 32 }[config.parity];
      const stop = config.stopBits === 2 ? 2 : 0;
      return config.dataBits << 8 | parity | stop;
    };
    cp210xModemControl = (dtr, rts) => MHS_WRITE_DTR | MHS_WRITE_RTS | (dtr ? MHS_DTR : 0) | (rts ? MHS_RTS : 0);
    Cp210xDriver = class {
      constructor(_device) {
        this._device = _device;
        __publicField(this, "_interfaceNumber", 0);
      }
      _out(request, value, data) {
        return controlOut(
          this._device,
          {
            requestType: "vendor",
            recipient: "interface",
            request,
            value,
            index: this._interfaceNumber
          },
          data
        );
      }
      async open() {
        const iface = findInterface(this._device, VENDOR_CLASS);
        if (!iface) {
          throw new Error("No vendor specific interface found");
        }
        this._interfaceNumber = iface.interfaceNumber;
        await this._device.claimInterface(iface.interfaceNumber);
        await this._out(IFC_ENABLE, UART_ENABLE);
        return findBulkEndpoints(iface);
      }
      async setLineConfig(config) {
        const baud = new Uint8Array(4);
        new DataView(baud.buffer).setUint32(0, config.baudRate, true);
        await this._out(SET_BAUDRATE, 0, baud);
        await this._out(SET_LINE_CTL, cp210xLineControl(config));
      }
      setSignals(dataTerminalReady, requestToSend) {
        return this._out(
          SET_MHS,
          cp210xModemControl(dataTerminalReady, requestToSend)
        );
      }
      setBreak(enabled) {
        return this._out(SET_BREAK, enabled ? 1 : 0);
      }
      async close() {
        await this._out(IFC_ENABLE, UART_DISABLE);
      }
    };
    createCp210xDriver = (device) => new Cp210xDriver(device);
  }
});

// ch34x.ts
var ch34x_exports = {};
__export(ch34x_exports, {
  ch34xBaudDivisor: () => ch34xBaudDivisor,
  ch34xLineControl: () => ch34xLineControl,
  ch34xModemControl: () => ch34xModemControl,
  createCh34xDriver: () => createCh34xDriver
});
var VENDOR_CLASS2, REQ_READ_VERSION, REQ_WRITE_REG, REQ_SERIAL_INIT, REQ_MODEM_CTRL, BIT_DTR, BIT_RTS, LCR_ENABLE_RX, LCR_ENABLE_TX, LCR_PAR_EVEN, LCR_ENABLE_PAR, LCR_STOP_BITS_2, CLOCK_RATE, MIN_BPS, MAX_BPS, clockDivisor, MIN_RATES, ch34xBaudDivisor, ch34xLineControl, ch34xModemControl, Ch34xDriver, createCh34xDriver;
var init_ch34x = __esm({
  "ch34x.ts"() {
    init_driver();
    VENDOR_CLASS2 = 255;
    REQ_READ_VERSION = 95;
    REQ_WRITE_REG = 154;
    REQ_SERIAL_INIT = 161;
    REQ_MODEM_CTRL = 164;
    BIT_DTR = 1 << 5;
    BIT_RTS = 1 << 6;
    LCR_ENABLE_RX = 128;
    LCR_ENABLE_TX = 64;
    LCR_PAR_EVEN = 16;
    LCR_ENABLE_PAR = 8;
    LCR_STOP_BITS_2 = 4;
    CLOCK_RATE = 48e6;
    MIN_BPS = 46;
    MAX_BPS = 3e6;
    clockDivisor = (ps, fact) => 1 << 12 - 3 * ps - fact;
    MIN_RATES = [0, 1, 2, 3].map(
      (ps) => Math.floor(CLOCK_RATE / (clockDivisor(ps, 1) * 512))
    );
    ch34xBaudDivisor = (baudRate) => {
      const speed = Math.min(Math.max(baudRate, MIN_BPS), MAX_BPS);
      let fact = 1;
      let ps = 3;
      while (ps > 0 && speed <= MIN_RATES[ps]) {
        ps--;
      }
      let clkDiv = clockDivisor(ps, fact);
      let div = Math.floor(CLOCK_RATE / (clkDiv * speed));
      if (div < 9 || div > 255) {
        div = Math.floor(div / 2);
        clkDiv *= 2;
        fact = 0;
      }
      if (16 * CLOCK_RATE / (clkDiv * div) - 16 * speed >= 16 * speed - 16 * CLOCK_RATE / (clkDiv * (div + 1))) {
        div++;
      }
      if (fact === 1 && div % 2 === 0) {
        div /= 2;
        fact = 0;
      }
      return 256 - div << 8 | fact << 2 | ps;
    };
    ch34xLineControl = (config) => {
      let lcr = LCR_ENABLE_RX | LCR_ENABLE_TX | config.dataBits - 5;
      if (config.parity !== "none") {
        lcr |= LCR_ENABLE_PAR;
        if (config.parity === "even") {
          lcr |= LCR_PAR_EVEN;
        }
      }
      if (config.stopBits === 2) {
        lcr |= LCR_STOP_BITS_2;
      }
      return lcr;
    };
    ch34xModemControl = (dtr, rts) => ~((dtr ? BIT_DTR : 0) | (rts ? BIT_RTS : 0)) & 65535;
    Ch34xDriver = class {
      constructor(_device) {
        this._device = _device;
      }
      _out(request, value, index) {
        return controlOut(this._device, {
          requestType: "vendor",
          recipient: "device",
          request,
          value,
          index
        });
      }
      async open() {
        const iface = findInterface(this._device, VENDOR_CLASS2);
        if (!iface) {
          throw new Error("No vendor specific interface found");
        }
        await this._device.claimInterface(iface.interfaceNumber);
        await controlIn(
          this._device,
          {
            requestType: "vendor",
            recipient: "device",
            request: REQ_READ_VERSION,
            value: 0,
            index: 0
          },
          2
        );
        await this._out(REQ_SERIAL_INIT, 0, 0);
        return findBulkEndpoints(iface);
      }
      async setLineConfig(config) {
        await this._out(
          REQ_WRITE_REG,
          4882,
          ch34xBaudDivisor(config.baudRate) | 128
        );
        await this._out(REQ_WRITE_REG, 9496, ch34xLineControl(config));
      }
      setSignals(dataTerminalReady, requestToSend) {
        return this._out(
          REQ_MODEM_CTRL,
          ch34xModemControl(dataTerminalReady, requestToSend),
          0
        );
      }
    };
    createCh34xDriver = (device) => new Ch34xDriver(device);
  }
});

// ftdi.ts
var ftdi_exports = {};
__export(ftdi_exports, {
  createFtdiDriver: () => createFtdiDriver,
  ftdiBaudDivisor: () => ftdiBaudDivisor,
  ftdiChipFromVersion: () => ftdiChipFromVersion,
  ftdiDataBits: () => ftdiDataBits,
  ftdiModemControl: () => ftdiModemControl,
  ftdiStripStatus: () => ftdiStripStatus
});
var VENDOR_CLASS3, REQ_SET_MODEM_CTRL, REQ_SET_FLOW_CTRL, REQ_SET_BAUDRATE, REQ_SET_DATA, MODEM_DTR_HIGH, MODEM_RTS_HIGH, MODEM_DTR_MASK, MODEM_RTS_MASK, DATA_BREAK, STATUS_BYTES, ftdiChipFromVersion, DIV_FRAC, divRoundClosest, ftdiBaudDivisor, ftdiDataBits, ftdiModemControl, FtdiDriver, ftdiStripStatus, createFtdiDriver;
var init_ftdi = __esm({
  "ftdi.ts"() {
    init_driver();
    VENDOR_CLASS3 = 255;
    REQ_SET_MODEM_CTRL = 1;
    REQ_SET_FLOW_CTRL = 2;
    REQ_SET_BAUDRATE = 3;
    REQ_SET_DATA = 4;
    MODEM_DTR_HIGH = 1;
    MODEM_RTS_HIGH = 2;
    MODEM_DTR_MASK = 256;
    MODEM_RTS_MASK = 512;
    DATA_BREAK = 1 << 14;
    STATUS_BYTES = 2;
    ftdiChipFromVersion = (bcdDevice, multiInterface) => {
      if (bcdDevice < 1024) {
        return void 0;
      }
      if (multiInterface) {
        const highSpeed = bcdDevice === 1792 || bcdDevice === 2048;
        return { highSpeed, shiftBaudIndex: true, maxBaud: highSpeed ? 12e6 : 3e6 };
      }
      switch (bcdDevice) {
        case 1024:
        // FT232BM
        case 1536:
          return { highSpeed: false, shiftBaudIndex: false, maxBaud: 3e6 };
        case 1280:
        // FT2232C/D
        case 4096:
          return { highSpeed: false, shiftBaudIndex: true, maxBaud: 3e6 };
        default:
          return { highSpeed: true, shiftBaudIndex: true, maxBaud: 12e6 };
      }
    };
    DIV_FRAC = [0, 3, 2, 4, 1, 5, 6, 7];
    divRoundClosest = (n, d) => Math.floor((n + Math.floor(d / 2)) / d);
    ftdiBaudDivisor = (baudRate, chip) => {
      const baud = Math.min(Math.max(baudRate, 1), chip.maxBaud);
      const divisor3 = chip.highSpeed ? divRoundClosest(8 * 12e7, 10 * baud) : divRoundClosest(48e6, 2 * baud);
      let divisor = divisor3 >> 3 | DIV_FRAC[divisor3 & 7] << 14;
      if (divisor === 1) {
        divisor = 0;
      } else if (divisor === 16385) {
        divisor = 1;
      }
      if (chip.highSpeed) {
        divisor |= 131072;
      }
      return divisor;
    };
    ftdiDataBits = (config, brk = false) => {
      const parity = { none: 0, odd: 1, even: 2 }[config.parity];
      const stop = config.stopBits === 2 ? 2 : 0;
      return config.dataBits | parity << 8 | stop << 11 | (brk ? DATA_BREAK : 0);
    };
    ftdiModemControl = (dtr, rts) => MODEM_DTR_MASK | MODEM_RTS_MASK | (dtr ? MODEM_DTR_HIGH : 0) | (rts ? MODEM_RTS_HIGH : 0);
    FtdiDriver = class {
      constructor(_device) {
        this._device = _device;
        __publicField(this, "_chip");
        __publicField(this, "_index", 0);
        __publicField(this, "_packetSize", 64);
        __publicField(this, "_config");
      }
      _out(request, value, index) {
        return controlOut(this._device, {
          requestType: "vendor",
          recipient: "device",
          request,
          value,
          index
        });
      }
      async open() {
        const iface = findInterface(this._device, VENDOR_CLASS3);
        if (!iface) {
          throw new Error("No vendor specific interface found");
        }
        const multiInterface = this._device.configuration.interfaces.length > 1;
        const bcdDevice = this._device.deviceVersionMajor << 8 | this._device.deviceVersionMinor << 4 | this._device.deviceVersionSubminor;
        const chip = ftdiChipFromVersion(bcdDevice, multiInterface);
        if (!chip) {
          throw new Error("Unsupported FTDI chip");
        }
        this._chip = chip;
        this._index = multiInterface ? iface.interfaceNumber + 1 : 0;
        await this._device.claimInterface(iface.interfaceNumber);
        const endpoints = findBulkEndpoints(iface);
        this._packetSize = endpoints.in.packetSize;
        await this._out(REQ_SET_FLOW_CTRL, 0, this._index);
        return endpoints;
      }
      async setLineConfig(config) {
        this._config = config;
        const divisor = ftdiBaudDivisor(config.baudRate, this._chip);
        const value = divisor & 65535;
        let index = divisor >>> 16 & 65535;
        if (this._chip.shiftBaudIndex) {
          index = index << 8 | this._index;
        }
        await this._out(REQ_SET_BAUDRATE, value, index);
        await this._out(REQ_SET_DATA, ftdiDataBits(config), this._index);
      }
      setSignals(dataTerminalReady, requestToSend) {
        return this._out(
          REQ_SET_MODEM_CTRL,
          ftdiModemControl(dataTerminalReady, requestToSend),
          this._index
        );
      }
      setBreak(enabled) {
        return this._out(
          REQ_SET_DATA,
          ftdiDataBits(this._config, enabled),
          this._index
        );
      }
      processRead(data) {
        return ftdiStripStatus(data, this._packetSize);
      }
    };
    ftdiStripStatus = (data, packetSize) => {
      const out = new Uint8Array(data.byteLength);
      let length = 0;
      for (let start = 0; start < data.byteLength; start += packetSize) {
        const payload = data.subarray(
          start + STATUS_BYTES,
          Math.min(start + packetSize, data.byteLength)
        );
        out.set(payload, length);
        length += payload.byteLength;
      }
      return out.subarray(0, length);
    };
    createFtdiDriver = (device) => new FtdiDriver(device);
  }
});

// cdc-acm.ts
var cdc_acm_exports = {};
__export(cdc_acm_exports, {
  cdcLineCoding: () => cdcLineCoding,
  createCdcAcmDriver: () => createCdcAcmDriver
});
var CDC_CLASS, CDC_DATA_CLASS, SET_LINE_CODING, SET_CONTROL_LINE_STATE, SEND_BREAK, PARITY_INDEX, cdcLineCoding, CdcAcmDriver, createCdcAcmDriver;
var init_cdc_acm = __esm({
  "cdc-acm.ts"() {
    init_driver();
    CDC_CLASS = 2;
    CDC_DATA_CLASS = 10;
    SET_LINE_CODING = 32;
    SET_CONTROL_LINE_STATE = 34;
    SEND_BREAK = 35;
    PARITY_INDEX = {
      none: 0,
      odd: 1,
      even: 2
    };
    cdcLineCoding = (config) => {
      const buffer = new Uint8Array(7);
      new DataView(buffer.buffer).setUint32(0, config.baudRate, true);
      buffer[4] = config.stopBits === 2 ? 2 : 0;
      buffer[5] = PARITY_INDEX[config.parity];
      buffer[6] = config.dataBits;
      return buffer;
    };
    CdcAcmDriver = class {
      constructor(_device) {
        this._device = _device;
        __publicField(this, "_controlNumber", 0);
      }
      async open() {
        const control = findInterface(this._device, CDC_CLASS);
        if (!control) {
          throw new Error("No CDC control interface found");
        }
        const data = findInterface(this._device, CDC_DATA_CLASS) ?? control;
        this._controlNumber = control.interfaceNumber;
        await this._device.claimInterface(control.interfaceNumber);
        if (data !== control) {
          await this._device.claimInterface(data.interfaceNumber);
        }
        return findBulkEndpoints(data);
      }
      setLineConfig(config) {
        return controlOut(
          this._device,
          {
            requestType: "class",
            recipient: "interface",
            request: SET_LINE_CODING,
            value: 0,
            index: this._controlNumber
          },
          cdcLineCoding(config)
        );
      }
      setSignals(dataTerminalReady, requestToSend) {
        return controlOut(this._device, {
          requestType: "class",
          recipient: "interface",
          request: SET_CONTROL_LINE_STATE,
          value: (dataTerminalReady ? 1 : 0) | (requestToSend ? 2 : 0),
          index: this._controlNumber
        });
      }
      setBreak(enabled) {
        return controlOut(this._device, {
          requestType: "class",
          recipient: "interface",
          request: SEND_BREAK,
          value: enabled ? 65535 : 0,
          index: this._controlNumber
        });
      }
    };
    createCdcAcmDriver = (device) => new CdcAcmDriver(device);
  }
});

// port.ts
var port_exports = {};
__export(port_exports, {
  UsbSerialPort: () => UsbSerialPort,
  toLineConfig: () => toLineConfig
});
var DEFAULT_BUFFER_SIZE, toLineConfig, UsbSerialPort;
var init_port = __esm({
  "port.ts"() {
    DEFAULT_BUFFER_SIZE = 255;
    toLineConfig = (options) => {
      const { baudRate, dataBits = 8, stopBits = 1, parity = "none" } = options;
      if (!Number.isInteger(baudRate) || baudRate <= 0) {
        throw new RangeError(`invalid Baud Rate ${baudRate}`);
      }
      if (![5, 6, 7, 8].includes(dataBits)) {
        throw new RangeError(`invalid dataBits ${dataBits}`);
      }
      if (stopBits !== 1 && stopBits !== 2) {
        throw new RangeError(`invalid stopBits ${stopBits}`);
      }
      if (!["none", "even", "odd"].includes(parity)) {
        throw new RangeError(`invalid parity ${parity}`);
      }
      return { baudRate, dataBits, stopBits, parity };
    };
    UsbSerialPort = class {
      constructor(_device, _createDriver) {
        this._device = _device;
        this._createDriver = _createDriver;
        __publicField(this, "_driver");
        __publicField(this, "_endpoints");
        __publicField(this, "_options");
        __publicField(this, "_readable", null);
        __publicField(this, "_writable", null);
        __publicField(this, "_pendingRead");
        __publicField(this, "_signals", { dataTerminalReady: false, requestToSend: false });
        __publicField(this, "_break", false);
      }
      get _bufferSize() {
        return this._options?.bufferSize ?? DEFAULT_BUFFER_SIZE;
      }
      get readable() {
        if (this._readable || !this._device.opened || !this._endpoints) {
          return this._readable;
        }
        const stream = new ReadableStream(
          {
            pull: async (controller) => {
              this._pendingRead ?? (this._pendingRead = this._read(controller.desiredSize));
              this._pendingRead.catch(() => void 0);
              let chunk;
              try {
                chunk = await this._pendingRead;
              } catch (err) {
                this._pendingRead = void 0;
                if (this._readable === stream) {
                  this._readable = null;
                }
                controller.error(err);
                return;
              }
              if (this._readable !== stream) {
                return;
              }
              this._pendingRead = void 0;
              controller.enqueue(chunk);
            },
            cancel: () => {
              if (this._readable === stream) {
                this._readable = null;
              }
            }
          },
          new ByteLengthQueuingStrategy({ highWaterMark: this._bufferSize })
        );
        this._readable = stream;
        return stream;
      }
      get writable() {
        if (this._writable || !this._device.opened || !this._endpoints) {
          return this._writable;
        }
        const stream = new WritableStream(
          {
            write: async (chunk) => {
              if (!chunk.byteLength) {
                return;
              }
              try {
                const result = await this._device.transferOut(
                  this._endpoints.out.endpointNumber,
                  chunk
                );
                if (result.status !== "ok") {
                  throw new Error(`USB error: ${result.status}`);
                }
              } catch (err) {
                if (this._writable === stream) {
                  this._writable = null;
                }
                throw err;
              }
            }
          },
          new ByteLengthQueuingStrategy({ highWaterMark: this._bufferSize })
        );
        this._writable = stream;
        return stream;
      }
      /** Read one non-empty, driver-processed chunk from the IN endpoint. */
      async _read(desiredSize) {
        const endpoint = this._endpoints.in;
        const driver = this._driver;
        const packetSize = endpoint.packetSize;
        const length = Math.max(1, Math.ceil((desiredSize ?? packetSize) / packetSize)) * packetSize;
        for (; ; ) {
          const result = await this._device.transferIn(
            endpoint.endpointNumber,
            length
          );
          if (result.status !== "ok") {
            throw new Error(`USB error: ${result.status}`);
          }
          if (!result.data?.byteLength) {
            continue;
          }
          let chunk = new Uint8Array(
            result.data.buffer,
            result.data.byteOffset,
            result.data.byteLength
          );
          if (driver.processRead) {
            chunk = driver.processRead(chunk);
          }
          if (chunk.byteLength) {
            return chunk;
          }
        }
      }
      async open(options) {
        const config = toLineConfig(options);
        this._options = options;
        try {
          await this._device.open();
          if (this._device.configuration === null) {
            await this._device.selectConfiguration(1);
          }
          const driver = this._createDriver(this._device);
          this._driver = driver;
          this._endpoints = await driver.open();
          await driver.setLineConfig(config);
          this._signals = { dataTerminalReady: false, requestToSend: false };
          this._break = false;
          await driver.setSignals(false, false);
        } catch (err) {
          this._driver = void 0;
          this._endpoints = void 0;
          if (this._device.opened) {
            await this._device.close().catch(() => void 0);
          }
          throw new Error(`Error setting up device: ${err}`);
        }
      }
      async close() {
        await Promise.all([
          this._readable?.cancel().catch(() => void 0),
          this._writable?.abort().catch(() => void 0)
        ]);
        this._readable = null;
        this._writable = null;
        this._pendingRead = void 0;
        if (!this._device.opened) {
          return;
        }
        try {
          await this._driver?.setSignals(false, false);
          await this._driver?.close?.();
        } catch {
        }
        this._driver = void 0;
        this._endpoints = void 0;
        await this._device.close();
      }
      forget() {
        return this._device.forget();
      }
      getInfo() {
        return {
          usbVendorId: this._device.vendorId,
          usbProductId: this._device.productId
        };
      }
      /** Change line settings (baud rate, ...) of an open port. */
      async reconfigure(options) {
        const merged = { ...this._options, ...options };
        const config = toLineConfig(merged);
        this._options = merged;
        await this._requireDriver().setLineConfig(config);
      }
      async setSignals(signals) {
        const driver = this._requireDriver();
        if (signals.dataTerminalReady !== void 0 || signals.requestToSend !== void 0) {
          this._signals = {
            dataTerminalReady: signals.dataTerminalReady ?? this._signals.dataTerminalReady,
            requestToSend: signals.requestToSend ?? this._signals.requestToSend
          };
          await driver.setSignals(
            this._signals.dataTerminalReady,
            this._signals.requestToSend
          );
        }
        if (signals.break !== void 0 && driver.setBreak) {
          this._break = signals.break;
          await driver.setBreak(this._break);
        }
      }
      _requireDriver() {
        if (!this._driver) {
          throw new DOMException("The port is not open.", "InvalidStateError");
        }
        return this._driver;
      }
    };
  }
});

// index.ts
init_driver();
var usbSerialDrivers = [
  {
    name: "CP210x",
    // Same product IDs as the Linux cp210x driver's default table.
    // Vendor ID 0x10c4 is Silicon Labs.
    filters: [6e4, 60001, 60003, 60016, 60017].map((productId) => ({
      vendorId: 4292,
      productId
    })),
    load: () => Promise.resolve().then(() => (init_cp210x(), cp210x_exports)).then((m) => m.createCp210xDriver)
  },
  {
    name: "CH340/CH341",
    filters: [
      { vendorId: 6790, productId: 29987 },
      { vendorId: 6790, productId: 29986 },
      { vendorId: 6790, productId: 21795 },
      { vendorId: 17224, productId: 21795 }
    ],
    load: () => Promise.resolve().then(() => (init_ch34x(), ch34x_exports)).then((m) => m.createCh34xDriver)
  },
  {
    name: "FTDI",
    // FT232R/BM/AM, FT2232, FT4232, FT232H, FT-X.
    filters: [24577, 24592, 24593, 24596, 24597].map((productId) => ({
      vendorId: 1027,
      productId
    })),
    load: () => Promise.resolve().then(() => (init_ftdi(), ftdi_exports)).then((m) => m.createFtdiDriver)
  },
  {
    name: "CDC-ACM",
    // USB class 0x02 is the communications device class.
    filters: [{ classCode: 2 }],
    load: () => Promise.resolve().then(() => (init_cdc_acm(), cdc_acm_exports)).then((m) => m.createCdcAcmDriver)
  }
];
var usbSerialFilters = () => usbSerialDrivers.flatMap((driver) => driver.filters);
var findUsbSerialDriver = (device) => usbSerialDrivers.find((driver) => matchesDefinition(device, driver));
var requestUsbSerialPort = async () => {
  const device = await navigator.usb.requestDevice({
    filters: usbSerialFilters()
  });
  const definition = findUsbSerialDriver(device);
  if (!definition) {
    throw new Error("Unsupported USB serial device");
  }
  const [create, { UsbSerialPort: UsbSerialPort2 }] = await Promise.all([
    definition.load(),
    Promise.resolve().then(() => (init_port(), port_exports))
  ]);
  return new UsbSerialPort2(device, create);
};
export {
  findUsbSerialDriver,
  requestUsbSerialPort,
  usbSerialDrivers,
  usbSerialFilters
};
