/* The release is a merged ESP32-C3 image: bootloader starts at address zero. */
async function loadVerifiedFirmware(project, fetcher = fetch, cryptoApi = crypto) {
    const manifestResponse = await fetcher(project.manifestUrl, {cache:'no-store'});
    if (!manifestResponse.ok) throw new Error('Firmware manifest unavailable. Try again after deployment finishes.');
    const manifest = await manifestResponse.json();
    if (manifest.chip !== 'esp32c3' || manifest.offset !== 0 ||
        !Number.isInteger(manifest.size) || manifest.size < 32 || manifest.size > 4*1024*1024 ||
        !/^[a-f0-9]{64}$/.test(manifest.sha256)) throw new Error('Unsupported firmware manifest.');
    const response = await fetcher(project.firmwareUrl, {cache:'no-store'});
    if (!response.ok) throw new Error('Firmware download failed. Reload and try again.');
    const data = await response.arrayBuffer();
    const bytes = new Uint8Array(data);
    if (bytes.length !== manifest.size || bytes[0] !== 0xe9 || bytes[12] !== 5 || bytes[13] !== 0)
        throw new Error('This is not the expected merged ESP32-C3 image.');
    const digest = Array.from(new Uint8Array(await cryptoApi.subtle.digest('SHA-256',data)),
        b=>b.toString(16).padStart(2,'0')).join('');
    if (digest !== manifest.sha256) throw new Error('Firmware checksum mismatch. No data was written.');
    return {manifest, blob:new Blob([data],{type:'application/octet-stream'})};
}
function flashTransportMode(nav, secure) {
    if (!secure) return null;
    // Android's native Web Serial can expose Bluetooth without exposing USB.
    const android=/Android/i.test(nav.userAgent || '') || nav.userAgentData?.platform === 'Android';
    if (android) return nav.usb ? 'usb' : null;
    if (nav.usb && !nav.serial) return 'usb';
    return nav.serial ? 'serial' : null;
}
async function prepareFlashTransport(nav, secure, loadUsb=()=>import('./vendor/usb-serial.mjs')) {
    const kind=flashTransportMode(nav,secure);
    if (!kind) return null;
    if (kind==='serial') return {kind,requestPort:()=>nav.serial.requestPort()};
    // Preload before enabling Connect: USB permission needs the click's activation.
    const {requestUsbSerialPort}=await loadUsb();
    return {kind,requestPort:()=>requestUsbSerialPort()};
}
function createBlackboxFlasher(Flasher, project, transport) {
    const flasher=new Flasher({chip:'esp32c3',firmwareUrl:project.firmwareUrl,firmwareOffset:0});
    if (transport) {
        // The pinned toolkit exposes DeviceConnection and accepts an existing port.
        // Keep its state machine, chip detection and mismatch refusal intact.
        const device=flasher.getDevice(),connect=device.connect.bind(device);
        device.connect=async(chip,options={})=>{
            const port=await transport.requestPort();
            return connect(chip,{...options,port});
        };
    }
    return flasher;
}
if (typeof module !== 'undefined') module.exports={loadVerifiedFirmware,createBlackboxFlasher,flashTransportMode,prepareFlashTransport};
async function initializeBlackboxFlasher(prepareTransport=prepareFlashTransport) {
    const $=id=>document.getElementById(id);
    const select=$('project-select'),button=$('flash-button'),status=$('status-box');
    const Flasher=window.ESPWebFlash?.ESPFlasher;
    let transport=null,supported=false;
    let flasher=null,firmware=null,selection=0,busy=false,completed=false;
    const message=(title,detail,state='')=>{
        status.className='status-box '+state;
        status.querySelector('.status-text').textContent=title;
        status.querySelector('.status-subtext').textContent=detail;
    };
    const log=(text)=>{
        const line=document.createElement('div');line.className='debug-line';line.textContent=text;
        $('debug-window').appendChild(line);$('debug-window').scrollTop=$('debug-window').scrollHeight;
    };
    const refresh=()=>{
        select.disabled=!supported||busy;
        button.disabled=!supported||busy||!firmware||completed;
        button.textContent=completed?'Installation complete':busy?'Please wait…':
            flasher?.isConnected()?'Flash firmware':'Connect device';
    };
    for(const [id,p] of Object.entries(PROJECTS)){
        const option=document.createElement('option');option.value=id;option.textContent=p.name;select.appendChild(option);
    }
    select.addEventListener('change',async()=>{
        const generation=++selection;firmware=null;completed=false;
        if(flasher){await flasher.disconnect();flasher.dispose();flasher=null;}
        $('project-info').style.display=select.value?'block':'none';refresh();
        if(!select.value||!supported)return;
        const project=PROJECTS[select.value];busy=true;refresh();
        message('Checking firmware','Downloading and verifying the published release…');
        try{
            const verified=await loadVerifiedFirmware(project);
            if(generation!==selection)return;
            firmware=verified;flasher=createBlackboxFlasher(Flasher,project,transport);
            flasher.addEventListener('log',e=>log(e.detail.message));
            flasher.addEventListener('progress',e=>message('Writing '+e.detail.percent+'%','Keep the USB cable connected.','flashing'));
            flasher.addEventListener('disconnected',()=>{if(!busy){message('Device disconnected','Reconnect to continue.');refresh();}});
            flasher.addEventListener('chip-mismatch',e=>{queueMicrotask(()=>e.detail.cancel());message('Wrong device','Only an ESP32-C3 can use this firmware.','error');});
            message('Verified '+verified.manifest.version,'Connect an ESP32-C3 with a data-capable USB cable.');
            log('SHA-256 verified. Merged image; flash offset 0x0.');
        }catch(e){message('Firmware unavailable',e.message,'error');log(e.message);}
        finally{busy=false;refresh();}
    });
    button.addEventListener('click',async()=>{
        if(!flasher||!firmware||busy||completed)return;
        busy=true;refresh();
        try{
            if(!flasher.isConnected()){
                message('Connecting',transport.kind==='usb'?'Select your ESP32-C3 USB device and allow access.':'Select your ESP32-C3 serial port.');
                await flasher.connect();
                message('Ready to install '+firmware.manifest.version,'Flashing replaces the full device image, including saved device configuration. Browser recordings are unaffected.','connected');
            }else{
                let wakeLock;
                try {
                    try {wakeLock=await navigator.wakeLock?.request('screen');} catch (_) { /* Keep-screen-on advice remains visible. */ }
                    await flasher.flash({customFirmware:firmware.blob,firmwareOffset:0});
                } finally {try {await wakeLock?.release();} catch (_) { /* Best effort. */ }}
                completed=true;
                message('Firmware installed','Press RESET or reconnect USB. Keep the unit still during calibration.','success');
                await flasher.disconnect();
            }
        }catch(e){
            await flasher.disconnect();
            const detail=e.name==='NotFoundError'?'No device selected. Tap Connect device to try again.':
                e.name==='NotAllowedError'||e.name==='SecurityError'?'USB access was denied. Allow this website to access the ESP, then reconnect.':
                e.message.includes('imeout')?'Hold BOOT, tap RESET, release BOOT, and retry. Close other serial monitors.':
                e.message+' Check the USB data cable and phone USB host/OTG support, then reconnect.';
            message('Could not finish',detail,'error');log(e.message);
        }
        finally{busy=false;refresh();}
    });
    $('dev-toggle').addEventListener('click',()=>$('dev-panel').classList.toggle('active'));
    window.addEventListener('scroll',()=>$('nav').classList.toggle('scrolled',window.scrollY>50));
    refresh();
    message('Loading USB support','Keep this page open while the installer loads.');
    try {
        transport=await prepareTransport(navigator,window.isSecureContext);
        supported=!!transport&&!!Flasher;
        if (supported) message('Ready to install',transport.kind==='usb'?'Android USB ready. Select the NEO-M9N firmware for an M9N receiver.':'Select firmware to begin.');
        else message('Browser flashing unavailable',!window.isSecureContext?'Open this installer over HTTPS.':!Flasher?'The flashing library could not load. Reload and try again.':'Use Android Chrome with USB host/OTG, or desktop Chrome/Edge. iPhone USB flashing is not supported.','error');
    } catch(e) {message('USB support could not load','Check your internet connection and reload this page.','error');log(e.message);}
    refresh();
}
if (typeof document !== 'undefined') initializeBlackboxFlasher();
