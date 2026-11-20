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
function createBlackboxFlasher(Flasher, project) {
    return new Flasher({chip:'esp32c3',firmwareUrl:project.firmwareUrl,firmwareOffset:0});
}
if (typeof module !== 'undefined') module.exports={loadVerifiedFirmware,createBlackboxFlasher};
if (typeof document !== 'undefined') {
    const $=id=>document.getElementById(id);
    const select=$('project-select'),button=$('flash-button'),status=$('status-box');
    const Flasher=window.ESPWebFlash?.ESPFlasher;
    const supported=!!navigator.serial&&!!Flasher&&!!window.isSecureContext;
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
            firmware=verified;flasher=createBlackboxFlasher(Flasher,project);
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
                message('Connecting','Select your ESP32-C3 serial port.');
                await flasher.connect();
                message('Ready to install '+firmware.manifest.version,'Flashing replaces the full device image, including saved device configuration. Browser recordings are unaffected.','connected');
            }else{
                await flasher.flash({customFirmware:firmware.blob,firmwareOffset:0});
                completed=true;
                message('Firmware installed','Press RESET or reconnect USB. Keep the unit still during calibration.','success');
                await flasher.disconnect();
            }
        }catch(e){message('Could not finish',e.message.includes('imeout')?'Hold BOOT, tap RESET, release BOOT, and retry. Close other serial monitors.':e.message,'error');log(e.message);}
        finally{busy=false;refresh();}
    });
    $('dev-toggle').addEventListener('click',()=>$('dev-panel').classList.toggle('active'));
    window.addEventListener('scroll',()=>$('nav').classList.toggle('scrolled',window.scrollY>50));
    if(!supported)message('Browser flashing unavailable',!navigator.serial?'Use desktop Chrome or Edge with Web Serial. You can also download the release and flash from the command line.':'The flashing library could not load, or this page is not secure. Reload over HTTPS.','error');
    refresh();
}
