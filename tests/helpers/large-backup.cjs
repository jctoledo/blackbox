// Test adapter for the production inline module; does not run tests on import.
const vm=require('node:vm');
const fs=require('node:fs');
const {indexedDB,IDBKeyRange}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));
let serial=0;
function context(){
 const c={console,Number,Math,Date,Map,Set,Float32Array,ArrayBuffer,Promise,TextEncoder,TextDecoder,structuredClone,indexedDB,IDBKeyRange,AbortController,crypto:require('node:crypto').webcrypto,rec:false,recordingBusy:false,chunkWrite:null,activeTrack:null,trackRecorder:null,chunkBuffer:[],sessionId:null,sessionStart:0,chunkIndex:0};
 vm.createContext(c);vm.runInContext(slice('// Drive session core.','// End Drive session core.'),c);vm.runInContext(slice('const failedRecordingMetadata=','// End recording details core.'),c);vm.runInContext(slice('function validSpatialReference(','class SpatialMatcher'),c);vm.runInContext(slice('// Versioned backup / restore.','// End backup / restore'),c);vm.runInContext(slice('function portableReference(','async function exportTrackData'),c);
 c.DriveSession=vm.runInContext('DriveSession',c);c.DriveReplay=vm.runInContext('DriveReplay',c);return c;
}
function database(tracks=false){return new Promise((resolve,reject)=>{const r=indexedDB.open('multipart-test-'+process.pid+'-'+(++serial));r.onerror=()=>reject(r.error);r.onsuccess=()=>resolve(r.result);r.onupgradeneeded=()=>{const d=r.result;if(tracks){d.createObjectStore('tracks',{keyPath:'id'});d.createObjectStore('reference_laps',{keyPath:'id'}).createIndex('trackId','trackId',{unique:true});d.createObjectStore('lap_history',{keyPath:'id'})}else{d.createObjectStore('sessions',{keyPath:'sessionId'});d.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']}).createIndex('sessionId','sessionId')}}})}
function file(part){return{name:part.name,size:Buffer.byteLength(part.text),text:async()=>part.text,arrayBuffer:async()=>{const bytes=Buffer.from(part.text);return bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)}}}
async function exported(c,scope='all',options={}){const job=await c.prepareMultipartBackup(scope,options),files=[];let part;while((part=await job.next()))files.push(file(part));return files}
module.exports={context,database,file,exported,indexedDB,IDBKeyRange,vm};
