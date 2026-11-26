const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
test('device dashboard has no internet-dependent rendering assets',()=>{
 const urls=[...html.matchAll(/<(?:script|img|iframe|source|video|audio|link)\b[^>]*\b(?:src|href|poster)\s*=\s*["']([^"']+)["']/gi)].map(m=>m[1]);
 for(const match of html.matchAll(/url\(\s*['"]?([^)'"\s]+)['"]?\s*\)/gi))urls.push(match[1]);
 for(const url of urls)assert.doesNotMatch(url,/^(?:https?:)?\/\//i,'external rendering dependency: '+url.slice(0,100));
 assert.doesNotMatch(html,/@import\s/i,'CSS imports could introduce online dependencies');
});
test('both instrument font files are embedded byte-for-byte in the device page',()=>{
 const specs=JSON.parse(fs.readFileSync('docs/assets/fonts/sources.json','utf8'));
 for(const [slug,spec] of Object.entries(specs)){
  const expected=fs.readFileSync('docs/assets/fonts/'+slug+'.woff2'),face=[...html.matchAll(/@font-face\{([^}]+)\}/g)].find(m=>m[1].includes("font-family:'"+spec.family+"'"));assert.ok(face,spec.family+' face');
  const encoded=face[1].match(/data:font\/woff2;base64,([A-Za-z0-9+/=]+)/);assert.ok(encoded,spec.family+' inline source');assert.deepEqual(Buffer.from(encoded[1],'base64'),expected,spec.family+' bundled font bytes');
 }
});
