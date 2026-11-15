// Page web servie par la blackbox (aucune ressource externe : fonctionne sans Internet)
#pragma once
#include <Arduino.h>

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Blackbox PSA</title>
<style>
:root{--bg:#111;--card:#1c1c1e;--txt:#eee;--dim:#8a8a8e;--ok:#34c759;--warn:#ff9f0a;--bad:#ff453a}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--txt)}
header{display:flex;align-items:center;gap:8px;padding:8px 12px;background:#222;position:sticky;top:0;z-index:2;flex-wrap:wrap}
h1{font-size:16px;margin:0}
.pill{padding:3px 9px;border-radius:10px;font-weight:700;font-size:12px;color:#000;background:#666}
.sp{flex:1}
button{background:#333;color:var(--txt);border:0;border-radius:6px;padding:8px 12px;font-size:14px;cursor:pointer}
nav button.on{background:#5a5a5e}
#mark{background:#0bc;color:#000;font-weight:800;font-size:16px;padding:10px 18px}
#mark.flash{background:#fff}
main{padding:10px;max-width:1100px;margin:auto}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.card{background:var(--card);border-radius:8px;padding:8px 10px}
.l{font-size:12px;color:var(--dim)}
.v{font-size:21px;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
h2{font-size:13px;color:var(--dim);margin:16px 0 6px;text-transform:uppercase;letter-spacing:.05em}
canvas{display:block;width:100%;height:170px;background:var(--card);border-radius:8px;margin-top:8px}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:5px 6px;border-bottom:1px solid #333;text-align:left;font-variant-numeric:tabular-nums}
a{color:#0bc}
.hide{display:none}
.note{color:var(--dim);font-size:13px}
</style></head><body>
<header>
  <h1>Blackbox PSA</h1><span id="st" class="pill">...</span><span id="arm" class="pill">...</span>
  <span id="up" class="note"></span><span class="sp"></span>
  <nav><button data-tab="live" class="on">Temps réel</button><button data-tab="ev">Événements</button><button data-tab="files">Fichiers</button></nav>
  <button id="pause">Pause</button><button id="mark">MARQUEUR</button>
</header>
<main>
<section id="live">
  <h2>Pédale — connecteur 48V NR</h2><div class="grid" id="gp"></div>
  <canvas id="cp"></canvas>
  <h2>Papillon motorisé — connecteur 32V GR</h2><div class="grid" id="gt"></div>
  <canvas id="ct"></canvas>
  <h2>Alimentations 5 V capteurs</h2><canvas id="c5"></canvas>
  <h2>Calculateur</h2><div class="grid" id="ge"></div>
  <h2>Enregistrement</h2><div class="grid" id="gr"></div>
  <p class="note">Courbes : 30 dernières secondes. Les bandes claires sont le min/max mesuré à 2 kHz entre deux rafraîchissements : une microcoupure y apparaît même si elle est brève.</p>
</section>
<section id="ev" class="hide">
  <table><thead><tr><th>temps</th><th>type</th><th>phase</th><th>voie</th><th>valeur</th><th>durée</th><th>capture</th></tr></thead><tbody id="evb"></tbody></table>
</section>
<section id="files" class="hide">
  <p class="note">Mettre l'enregistrement en <b>pause</b> pour télécharger le fichier en cours. Les autres se téléchargent à tout moment.</p>
  <button id="fr">Actualiser</button>
  <table><thead><tr><th>fichier</th><th>taille</th></tr></thead><tbody id="fb"></tbody></table>
</section>
</main>
<script>
const $=id=>document.getElementById(id), WIN=30, H=[];
let th={}, tab='live', lastEv=0;
const COL={ok:'#34c759',warn:'#ff9f0a',bad:'#ff453a'};
function card(g,id,l){g.insertAdjacentHTML('beforeend',`<div class="card"><div class="l">${l}</div><div class="v" id="${id}">-</div></div>`)}
[['gp',[['vV5P','+5 V (G1 · 1380)'],['vS1','S1 (F2 · 1377)'],['vS2','S2 (G2 · 1378)'],['vGNDP','masse capteur (F1)'],['vEP','écart S1 − 2·S2']]],
 ['gt',[['vV5T','+5 V (A1 · 1219)'],['vT1','piste 1 (B1 · 1218)'],['vT2','piste 2 (B4 · 1367)'],['vGNDT','masse capteur (A4)'],['vTS','T1 + T2']]],
 ['ge',[['vV12BAT','+BAT (M3 · 140F)'],['vV12REL','+ après relais'],['vGNDECU','masse calculateur'],['vV12C','12 V vu par le calc.']]],
 ['gr',[['vFILE','fichier'],['vEV','événements'],['vCAP','captures'],['vMB','écrit'],['vDROP','pertes lignes / évts'],['vHZ','cadence ADS / rapide']]]
].forEach(([g,a])=>a.forEach(([i,l])=>card($(g),i,l)));
function set(id,t,c){const e=$(id);e.textContent=t;e.style.color=c||''}
const f3=v=>v.toFixed(3)+' V', mv=v=>(v*1000).toFixed(0)+' mV';
const c5=v=>(v<th.v5min||v>th.v5max)?COL.bad:COL.ok;
const cs=v=>(v<th.smin||v>th.smax)?COL.bad:'';
const cg=v=>{v=Math.abs(v);return v>th.gnd?COL.bad:(v>th.gnd/2?COL.warn:COL.ok)};
const c12=v=>(v<th.v12min||v>th.v12max)?COL.bad:COL.ok;
function pill(id,t,c){const e=$(id);e.textContent=t;e.style.background=c}
function hms(ms){const s=Math.floor(ms/1000);return [s/3600|0,(s/60|0)%60,s%60].map(x=>String(x).padStart(2,'0')).join(':')}

function chart(cv,ser,ymin,ymax,lines){
  const r=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  cv.width=w*r;cv.height=h*r;const x=cv.getContext('2d');x.scale(r,r);
  if(!H.length)return;
  const now=H[H.length-1].t,X=t=>w-(now-t)/1000/WIN*w,Y=v=>h-4-(v-ymin)/(ymax-ymin)*(h-8);
  x.font='10px sans-serif';x.lineWidth=1;
  for(let v=Math.ceil(ymin);v<=ymax;v++){x.strokeStyle='#2c2c2e';x.beginPath();x.moveTo(0,Y(v));x.lineTo(w,Y(v));x.stroke();x.fillStyle='#666';x.fillText(v+' V',3,Y(v)-2)}
  (lines||[]).forEach(([v,c])=>{x.strokeStyle=c;x.setLineDash([4,4]);x.beginPath();x.moveTo(0,Y(v));x.lineTo(w,Y(v));x.stroke();x.setLineDash([])});
  ser.forEach((s,k)=>{
    x.fillStyle=s.c+'55';
    H.forEach(p=>{const b=s.f(p);if(b){const y1=Y(b[2]),y0=Y(b[1]);x.fillRect(X(p.t)-1.5,y1,3,Math.max(1,y0-y1))}});
    x.strokeStyle=s.c;x.lineWidth=1.5;x.beginPath();let st=false;
    H.forEach(p=>{const b=s.f(p);if(!b)return;const px=X(p.t),py=Y(b[0]);st?x.lineTo(px,py):x.moveTo(px,py);st=true});
    x.stroke();x.fillStyle=s.c;x.fillText(s.n,w-70,13+13*k);
  });
}
const F=(n,k=1)=>p=>p.f[n]?p.f[n].map(v=>v*k):null;

function update(d){
  th=d.th;const a=d.a;
  pill('st',!d.sd?'PAS DE SD':(d.rec?'REC':'PAUSE'),!d.sd?COL.bad:(d.rec?COL.bad:COL.warn));
  pill('arm',d.armed?'CALC. ALIMENTÉ':'CALC. EN ATTENTE',d.armed?COL.ok:'#666');
  $('up').textContent=hms(d.t);$('pause').textContent=d.rec?'Pause':'Reprise';
  set('vV5P',f3(a.V5P),c5(a.V5P));set('vS1',f3(a.S1),cs(a.S1));set('vS2',f3(a.S2),cs(a.S2));set('vGNDP',mv(a.GNDP),cg(a.GNDP));
  const ref=(a.V5P-a.GNDP>3)?a.V5P-a.GNDP:5, e=((a.S1-a.GNDP)-th.k*(a.S2-a.GNDP))/ref*100;
  set('vEP',(e>=0?'+':'')+e.toFixed(1)+' %',Math.abs(e)>th.ped?COL.bad:COL.ok);
  set('vV5T',f3(a.V5T),c5(a.V5T));set('vT1',f3(a.T1),cs(a.T1));set('vT2',f3(a.T2),cs(a.T2));set('vGNDT',mv(a.GNDT),cg(a.GNDT));
  set('vTS',f3(a.T1+a.T2-2*a.GNDT));
  set('vV12BAT',f3(a.V12BAT),c12(a.V12BAT));set('vV12REL',f3(a.V12REL),c12(a.V12REL));set('vGNDECU',mv(a.GNDECU),cg(a.GNDECU));
  set('vV12C',f3(a.V12REL-a.GNDECU),c12(a.V12REL-a.GNDECU));
  set('vFILE',d.file);set('vEV',d.ev);set('vCAP',d.caps);set('vMB',d.mb.toFixed(1)+' Mo');
  set('vDROP',d.drop+' / '+d.evlost,(d.drop||d.evlost)?COL.bad:'');set('vHZ',d.adsHz+' / '+d.fastHz+' Hz');
  if(d.ev!==lastEv&&tab==='ev')loadEv();lastEv=d.ev;
}
function draw(){
  if(tab!=='live')return;
  chart($('cp'),[{n:'S1',c:'#ffd60a',f:F('S1')},{n:'2 × S2',c:'#64d2ff',f:F('S2',th.k||2)}],0,5);
  chart($('ct'),[{n:'piste 1',c:'#bf5af2',f:F('T1')},{n:'piste 2',c:'#30d158',f:F('T2')}],0,5);
  chart($('c5'),[{n:'5 V pédale',c:'#ffd60a',f:F('V5P')},{n:'5 V papillon',c:'#bf5af2',f:F('V5T')}],4.5,5.5,[[th.v5min,COL.bad],[th.v5max,COL.bad]]);
}
async function poll(){
  try{
    const d=await (await fetch('/api/live',{cache:'no-store'})).json();
    H.push({t:d.t,f:d.f});while(H.length&&d.t-H[0].t>WIN*1000)H.shift();
    update(d);draw();
  }catch(e){pill('st','HORS LIGNE','#666')}
  setTimeout(poll,100);
}
async function loadEv(){
  try{
    const ev=await (await fetch('/api/events')).json();
    $('evb').innerHTML=ev.map(e=>`<tr style="color:${e.type==='MARQUEUR'?'#0bc':(e.ph==='fin'?'#8a8a8e':'')}"><td>${hms(e.t)}.${String(e.t%1000).padStart(3,'0')}</td><td>${e.type}</td><td>${e.ph}</td><td>${e.ch}</td><td>${e.v.toFixed(3)}</td><td>${e.d?e.d+' ms':''}</td><td>${e.cap||''}</td></tr>`).join('')||'<tr><td>aucun</td></tr>';
  }catch(e){}
}
async function loadFiles(){
  try{
    const fl=await (await fetch('/api/files')).json();
    fl.sort((a,b)=>a.n<b.n?1:-1);
    $('fb').innerHTML=fl.map(f=>`<tr><td><a href="/dl?f=${encodeURIComponent(f.n)}">${f.n}</a></td><td>${(f.s/1048576).toFixed(2)} Mo</td></tr>`).join('')||'<tr><td>carte vide ou absente</td></tr>';
  }catch(e){}
}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{
  tab=b.dataset.tab;document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('on',x===b));
  ['live','ev','files'].forEach(s=>$(s).classList.toggle('hide',s!==tab));
  if(tab==='ev')loadEv();if(tab==='files')loadFiles();
});
$('fr').onclick=loadFiles;
$('mark').onclick=async()=>{await fetch('/api/mark',{method:'POST'});$('mark').classList.add('flash');setTimeout(()=>$('mark').classList.remove('flash'),300)};
$('pause').onclick=()=>fetch('/api/pause',{method:'POST'});
addEventListener('resize',draw);
poll();
</script></body></html>
)HTML";
