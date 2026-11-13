// FW57J-B Flight Simulator — ตัวเกมหลัก: เมนู → ขึ้นราง → บิน → กางร่มลงจอด → สรุปผล
import * as THREE from 'three';
import { makeTerrain } from './sim/terrainfn.js';
import { makeRealTerrain, neToLatLon, terrainAhead } from './sim/realterrain.js';
import { Aircraft, railProfile, trimLevel, thrustFull } from './sim/fdm.js';
import { Autopilot, MODES } from './sim/autopilot.js';
import { fillFuel, massProps, density, DEG, KMH, G, clamp, fuelTotal } from './sim/core.js';
import { World } from './view/world.js';
import { AircraftModel } from './view/model.js';
import { Hud, fmtT } from './view/hud.js';
import { Planner } from './view/planner.js';
import { Input } from './input.js';
import { Sound } from './audio.js';

const $ = id => document.getElementById(id);
const data = await (await fetch('data/fw57jb_sim.json')).json();
const H = 1 / 240;                                   // ก้าวเวลาฟิสิกส์ (s)

// ---------- ฉาก ----------
const renderer = new THREE.WebGLRenderer({ canvas: $('view'), antialias: true, logarithmicDepthBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(62, 1, 0.3, 160000);
// ---------- แผนที่: โลกสร้างอัตโนมัติ (มีเสมอ) + แผนที่จริงใน maps/ (ถ้าดาวน์โหลดไว้ด้วย tools/fetch_map.py) ----------
const procedural = makeTerrain(), terrains = { '': procedural };
const mapList = await fetch('maps/index.json').then(r => r.ok ? r.json() : []).catch(() => []);
for (const m of mapList) $('map').insertAdjacentHTML('afterbegin', `<option value="${m.id}">${m.name}</option>`);
async function loadTerrain(id) {
  if (terrains[id]) return terrains[id];
  const url = 'maps/' + id, meta = await (await fetch(url + '/meta.json')).json();
  meta.url = url;
  const bin = async name => { const r = await fetch(url + '/' + name); if (!r.ok) throw new Error(name); return r.arrayBuffer(); };
  const elev = new Int16Array(await bin('elev.bin')), forest = meta.forest ? new Uint8Array(await bin(meta.forest)) : null;
  return (terrains[id] = makeRealTerrain(meta, elev, forest));
}
let terrain = procedural;
if (mapList.length) {                                 // มีแผนที่จริง → ใช้เป็นค่าเริ่มต้น
  $('loading').textContent = 'กำลังโหลดแผนที่จริง…';
  try { terrain = await loadTerrain(mapList[mapList.length - 1].id); $('map').value = terrain.meta.id; } catch (err) { console.warn('โหลดแผนที่จริงไม่ได้ ใช้โลกสร้างอัตโนมัติแทน', err); $('map').value = ''; }
}
const world = new World(scene, terrain);
const model = new AircraftModel(data); model.addTo(scene);
world.loadRail().then(m => { if (m) console.info(`รางจาก CAD: ${m.nbody} ชิ้น ${m.ntri.toLocaleString()} สามเหลี่ยม`); else console.warn('ไม่พบ models/lr2 — ใช้รางสำรอง'); });
model.loadCad().then(m => { if (m) console.info(`โมเดลจาก CAD: ${m.nbody} ชิ้น ${m.ntri.toLocaleString()} สามเหลี่ยม (ส่งออก ${m.source.exported})`); else console.warn('ไม่พบ models/fw57jb — ใช้โมเดลสำรองจาก spec'); });
const hud = new Hud($('hud'), data);
const input = new Input();
const sound = new Sound();
function resize() { renderer.setSize(window.innerWidth, window.innerHeight, false); camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); hud.resize(); }
window.addEventListener('resize', resize); resize();

// ---------- สถานะเกม ----------
let ac, ap, state = 'menu', paused = false, acc = 0, timeScale = 1, settings = null;
let stats, log, endTimer = 0, toast = '', toastT = 0, launchConfirmT = 0, chuteConfirmT = 0;
const cam = { mode: 0, az: 0.5, el: 0.28, dist: 9, pos: new THREE.Vector3(), menuT: 0 };
const CAMS = ['ตามหลัง', 'หัวเครื่อง (FPV)', 'ภาคพื้น'];
const say = (t, sec = 3) => { toast = t; toastT = sec; };

function readSettings() {
  return {
    cargoKg: +$('cargo').value, fuelFrac: +$('fuel').value / 100,
    wind: { speed: +$('wind').value, fromDeg: +$('windDir').value, turb: +$('turb').value },
    quickStart: $('quick').checked, airStart: $('airStart').checked, autoMission: $('autoMission').checked,
  };
}
// หน้าวางแผนภารกิจ: ประมาณเวลา/น้ำมันจากแรงขับบินระดับที่ความเร็วภารกิจ (มวลตามที่ตั้งในเมนู หรือมวลปัจจุบันถ้ากำลังบิน)
const planner = new Planner({
  onChange: () => syncMission(),
  estimate: (kmh, dist) => {
    const flying = state === 'play' && ac && ac.phase === 'air';
    const s = readSettings(), mp = flying ? ac.mp : massProps(data, s.cargoKg, fillFuel(data, s.cargoKg, s.fuelFrac));
    const rho = density(terrain.siteAlt + 300, data.atmos.oat_C), tr = trimLevel(data, mp, kmh / KMH, { rho });
    const E = data.engine, Tfull = thrustFull(data, kmh / KMH) * rho / density(0, data.atmos.oat_C);
    const ff = E.fuel_flow_idle_g_min + (E.fuel_flow_max_g_min - E.fuel_flow_idle_g_min) * clamp(tr.thrust / Tfull, 0, 1);
    const min = dist / (kmh / KMH) / 60;
    return { min, fuelKg: ff * min / 1000, fuelAvailKg: flying ? fuelTotal(ac.fuel) : mp.fuelKg };
  },
});
planner.setTerrain(terrain);
function syncMission() {                              // ส่งแผนล่าสุดให้ autopilot และตัวแสดงผล
  const wps = planner.mission();
  world.setRoute(wps);
  if (!ap) return;
  ap.mission = wps; ap.missionSpd = planner.spd / KMH; ap.home = { n: 0, e: 0, alt: terrain.siteAlt }; ap.terrainFn = terrain.height;
  if (ap.mode === 'AUTO') { ap.wpIdx = Math.min(ap.wpIdx, Math.max(wps.length - 1, 0)); if (ac) ap.navFrom = { n: ac.s.pos[0], e: ac.s.pos[1], alt: ac.alt }; }
}
function openPlanner() {
  if (state === 'play') { paused = true; $('help').classList.add('hide'); }
  planner.open(state === 'play' && ac ? [ac.s.pos[0], ac.s.pos[1], ac.euler().yaw] : null);
}
$('plan').addEventListener('click', openPlanner); $('planHelp').addEventListener('click', openPlanner);
$('planClose').addEventListener('click', () => { planner.close(); if (state === 'play') paused = false; });

function refreshMenu() {
  const s = readSettings(), fuel = fillFuel(data, s.cargoKg, s.fuelFrac), mp = massProps(data, s.cargoKg, fuel);
  const rp = railProfile(data, mp.m), rho = density(terrain.siteAlt, data.atmos.oat_C);
  const vs = Math.sqrt(2 * mp.m * G / (rho * data.aero.S * data.aero.CLmax_design)) * KMH;
  const tr = trimLevel(data, mp, 200 / KMH, { rho });
  const ff = data.engine.fuel_flow_idle_g_min + (data.engine.fuel_flow_max_g_min - data.engine.fuel_flow_idle_g_min) * clamp(tr.thrust / 111.4, 0, 1);
  const win = data.mass.cg_window_x, okCg = mp.x >= win[0] && mp.x <= win[1];
  $('cargoV').textContent = `${s.cargoKg} kg`; $('fuelV').textContent = `${(s.fuelFrac * 100).toFixed(0)} %`;
  $('windV').textContent = `${s.wind.speed} m/s`; $('windDirV').textContent = `${s.wind.fromDeg}°`;
  $('info').innerHTML = `
    <div><b>มวลขึ้น</b><span>${mp.m.toFixed(2)} kg <small>(ขีดราง ${data.mass.tow_max_rail_kg.toFixed(2)})</small></span></div>
    <div><b>น้ำมัน</b><span>${mp.fuelKg.toFixed(2)} kg · ${(mp.fuelKg / data.mass.fuel_rho_kg_per_L).toFixed(1)} L</span></div>
    <div><b>CG (x)</b><span class="${okCg ? '' : 'bad'}">${mp.x.toFixed(4)} m · SM ${(mp.SM * 100).toFixed(1)} %</span></div>
    <div><b>ออกราง</b><span>${(rp.Vexit * KMH).toFixed(1)} km/h · สูงสุด ${(rp.aPeak / G).toFixed(1)} g · ${rp.tExit.toFixed(2)} s</span></div>
    <div><b>ร่วงหล่น</b><span>${vs.toFixed(1)} km/h (CLmax 1.19 · ลานปล่อยสูง ${terrain.siteAlt.toFixed(0)} m)</span></div>
    <div><b>บินได้ราว</b><span>${fmtT(mp.fuelKg * 60000 / ff)} นาที ที่ 200 km/h (${ff.toFixed(0)} g/min)</span></div>`;
}
for (const id of ['cargo', 'fuel', 'wind', 'windDir', 'turb']) $(id).addEventListener('input', refreshMenu);
function mapNote() {
  const m = terrain.meta;
  $('mapNote').innerHTML = !m ? 'ภูมิประเทศสร้างจากสูตร ไม่มีขอบ ไม่ใช่สถานที่จริง'
    : `ลานปล่อย ${m.lat0.toFixed(4)}°N ${m.lon0.toFixed(4)}°E สูง ${m.site_alt.toFixed(0)} m · แผนที่ ${(m.grid.half * 2 / 1000).toFixed(0)} × ${(m.grid.half * 2 / 1000).toFixed(0)} km · จุดสูงสุด ${m.elev_max.toFixed(0)} m<br><small>${m.attribution.join('<br>')}</small>`;
}
async function setMap(id) {                            // สลับแผนที่จากเมนู
  $('map').disabled = true;
  try { terrain = await loadTerrain(id); } catch (err) { console.warn(err); terrain = procedural; }
  $('map').value = terrain.meta?.id ?? ''; $('map').disabled = false;
  world.setTerrain(terrain); ac = null; ap = null; cam.pos.set(0, 0, 0);
  planner.setTerrain(terrain); syncMission();
  world.preload(new THREE.Vector3(0, terrain.siteAlt + 3, 0));
  mapNote(); refreshMenu();
}
$('map').addEventListener('change', () => setMap($('map').value));
mapNote(); refreshMenu();

function newMission() {
  settings = readSettings();
  ac = new Aircraft(data, { cargoKg: settings.cargoKg, fuelFrac: settings.fuelFrac, wind: settings.wind, terrain: terrain.height, seed: (Date.now() & 0xffff) + 1 });
  ap = new Autopilot(data); syncMission();
  ap.afterTakeoff = settings.autoMission ? 'AUTO' : 'FBWA';
  const heading = settings.wind.fromDeg;                       // ตั้งรางหันสู้ลม
  ac.initRail({ heading });
  world.buildSite(heading, data.rail); world.setWind(settings.wind.fromDeg, settings.wind.speed);
  input.throttle = 0; input.roll = input.pitch = 0; input.takeEdges();
  if (settings.airStart) { ac.initAir({ V: 200 / KMH, alt: terrain.siteAlt + 400, heading }); ap.setMode(settings.autoMission && ap.mission.length ? 'AUTO' : 'CRUISE', ac); input.throttle = ac.ctl.throttle; }
  stats = { t: 0, dist: 0, maxV: 0, maxN: 1, minN: 1, maxAlt: 0, fuel0: fuelTotal(ac.fuel), railG: 0, railV: 0, airborne: settings.airStart };
  log = []; acc = 0; timeScale = 1; endTimer = 0; paused = false;
  cam.az = 0.5; cam.el = 0.28; cam.dist = 9; cam.mode = 0;
  hud.resetTrail();
  state = 'play';
  $('menu').classList.add('hide'); $('report').classList.add('hide'); $('help').classList.add('hide');
  const p = ac.s.pos; world.preload(new THREE.Vector3(p[1], -p[2], -p[0]));
}
$('start').addEventListener('click', () => { sound.start(); newMission(); });
$('again').addEventListener('click', () => newMission());
$('toMenu').addEventListener('click', () => { state = 'menu'; $('report').classList.add('hide'); $('menu').classList.remove('hide'); });
$('resume').addEventListener('click', () => { paused = false; $('help').classList.add('hide'); });
$('quit').addEventListener('click', () => { paused = false; state = 'menu'; $('help').classList.add('hide'); $('menu').classList.remove('hide'); });
$('csv').addEventListener('click', downloadCsv);

// กล้อง: ลากเมาส์ = หมุนรอบ · ล้อ = ระยะ
let drag = null;
$('hud').addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY]; });
window.addEventListener('pointerup', () => { drag = null; });
window.addEventListener('pointermove', e => { if (!drag) return; cam.az -= (e.clientX - drag[0]) * 0.006; cam.el = clamp(cam.el + (e.clientY - drag[1]) * 0.005, -0.3, 1.4); drag = [e.clientX, e.clientY]; });
window.addEventListener('wheel', e => { cam.dist = clamp(cam.dist * (e.deltaY > 0 ? 1.12 : 0.89), 1.6, 120); }, { passive: true });

// ---------- ปุ่มคำสั่ง ----------
function handleKeys() {
  for (const { code, shift } of input.takeEdges()) {
    if (state !== 'play') continue;
    if (planner.opened) { if (code === 'Escape' || code === 'KeyB') { planner.close(); paused = false; } continue; }
    if (code === 'Escape') { paused = !paused; $('help').classList.toggle('hide', !paused); continue; }
    if (paused) continue;
    switch (code) {
      case 'KeyE':
        if (shift) { ac.engine.shutdown(); say('ดับเครื่องยนต์'); }
        else if (ac.engine.state === 'off' || ac.engine.state === 'cool') { if (fuelTotal(ac.fuel) > 0.01) { ac.engine.start(settings.quickStart); say('เริ่มสตาร์ตเครื่องยนต์'); } else say('ไม่มีน้ำมัน'); }
        break;
      case 'KeyF': input.throttle = 1; break;
      case 'KeyG': input.throttle = 0; break;
      case 'Space':
        if (ac.phase === 'rail' && !ac.rail.launched) {
          if (ac.engine.thrustFrac < 0.9 && launchConfirmT <= 0) { launchConfirmT = 3; say('แรงขับยังไม่ถึง 90 % — กด Space อีกครั้งถ้าจะปล่อยทั้งอย่างนี้', 3); }
          else { ap.setMode('TAKEOFF'); ac.launch(); stats.airborne = true; say('ปล่อย!', 1.5); }
        }
        break;
      case 'KeyP':
        if (ac.phase === 'air' && ac.chute.state === 'stowed') {
          if (chuteConfirmT <= 0) { chuteConfirmT = 2.5; say('กด P อีกครั้งเพื่อกางร่ม (เครื่องยนต์จะดับ)', 2.5); }
          else { ac.engine.shutdown(); ac.deployChute(); stats.chuteV = ac.air.V; stats.chuteAgl = ac.agl; say('กางร่ม', 2); }
        }
        break;
      case 'KeyM': if (ac.phase === 'air') { const i = MODES.indexOf(ap.mode); ap.setMode(MODES[(i + 1) % MODES.length], ac); say('โหมด ' + ap.mode); } break;
      case 'Digit1': case 'Digit2': case 'Digit3': if (ac.phase === 'air') { ap.setMode(MODES[+code.slice(5) - 1], ac); say('โหมด ' + ap.mode); } break;
      case 'Digit4': if (ac.phase === 'air') { if (ap.mission.length) { ap.setMode('AUTO', ac); say(`โหมด AUTO — ไปจุด ${ap.wpIdx + 1}/${ap.mission.length}`); } else say('ยังไม่มีจุดบิน — กด B เพื่อวางแผนภารกิจ'); } break;
      case 'Digit5': case 'KeyH': if (ac.phase === 'air') { ap.setMode('RTL', ac); say(`โหมด RTL — กลับฐานที่ความสูง ${ap.rtlAlt.toFixed(0)} m`); } break;
      case 'KeyB': openPlanner(); break;
      case 'KeyC': cam.mode = (cam.mode + 1) % CAMS.length; say('กล้อง: ' + CAMS[cam.mode], 1.5); break;
      case 'KeyV': say(model.setXray(!model.xray) ? 'มุมมองโปร่ง: เห็นโครงและอุปกรณ์ข้างใน' : 'มุมมองปกติ', 1.8); break;
      case 'KeyT': timeScale = timeScale === 1 ? 2 : timeScale === 2 ? 4 : 1; say('ความเร็วเวลา ×' + timeScale, 1.5); break;
      case 'KeyN': sound.on = !sound.on; say(sound.on ? 'เปิดเสียง' : 'ปิดเสียง', 1.5); break;
      case 'KeyL': downloadCsv(); break;
      case 'KeyR': newMission(); break;
    }
  }
}

function simulate(dt) {
  acc = Math.min(acc + dt * timeScale, 0.25);
  const inp = { roll: input.roll, pitch: input.pitch, throttle: input.throttle, speedAdjust: input.speedAdjust };
  if (ap.mode === 'MANUAL') ap.trimDe = clamp(ap.trimDe - input.trimStep * 4 * dt * timeScale, -15, 10);
  while (acc >= H) {
    acc -= H;
    const wasRail = ac.phase === 'rail';
    ap.update(ac, inp, H); ac.step(H);
    if (wasRail && ac.phase === 'air') { stats.railG = ac.rail.peakG; stats.railV = ac.rail.v; }
    if (ac.phase === 'air') {
      stats.t += H; const vn = ac.vNED(); stats.dist += Math.hypot(vn[0], vn[1]) * H;
      stats.maxV = Math.max(stats.maxV, ac.air.V); stats.maxAlt = Math.max(stats.maxAlt, ac.alt);
      if (ac.chute.state === 'stowed') { stats.maxN = Math.max(stats.maxN, ac.nz); stats.minN = Math.min(stats.minN, ac.nz); }
    }
  }
  if (ap.mode === 'CRUISE' || ap.mode === 'AUTO' || ap.mode === 'RTL' || ac.chute.state !== 'stowed') input.throttle = ac.ctl.throttle;      // คันเร่งในมือตามค่าอัตโนมัติ (เปลี่ยนโหมดแล้วไม่กระตุก)
  if (ap.msg) { say(ap.msg); ap.msg = ''; }
  // บันทึกข้อมูลการบิน 4 Hz
  if (ac.phase !== 'rail' || ac.rail.launched) {
    const tl = ac.t; if (!log.length || tl - log[log.length - 1][0] >= 0.25) {
      const e = ac.euler();
      log.push([tl, ac.s.pos[0], ac.s.pos[1], ac.alt, ac.air.V * KMH, ac.air.alpha / DEG, ac.air.beta / DEG, e.roll / DEG, e.pitch / DEG, e.yaw / DEG, ac.nz, ac.ctl.throttle, ac.engine.thrust, fuelTotal(ac.fuel), ac.dL, ac.dR, ap.mode, ac.phase]);
    }
  }
}

function downloadCsv() {
  if (!log || !log.length) return;
  const head = 't_s,N_m,E_m,alt_m,V_kmh,alpha_deg,beta_deg,roll_deg,pitch_deg,yaw_deg,nz_g,throttle,thrust_N,fuel_kg,elevonL_deg,elevonR_deg,mode,phase';
  const body = log.map(r => r.map(v => typeof v === 'number' ? +v.toFixed(4) : v).join(',')).join('\n');
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([head + '\n' + body], { type: 'text/csv' }));
  a.download = `FW57JB_flight_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`; a.click(); URL.revokeObjectURL(a.href);
}

function showReport() {
  const td = ac.touchdown, ok = ac.phase === 'landed';
  let title, cls;
  if (ok && td.type === 'chute') { title = td.water ? 'ลงน้ำด้วยร่ม' : 'ลงจอดด้วยร่มสำเร็จ'; cls = td.water ? 'warn' : 'ok'; }
  else if (ok) { title = 'ลงท้องไถล (เครื่องไม่มีล้อ — โครงเสียหาย)'; cls = 'warn'; }
  else { title = 'เครื่องตก: ' + (ac.crashReason || ''); cls = 'bad'; }
  const row = (a, b) => `<div><b>${a}</b><span>${b}</span></div>`;
  let html = row('เวลาบิน', fmtT(stats.t) + ' นาที') + row('ระยะทางบิน', (stats.dist / 1000).toFixed(2) + ' km')
    + row('ความเร็วสูงสุด', (stats.maxV * KMH).toFixed(0) + ' km/h') + row('ความสูงสูงสุด', stats.maxAlt.toFixed(0) + ' m')
    + row('โหลดแฟกเตอร์', `${stats.minN.toFixed(2)} ถึง ${stats.maxN.toFixed(2)} g (ขีด ${data.limits.n_limit})`)
    + row('น้ำมันที่ใช้', (stats.fuel0 - fuelTotal(ac.fuel)).toFixed(2) + ' kg (เหลือ ' + fuelTotal(ac.fuel).toFixed(2) + ')');
  if (stats.railV) html += row('ออกราง', `${(stats.railV * KMH).toFixed(1)} km/h · สูงสุด ${stats.railG.toFixed(1)} g`);
  if (stats.chuteV) html += row('กางร่มที่', `${(stats.chuteV * KMH).toFixed(0)} km/h · สูงจากพื้น ${stats.chuteAgl.toFixed(0)} m · แรงกระตุก ${ac.chute.forceMax.toFixed(0)} N (ขีด ${data.chute.F_limit_N.toFixed(0)})`);
  if (td) html += row('แตะพื้น', `ลง ${td.vz.toFixed(1)} m/s · ไถล ${td.vh.toFixed(1)} m/s`);
  html += row('ห่างจุดปล่อย', (Math.hypot(ac.s.pos[0], ac.s.pos[1]) / 1000).toFixed(2) + ' km');
  $('repTitle').textContent = title; $('repTitle').className = cls; $('repBody').innerHTML = html;
  $('report').classList.remove('hide');
}

// ---------- กล้อง ----------
const _v = new THREE.Vector3(), _t = new THREE.Vector3();
function updateCamera(dt) {
  const p = model.group.position;
  if (state === 'menu') {
    cam.menuT += dt;
    const a = cam.menuT * 0.12 + 0.6, d = 11;
    camera.position.set(p.x + Math.sin(a) * d, p.y + 2.6, p.z + Math.cos(a) * d); camera.fov = 55; camera.lookAt(p); camera.updateProjectionMatrix();
    return;
  }
  const yaw = ac.euler().yaw;
  if (cam.mode === 1) {                               // หัวเครื่อง
    camera.quaternion.copy(model.group.quaternion);
    camera.position.copy(p).add(_v.set(0, 0.22, -0.55).applyQuaternion(model.group.quaternion));
    camera.fov = 70;
  } else if (cam.mode === 2) {                        // ภาคพื้น (ยืนข้างสถานีควบคุม ซูมตามระยะ)
    camera.position.set(11, terrain.siteAlt + 1.7, 12);
    const d = camera.position.distanceTo(p);
    camera.fov = clamp(2 * Math.atan(9 / d) / DEG, 1.2, 55); camera.lookAt(p);
  } else {                                            // ตามหลัง
    const a = -yaw + cam.az - 0.5, el = cam.el, d = cam.dist;       // az 0.5 = อยู่หลังเครื่องพอดี
    _t.set(p.x + Math.sin(a) * Math.cos(el) * d, p.y + Math.sin(el) * d, p.z + Math.cos(a) * Math.cos(el) * d);
    const ground = Math.max(terrain.height(-_t.z, _t.x), 0) + 0.6; if (_t.y < ground) _t.y = ground;
    const k = ac.phase === 'rail' ? 1 : 1 - Math.exp(-dt * 9 * timeScale);      // เร่งเวลาแล้วกล้องต้องตามเร็วขึ้นด้วย ไม่งั้นเครื่องหลุดไกล
    // cam.pos = ระยะเยื้องจากเครื่อง (ไม่ใช่ตำแหน่งในโลก) → ซูมใกล้ดูรายละเอียดตอนบินเร็วได้ กล้องไม่ถูกทิ้งห่างตามความเร็ว
    _t.sub(p); cam.pos.lerp(_t, cam.pos.lengthSq() ? k : 1); camera.position.copy(p).add(cam.pos);
    camera.fov = 62; camera.lookAt(_v.copy(p).add(_t.set(0, Math.min(0.6, 0.067 * d), 0)));
  }
  camera.updateProjectionMatrix();
}

function hudState(dt) {
  const D = data, w = [], eng = ac.engine, air = ac.phase === 'air', stowed = ac.chute.state === 'stowed';
  const vsNow = Math.sqrt(2 * ac.mp.m * G / (ac.air.rho * D.aero.S * D.aero.CLmax_design));
  if (air && stowed) {
    if (ac.air.sigma > 0.5) { w.push(['ร่วงหล่น — STALL', 2]); sound.beep(660); }
    else if (ac.air.alpha > 13 * DEG || ac.air.V < 1.08 * vsNow) w.push(['ใกล้ร่วงหล่น', 1]);
    if (ac.air.V * KMH > D.limits.V_NE_kmh) { w.push(['เกิน V_NE — ลดความเร็ว', 2]); sound.beep(990); }
    if (Math.abs(ac.nz) > D.limits.n_limit) w.push(['เกินขีด g', 2]);
    if (ac.agl < 80 && ac.vNED()[2] > 3) { w.push(['ต่ำ! ดึงขึ้น', 2]); sound.beep(1200); }
    if (eng.state !== 'run' && eng.state !== 'start') w.push([eng.flameout ? 'เครื่องยนต์ดับ — น้ำมันหมด' : 'เครื่องยนต์ดับ', 2]);
  }
  if (fuelTotal(ac.fuel) < 1.5 && ac.phase !== 'landed') w.push(['น้ำมันต่ำ', 1]);
  if (ac.phase === 'rail' && eng.groundFullTime > 60) w.push([`เต็มคันเร่งบนราง ${eng.groundFullTime.toFixed(0)} / ${D.engine.ground_full_throttle_max_s} s`, eng.groundFullTime > D.engine.ground_full_throttle_max_s ? 2 : 1]);
  if (ac.chute.state === 'inflating') w.push(['ร่มกำลังกาง', 1]);
  if (ac.chute.state === 'open') w.push([`ร่มกางแล้ว — ลง ${ac.vNED()[2].toFixed(1)} m/s`, 1]);
  if (ac.chute.state === 'failed') w.push(['สายร่มขาด — กางเร็วเกินไป', 2]);
  if (air && stowed && ac.agl > 80) {                  // มองไปข้างหน้า 24 s ตามแนวบิน: พื้นจะสูงถึงเครื่องไหม
    const t = terrainAhead(terrain.height, ac.s.pos, ac.vNED());
    if (t) { w.push([`ภูมิประเทศข้างหน้า ${t} s — ไต่ขึ้นหรือเลี้ยว`, 2]); sound.beep(1200); }
  }
  const outside = !terrain.inside(ac.s.pos[0], ac.s.pos[1]);
  if (outside) w.push(['ออกนอกขอบแผนที่ — ไม่มีข้อมูลพื้นจริง', 1]);

  let banner = '', lines = [];
  if (ac.phase === 'rail' && !ac.rail.launched) {
    banner = 'อยู่บนรางปล่อย — เตรียมปล่อย';
    const st = eng.state === 'run' ? '✔ เครื่องทำงาน' : eng.state === 'start' ? `… ${eng.startStage}` : 'กด E';
    lines = [`1) สตาร์ตเครื่องยนต์ — ${st}`, `2) เร่งเต็ม (กด F หรือกด X ค้าง) — แรงขับ ${eng.thrust.toFixed(0)} / 125 N`, '3) กด Space เพื่อปล่อย'];
  }
  const names = { MANUAL: 'MANUAL บังคับตรง', FBWA: 'FBWA ช่วยทรงตัว', CRUISE: 'CRUISE รักษาระดับ', TAKEOFF: 'TAKEOFF ออกราง', AUTO: 'AUTO บินตามจุด', RTL: 'RTL กลับฐาน' };
  const tgt = `${(ap.spdT * KMH).toFixed(0)} km/h · ${ap.altT.toFixed(0)} m`, wpTo = ap.mission[ap.wpIdx];
  const target = !air || !stowed ? '' : ap.mode === 'CRUISE' ? `เป้า: ${tgt} · ${(((ap.hdgT / DEG) % 360 + 360) % 360).toFixed(0)}°`
    : ap.mode === 'AUTO' && wpTo ? `จุด ${ap.wpIdx + 1}/${ap.mission.length} อีก ${(Math.hypot(wpTo.n - ac.s.pos[0], wpTo.e - ac.s.pos[1]) / 1000).toFixed(1)} km · ${tgt}`
    : ap.mode === 'RTL' ? `${ap.rtlArrived ? 'วนรอเหนือฐาน' : 'กลับฐาน'} · ${tgt}` : '';
  const phase = { rail: ac.rail?.launched ? 'กำลังวิ่งบนราง' : 'บนราง', air: stowed ? 'กำลังบิน' : 'ลงด้วยร่ม', landed: 'ถึงพื้นแล้ว', crashed: 'ตก' }[ac.phase];
  toastT -= dt; launchConfirmT -= dt; chuteConfirmT -= dt;
  const N = ac.s.pos[0], E = ac.s.pos[1], hd = Math.hypot(N, E);
  return {
    hidden: false, fpv: cam.mode === 1, fovDeg: camera.fov, vsKmh: vsNow * KMH, warnings: w, banner, bannerLines: lines,
    modeName: names[ap.mode], phaseText: phase, target, route: ap.mission, routeIdx: ap.mode === 'AUTO' ? ap.wpIdx : -1, tFlight: stats.t, dist: stats.dist, timeScale,
    homeBrg: hd > 50 ? ((Math.atan2(-E, -N) / DEG) + 360) % 360 : null, homeDist: hd,
    mapRange: hd > 9000 ? 30000 : hd > 3000 ? 10000 : 4000, terrain, wind: ac.wind, agl: ac.agl,
    coord: terrain.meta ? (ll => `${ll.lat.toFixed(4)}°N ${ll.lon.toFixed(4)}°E`)(neToLatLon(terrain.meta, N, E)) : '',
    toast: toastT > 0 ? toast : '',
    hint: paused ? '' : 'W/S ก้ม-เงย · A/D เอียง · X/Z คันเร่ง · M โหมด · 4 AUTO · 5 RTL · B แผนที่ · C กล้อง · P ร่ม · T เร่งเวลา · Esc ช่วยเหลือ',
  };
}

// ---------- วงรอบหลัก ----------
let last = performance.now();
function tick(dt) {                                   // เดินเกม 1 เฟรม (ไม่วาด)
  input.update(dt);
  if (state === 'play') {
    handleKeys();
    if (!paused) simulate(dt);
    if ((ac.phase === 'landed' || ac.phase === 'crashed') && $('report').classList.contains('hide')) { endTimer += dt; if (endTimer > 2.2) showReport(); }
    model.update(ac, dt);
    world.updateRail(ac, dt, data.rail);
    sound.update(ac.engine.n, ac.engine.thrustFrac, ac.air.V, paused);
  } else {
    input.takeEdges();
    if (!ac) { ac = new Aircraft(data, { terrain: terrain.height }); ap = new Autopilot(data); world.buildSite(0, data.rail); syncMission(); }
    model.update(ac, dt); sound.update(0, 0, 0, true);
  }
  updateCamera(dt);
}
function draw(dt, budget = 5) {
  world.update(camera.position, budget);
  renderer.render(scene, camera);
  if (state === 'play') hud.draw(ac, ap, hudState(dt), dt); else hud.g.clearRect(0, 0, hud.w, hud.h);
}
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.1); last = now;
  tick(dt); draw(dt);
  requestAnimationFrame(frame);
}
world.preload(new THREE.Vector3(0, terrain.siteAlt + 3, 0));
$('loading').classList.add('hide'); $('menu').classList.remove('hide');
requestAnimationFrame(frame);
// ตัวช่วยทดสอบ: เดินเวลาเกมไปข้างหน้า sec วินาทีทันที แล้ววาด 1 เฟรม
const advance = sec => { for (let i = 0; i < Math.round(sec * 60); i++) tick(1 / 60); draw(1 / 60, Infinity); };
window.__sim = { advance, get stats() { return stats; }, get ac() { return ac; }, get ap() { return ap; }, input, cam, world, camera, model, get state() { return state; }, get terrain() { return terrain; }, setMap, planner, openPlanner, syncMission, newMission, data, set timeScale(v) { timeScale = v; } };
