// ปุ่มบังคับ: คีย์บอร์ด (ทำให้นุ่มเหมือนคันบังคับ) + จอยเกม (ถ้าเสียบอยู่)
import { clamp } from './sim/core.js';

const BLOCK = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space', 'Tab']);

export class Input {
  constructor() {
    this.down = new Set(); this.edges = [];
    this.roll = 0; this.pitch = 0; this.throttle = 0; this.speedAdjust = 0; this.trimStep = 0;
    window.addEventListener('keydown', e => {
      if (e.target instanceof HTMLInputElement && e.target.type !== 'range' && e.target.type !== 'checkbox') return;
      if (BLOCK.has(e.code)) e.preventDefault();
      if (!e.repeat) this.edges.push({ code: e.code, shift: e.shiftKey });
      this.down.add(e.code);
    });
    window.addEventListener('keyup', e => this.down.delete(e.code));
    window.addEventListener('blur', () => this.down.clear());
  }
  /** คืนรายการปุ่มที่เพิ่งกด (ครั้งเดียวต่อการกด) */
  takeEdges() { const e = this.edges; this.edges = []; return e; }
  key(...codes) { return codes.some(c => this.down.has(c)); }
  update(dt) {
    // แกนเอียง/ก้มเงย: กดค้าง = ค่อย ๆ โยก · ปล่อย = คืนกลาง
    const tr = (this.key('KeyD', 'ArrowRight') ? 1 : 0) - (this.key('KeyA', 'ArrowLeft') ? 1 : 0);
    const tp = (this.key('KeyS', 'ArrowDown') ? 1 : 0) - (this.key('KeyW', 'ArrowUp') ? 1 : 0);     // S/↓ = ดึงคันบังคับ (เงยหัว)
    const move = (v, t) => v + clamp(t - v, -(t === 0 ? 4 : 2.6) * dt, (t === 0 ? 4 : 2.6) * dt);
    this.roll = move(this.roll, tr); this.pitch = move(this.pitch, tp);
    const dth = (this.key('KeyX') ? 1 : 0) - (this.key('KeyZ') ? 1 : 0);
    this.throttle = clamp(this.throttle + dth * 0.45 * dt, 0, 1);
    this.speedAdjust = dth * 8;                                   // โหมด CRUISE: ปรับความเร็วเป้า (m/s ต่อวินาที)
    this.trimStep = (this.key('BracketRight') ? 1 : 0) - (this.key('BracketLeft') ? 1 : 0);
    // จอยเกม: คันซ้าย = เอียง/ก้มเงย · ไกขวา/ซ้าย = เพิ่ม/ลดคันเร่ง
    const pad = navigator.getGamepads ? [...navigator.getGamepads()].find(p => p && p.connected) : null;
    if (pad) {
      const dz = v => (Math.abs(v) < 0.08 ? 0 : v);
      const gx = dz(pad.axes[0] || 0), gy = dz(pad.axes[1] || 0);
      if (gx || gy) { this.roll = gx; this.pitch = gy; }
      const up = pad.buttons[7]?.value || 0, dn = pad.buttons[6]?.value || 0;
      if (up || dn) { this.throttle = clamp(this.throttle + (up - dn) * 0.6 * dt, 0, 1); this.speedAdjust = (up - dn) * 8; }
    }
  }
}
