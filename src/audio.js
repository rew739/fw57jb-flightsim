// เสียง: เครื่องยนต์ไอพ่น (เสียงหวีด + เสียงซ่า ตามรอบ) + เสียงลมตามความเร็ว + เสียงเตือน — สังเคราะห์ด้วย WebAudio ไม่ใช้ไฟล์เสียง
export class Sound {
  constructor() { this.ctx = null; this.on = true; this.beepT = 0; }
  start() {
    if (this.ctx) { this.ctx.resume(); return; }
    const AC = window.AudioContext || window.webkitAudioContext; if (!AC) return;
    const c = this.ctx = new AC();
    const noiseBuf = c.createBuffer(1, c.sampleRate * 2, c.sampleRate), d = noiseBuf.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
    const noise = () => { const s = c.createBufferSource(); s.buffer = noiseBuf; s.loop = true; s.start(); return s; };
    this.master = c.createGain(); this.master.gain.value = 0.5; this.master.connect(c.destination);
    // เสียงหวีดกังหัน
    this.whine = c.createOscillator(); this.whine.type = 'sawtooth'; this.whineG = c.createGain(); this.whineG.gain.value = 0;
    const wf = c.createBiquadFilter(); wf.type = 'bandpass'; wf.Q.value = 6; this.whineF = wf;
    this.whine.connect(wf); wf.connect(this.whineG); this.whineG.connect(this.master); this.whine.start();
    // เสียงซ่าไอพ่น
    this.jetF = c.createBiquadFilter(); this.jetF.type = 'lowpass'; this.jetG = c.createGain(); this.jetG.gain.value = 0;
    noise().connect(this.jetF); this.jetF.connect(this.jetG); this.jetG.connect(this.master);
    // เสียงลม
    this.windF = c.createBiquadFilter(); this.windF.type = 'bandpass'; this.windF.Q.value = 0.6; this.windG = c.createGain(); this.windG.gain.value = 0;
    noise().connect(this.windF); this.windF.connect(this.windG); this.windG.connect(this.master);
  }
  update(n, thrustFrac, V, paused) {
    const c = this.ctx; if (!c) return;
    const t = c.currentTime, k = this.on && !paused ? 1 : 0;
    this.whine.frequency.setTargetAtTime(300 + 2400 * n, t, 0.05); this.whineF.frequency.setTargetAtTime(600 + 3000 * n, t, 0.05);
    this.whineG.gain.setTargetAtTime(k * 0.05 * Math.min(1, n * 3), t, 0.08);
    this.jetF.frequency.setTargetAtTime(300 + 2600 * thrustFrac, t, 0.08); this.jetG.gain.setTargetAtTime(k * (0.02 * Math.min(1, n * 3) + 0.3 * thrustFrac), t, 0.08);
    this.windF.frequency.setTargetAtTime(200 + 9 * V, t, 0.1); this.windG.gain.setTargetAtTime(k * Math.min(0.25, (V / 90) ** 2 * 0.22), t, 0.1);
  }
  beep(freq = 880, dur = 0.12) {
    const c = this.ctx; if (!c || !this.on || c.currentTime < this.beepT) return;
    this.beepT = c.currentTime + dur + 0.25;
    const o = c.createOscillator(), g = c.createGain(); o.frequency.value = freq; o.type = 'square'; g.gain.value = 0.06;
    o.connect(g); g.connect(this.master); o.start(); o.stop(c.currentTime + dur);
  }
}
