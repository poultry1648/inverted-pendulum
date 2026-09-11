import './style.css';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connect');
const disconnectBtn = document.getElementById('disconnect');
const cartEl = document.getElementById('cart-value');
const potEl = document.getElementById('pot-value');
const potRawEl = document.getElementById('pot-raw');
const maxTravelEl = document.getElementById('max-travel');
const logEl = document.getElementById('log');
const goZeroBtn = document.getElementById('go-zero');
const homeBtn = document.getElementById('home');
const moveButtons = Array.from(document.querySelectorAll('button[data-delta]'));

const stopBtn = document.getElementById('stop');
const goBtn = document.getElementById('go');
const resetBtn = document.getElementById('reset');
const modeSelect = document.getElementById('mode-select');
const velInput = document.getElementById('vel-input');
const ampInput = document.getElementById('amp-input');
const freqInput = document.getElementById('freq-input');
const kpInput = document.getElementById('kp-input');
const applyParamsBtn = document.getElementById('apply-params');
const balanceBtn = document.getElementById('balance');
const balanceStateEl = document.getElementById('balance-state');

const calmodeBtn = document.getElementById('calmode');
const zeroBtn = document.getElementById('zero');
const calAngleInput = document.getElementById('cal-angle');
const calBtn = document.getElementById('cal');
const calstatusBtn = document.getElementById('calstatus');
const cmdInput = document.getElementById('cmd-input');
const cmdSendBtn = document.getElementById('cmd-send');

/* Every field of the firmware's telemetry_t. x_mm/theta_deg/raw arrive on the
 * 5 Hz status line; the rest (plus x_steps) on the 1 Hz dbg line. */
const TEL_FIELDS = {
  mode:           { label: 'mode',             digits: null },
  fault:          { label: 'fault',            digits: null },
  x_steps:        { label: 'x_steps',          digits: 0 },
  x_mm:           { label: 'x (mm)',           digits: 2 },
  theta_deg:      { label: 'theta (deg)',      digits: 1 },
  raw:            { label: 'raw',              digits: 0 },
  xdot:           { label: 'xdot (mm/s)',      digits: 1 },
  thetadot:       { label: 'thetadot (deg/s)', digits: 1 },
  v_cmd:          { label: 'v_cmd (mm/s)',     digits: 1 },
  tick:           { label: 'tick',             digits: 0 },
  period_min_us:  { label: 'period min (us)',  digits: 0 },
  period_mean_us: { label: 'period mean (us)', digits: 0 },
  period_max_us:  { label: 'period max (us)',  digits: 0 },
  missed:         { label: 'missed',           digits: 0 },
  lqr_state:      { label: 'lqr_state',        digits: null },
  x_ref_mm:       { label: 'x_ref (mm)',       digits: 2 },
  lqr_xi:         { label: 'lqr_xi',           digits: 3 },
};

const telemetryEl = document.getElementById('telemetry');
const telEls = {};
for (const [key, { label }] of Object.entries(TEL_FIELDS)) {
  const row = document.createElement('div');
  row.className = 'field';
  row.innerHTML = `<span class="field-label">${label}</span><span class="field-value">--</span>`;
  telemetryEl.append(row);
  telEls[key] = row.lastElementChild;
}

function setTel(key, value) {
  const el = telEls[key];
  if (!el) return;
  const { digits } = TEL_FIELDS[key];
  el.textContent =
    typeof value === 'number' && digits !== null ? value.toFixed(digits) : String(value);
  el.classList.toggle(
    'bad',
    (key === 'fault' && value === 'FAULT') || (key === 'missed' && Number(value) > 0)
  );
}

function resetTelemetry() {
  for (const el of Object.values(telEls)) {
    el.textContent = '--';
    el.classList.remove('bad');
  }
}

const LQR_STATE_TEXT = ['off', 'waiting for pole...', 'balancing'];
let lqrState = 0;

function setLqrState(value) {
  const n = Number(value);
  lqrState = Number.isFinite(n) ? n : 0;
  const text = LQR_STATE_TEXT[lqrState] ?? '?';
  balanceStateEl.textContent = text;
  setTel('lqr_state', text);
  const on = lqrState !== 0;
  balanceBtn.classList.toggle('active', on);
  balanceBtn.textContent = on ? 'Stop balance' : 'Balance';
}

const MAX_LOG_LINES = 200;
const DEFAULT_MAX_TRAVEL = 350;
const encoder = new TextEncoder();

let port = null;
let reader = null;
let writer = null;
let readableClosed = null;
let disconnecting = false;
let currentPos = null;
let maxTravel = DEFAULT_MAX_TRAVEL;
let calmodeOn = false;

function setStatus(text, state) {
  statusEl.textContent = text;
  statusEl.className = `status status--${state}`;
}

function log(line) {
  logEl.textContent += `${line}\n`;
  const lines = logEl.textContent.split('\n');
  if (lines.length > MAX_LOG_LINES) {
    logEl.textContent = lines.slice(lines.length - MAX_LOG_LINES).join('\n');
  }
  logEl.scrollTop = logEl.scrollHeight;
}

function updateControls() {
  const connected = port !== null && writer !== null;
  connectBtn.disabled = connected;
  disconnectBtn.disabled = !connected;
  goZeroBtn.disabled = !connected;
  homeBtn.disabled = !connected;
  const canJog = connected && currentPos !== null;
  for (const btn of moveButtons) btn.disabled = !canJog;

  for (const el of [
    stopBtn, goBtn, resetBtn, modeSelect, velInput, ampInput, freqInput,
    kpInput, applyParamsBtn, balanceBtn, calmodeBtn, zeroBtn, calAngleInput,
    calBtn, calstatusBtn, cmdInput, cmdSendBtn,
  ]) {
    el.disabled = !connected;
  }
}

function setCart(mm) {
  currentPos = mm;
  cartEl.textContent = mm.toFixed(2);
  updateControls();
}

function handleLine(rawLine) {
  const line = rawLine.replace(/\r$/, '');
  if (line.length === 0) return;

  log(line);

  const rangeMatch = line.match(/Range\s+0\.\.(\d+(?:\.\d+)?)\s*mm/);
  if (rangeMatch) {
    maxTravel = Number(rangeMatch[1]);
    maxTravelEl.textContent = maxTravel.toFixed(0);
  }

  const posMatch = line.match(/pos=\s*(-?\d+(?:\.\d+)?)\s*mm/);
  if (posMatch) {
    setCart(Number(posMatch[1]));
    setTel('x_mm', Number(posMatch[1]));
  }

  const movedMatch = line.match(/(?:moved (?:left|right) to|already at)\s+(-?\d+(?:\.\d+)?)\s*mm/);
  if (movedMatch) setCart(Number(movedMatch[1]));

  const homedMatch = line.match(/Homing: backed off to\s+(-?\d+(?:\.\d+)?)\s*mm/);
  if (homedMatch) setCart(Number(homedMatch[1]));

  const potMatch = line.match(/raw=\s*(\d+)\s+(-?\d+(?:\.\d+)?)\s*deg/);
  if (potMatch) {
    potRawEl.textContent = potMatch[1];
    potEl.textContent = Number(potMatch[2]).toFixed(1);
    setTel('raw', Number(potMatch[1]));
    setTel('theta_deg', Number(potMatch[2]));
  }

  const calMatch = line.match(/calstatus:.*calmode=(\w+)/);
  if (calMatch) {
    calmodeOn = calMatch[1] === 'on';
    calmodeBtn.textContent = `Calmode: ${calmodeOn ? 'on' : 'off'}`;
    calmodeBtn.classList.toggle('active', calmodeOn);
  }

  const dbgMatch = line.match(
    /dbg tick=(\d+) steps=(-?\d+) jit=(\d+)\/(\d+)\/(\d+) us miss=(\d+) fault=(\d+) mode=(\S+) lqr=(\d+) xref=(-?\d+(?:\.\d+)?) xi=(-?\d+(?:\.\d+)?) v=(-?\d+(?:\.\d+)?) xd=(-?\d+(?:\.\d+)?) thd=(-?\d+(?:\.\d+)?)/
  );
  if (dbgMatch) {
    const [, tick, steps, jmin, jmean, jmax, miss, fault, mode, lqr, xref, xi, v, xd, thd] = dbgMatch;
    setTel('tick', Number(tick));
    setTel('x_steps', Number(steps));
    setTel('period_min_us', Number(jmin));
    setTel('period_mean_us', Number(jmean));
    setTel('period_max_us', Number(jmax));
    setTel('missed', Number(miss));
    setTel('fault', fault === '1' ? 'FAULT' : 'ok');
    setTel('mode', mode);
    setLqrState(lqr);
    setTel('x_ref_mm', Number(xref));
    setTel('lqr_xi', Number(xi));
    setTel('v_cmd', Number(v));
    setTel('xdot', Number(xd));
    setTel('thetadot', Number(thd));
  }
}

async function sendCoord(mm) {
  if (!writer) return;
  const target = Math.min(Math.max(mm, 0), maxTravel);
  const text = `${target.toFixed(2)}\n`;
  try {
    await writer.write(encoder.encode(text));
    log(`>> ${text.trim()} mm`);
  } catch (err) {
    log(`Write error: ${err.message}`);
  }
}

async function sendHome() {
  if (!writer) return;
  try {
    await writer.write(encoder.encode('home\n'));
    log('>> home');
  } catch (err) {
    log(`Write error: ${err.message}`);
  }
}

/* Send any raw firmware command, exactly as typed into the terminal. */
async function sendCommand(text) {
  if (!writer) return;
  const cmd = text.trim();
  if (!cmd) return;
  try {
    await writer.write(encoder.encode(`${cmd}\n`));
    log(`>> ${cmd}`);
  } catch (err) {
    log(`Write error: ${err.message}`);
  }
}

async function toggleCalmode() {
  calmodeOn = !calmodeOn;
  calmodeBtn.textContent = `Calmode: ${calmodeOn ? 'on' : 'off'}`;
  calmodeBtn.classList.toggle('active', calmodeOn);
  await sendCommand(`calmode ${calmodeOn ? 'on' : 'off'}`);
}

async function connect() {
  if (!('serial' in navigator)) {
    setStatus('Web Serial unsupported', 'error');
    log('Web Serial is not available. Use Chrome/Edge on localhost.');
    return;
  }

  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
  } catch (err) {
    if (err && err.name !== 'NotFoundError') {
      setStatus('connection failed', 'error');
      log(`Error: ${err.message}`);
    }
    if (writer) {
      try { writer.releaseLock(); } catch { /* ignore */ }
      writer = null;
    }
    if (port) {
      try { await port.close(); } catch { /* ignore */ }
      port = null;
    }
    updateControls();
    return;
  }

  disconnecting = false;
  currentPos = null;
  resetTelemetry();
  setLqrState(0);
  setStatus('connected', 'on');
  log('--- connected (115200) ---');
  updateControls();

  const decoder = new TextDecoderStream();
  readableClosed = port.readable.pipeTo(decoder.writable).catch(() => {});
  reader = decoder.readable.getReader();

  let buffer = '';
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += value;
      let idx;
      while ((idx = buffer.indexOf('\n')) >= 0) {
        handleLine(buffer.slice(0, idx));
        buffer = buffer.slice(idx + 1);
      }
    }
  } catch (err) {
    if (!disconnecting) {
      setStatus('read error', 'error');
      log(`Read error: ${err.message}`);
    }
  } finally {
    await cleanup();
  }
}

async function cleanup() {
  if (reader) {
    try { reader.releaseLock(); } catch { /* ignore */ }
    reader = null;
  }
  if (readableClosed) {
    try { await readableClosed; } catch { /* ignore */ }
    readableClosed = null;
  }
  if (writer) {
    try { writer.releaseLock(); } catch { /* ignore */ }
    writer = null;
  }
  if (port) {
    try { await port.close(); } catch { /* ignore */ }
    port = null;
  }
  currentPos = null;
  setStatus('disconnected', 'off');
  updateControls();
}

async function disconnect() {
  if (!port) return;
  disconnecting = true;
  disconnectBtn.disabled = true;
  if (reader) {
    try { await reader.cancel(); } catch { /* ignore */ }
  } else {
    await cleanup();
  }
  log('--- disconnected ---');
}

for (const btn of moveButtons) {
  btn.addEventListener('click', () => {
    if (currentPos === null) return;
    sendCoord(currentPos + Number(btn.dataset.delta));
  });
}

goZeroBtn.addEventListener('click', () => sendCoord(0));
homeBtn.addEventListener('click', sendHome);
connectBtn.addEventListener('click', connect);
disconnectBtn.addEventListener('click', disconnect);

stopBtn.addEventListener('click', () => sendCommand('stop'));
goBtn.addEventListener('click', () => sendCommand('go'));
balanceBtn.addEventListener('click', () => {
  sendCommand(lqrState === 0 ? 'balance' : 'stop');
});
resetBtn.addEventListener('click', () => sendCommand('reset'));
applyParamsBtn.addEventListener('click', async () => {
  await sendCommand(`mode ${modeSelect.value}`);
  await sendCommand(`vel ${Number(velInput.value)}`);
  await sendCommand(`amp ${Number(ampInput.value)}`);
  await sendCommand(`freq ${Number(freqInput.value)}`);
  await sendCommand(`kp ${Number(kpInput.value)}`);
});

calmodeBtn.addEventListener('click', toggleCalmode);
zeroBtn.addEventListener('click', () => sendCommand('zero'));
calBtn.addEventListener('click', () => sendCommand(`cal ${Number(calAngleInput.value)}`));
calstatusBtn.addEventListener('click', () => sendCommand('calstatus'));

cmdSendBtn.addEventListener('click', () => {
  sendCommand(cmdInput.value);
  cmdInput.value = '';
});
cmdInput.addEventListener('keydown', (event) => {
  if (event.key === 'Enter') {
    sendCommand(cmdInput.value);
    cmdInput.value = '';
  }
});
