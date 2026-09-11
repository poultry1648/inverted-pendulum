import './style.css';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connect');
const disconnectBtn = document.getElementById('disconnect');
const cartEl = document.getElementById('cart-value');
const potEl = document.getElementById('pot-value');
const potRawEl = document.getElementById('pot-raw');
const maxTravelEl = document.getElementById('max-travel');
const logEl = document.getElementById('log');
const homeBtn = document.getElementById('home');
const moveButtons = Array.from(document.querySelectorAll('button[data-delta]'));

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
  homeBtn.disabled = !connected;
  const canJog = connected && currentPos !== null;
  for (const btn of moveButtons) btn.disabled = !canJog;
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
  if (posMatch) setCart(Number(posMatch[1]));

  const movedMatch = line.match(/(?:moved (?:left|right) to|already at)\s+(-?\d+(?:\.\d+)?)\s*mm/);
  if (movedMatch) setCart(Number(movedMatch[1]));

  const potMatch = line.match(/raw=\s*(\d+)\s+(-?\d+(?:\.\d+)?)\s*deg/);
  if (potMatch) {
    potRawEl.textContent = potMatch[1];
    potEl.textContent = Number(potMatch[2]).toFixed(1);
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

homeBtn.addEventListener('click', () => sendCoord(0));
connectBtn.addEventListener('click', connect);
disconnectBtn.addEventListener('click', disconnect);
