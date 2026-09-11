import './style.css';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connect');
const disconnectBtn = document.getElementById('disconnect');
const cartEl = document.getElementById('cart-value');
const potEl = document.getElementById('pot-value');
const potRawEl = document.getElementById('pot-raw');
const logEl = document.getElementById('log');

const MAX_LOG_LINES = 200;

let port = null;
let reader = null;
let readableClosed = null;
let disconnecting = false;

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

function handleLine(rawLine) {
  const line = rawLine.replace(/\r$/, '');
  if (line.length === 0) return;

  log(line);

  const posMatch = line.match(/pos=\s*(-?\d+(?:\.\d+)?)\s*mm/);
  if (posMatch) cartEl.textContent = Number(posMatch[1]).toFixed(2);

  const movedMatch = line.match(/(?:moved (?:left|right) to|already at)\s+(-?\d+(?:\.\d+)?)\s*mm/);
  if (movedMatch) cartEl.textContent = Number(movedMatch[1]).toFixed(2);

  const potMatch = line.match(/raw=\s*(\d+)\s+(-?\d+(?:\.\d+)?)\s*deg/);
  if (potMatch) {
    potRawEl.textContent = potMatch[1];
    potEl.textContent = Number(potMatch[2]).toFixed(1);
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
  } catch (err) {
    if (err && err.name !== 'NotFoundError') {
      setStatus('connection failed', 'error');
      log(`Error: ${err.message}`);
    }
    port = null;
    return;
  }

  disconnecting = false;
  connectBtn.disabled = true;
  disconnectBtn.disabled = false;
  setStatus('connected', 'on');
  log('--- connected (115200) ---');

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
    reader.releaseLock();
    reader = null;
    await readableClosed;
    readableClosed = null;
    await closePort();
  }
}

async function closePort() {
  if (!port) return;
  try {
    await port.close();
  } catch {
    // already closed
  }
  port = null;
}

async function disconnect() {
  if (!port) return;
  disconnecting = true;
  disconnectBtn.disabled = true;
  if (reader) await reader.cancel();
  await closePort();
  connectBtn.disabled = false;
  setStatus('disconnected', 'off');
  log('--- disconnected ---');
}

connectBtn.addEventListener('click', connect);
disconnectBtn.addEventListener('click', disconnect);
