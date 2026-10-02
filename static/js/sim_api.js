async function setPID(mode, prefix) {
    const type = document.getElementById(prefix + 'CtrlType').value;
    const val1 = parseFloat(document.getElementById(prefix + '1').value) || 0;
    const val2 = parseFloat(document.getElementById(prefix + '2').value) || 0;
    const val3 = parseFloat(document.getElementById(prefix + '3').value) || 0;
    const limit_i = parseInt(document.getElementById(prefix + 'LimI').value) || 30000;
    const blend_el = document.getElementById('sliderBlend');
    const blend_pct = blend_el ? (parseInt(blend_el.value) || 0) : 0;

    let payload = { mode: mode, p: 0, i: 0, d: 0, gain_output: 1.0, limit_i: limit_i, blend: blend_pct };

    if (type === 'leadlag') {
        payload.p = val1;
        payload.i = (val2 - val3) * val1 * CONTROL_PERIOD;
        payload.d = 0.0;
        payload.gain_output = val3 * CONTROL_PERIOD;
    } else if (type === 'c_pid') {
        payload.p = val1;
        payload.i = val2 * CONTROL_PERIOD;
        payload.d = val3 * CONTROL_FREQ;
    } else if (type === 't_pid') {
        payload.p = val1;
        payload.i = val2 !== 0 ? (val1 * CONTROL_PERIOD / val2) : 0;
        payload.d = val1 * val3 * CONTROL_FREQ;
    } else {
        payload.p = val1;
        payload.i = val2;
        payload.d = val3;
    }

    currentActiveParams[mode] = { c0: payload.p, c1: payload.i, c2: payload.d, d1: 1.0 };
    await fetch('/set_pid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
}

async function setADRC(mode, prefix) {
    const wc = parseFloat(document.getElementById(prefix + '_adrc_wc').value) || 0;
    const b0 = parseFloat(document.getElementById(prefix + '_adrc_b0').value) || 0;
    const ramp_time = parseFloat(document.getElementById(prefix + '_adrc_ramp').value) || 0;
    const payload = { mode: mode, wc: wc, b0: b0, ramp_time: ramp_time };
    await fetch('/set_adrc', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
}

async function resetPosition() {
    await fetch('/reset_position', { method: 'POST' });
}

async function resetAdrc() {
    await fetch('/reset_adrc', { method: 'POST' });
}

async function setSysID() {
    const type = parseInt(document.getElementById('sysidType').value);
    const amp = parseInt(document.getElementById('numSysIdAmp').value);
    const freqHz = parseFloat(document.getElementById('numSysIdFreq').value);
    const freq = Math.round(freqHz * 100);
    const offset = parseInt(document.getElementById('numSysIdOffset').value);
    const isSine = (type === 2);

    let payload = { waveform_type: type, amplitude: amp, frequency: freq, offset: offset, sine_enable: isSine };
    await fetch('/set_sysid', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
}

async function connectPort() {
    const port = document.getElementById('portSelect').value;
    const statusSpan = document.getElementById('connStatus');
    const connBtn = document.getElementById('connBtn');
    statusSpan.innerText = "Connecting..."; connBtn.disabled = true;
    const res = await fetch('/connect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ port: port, device_id: 48 }) });
    const data = await res.json();
    statusSpan.innerText = data.message; statusSpan.style.color = data.status === 'connected' ? "green" : "red";
    if (data.status === 'connected') { 
        isConnected = true; 
        connBtn.innerText = "Disconnect"; 
        connBtn.classList.add('danger-btn'); 
        startTelemetry(); 
    }
    connBtn.disabled = false;
}

async function disconnectPort() {
    const res = await fetch('/disconnect', { method: 'POST' }); const data = await res.json();
    document.getElementById('connStatus').innerText = data.message; document.getElementById('connStatus').style.color = "gray";
    if (ws) { ws.close(); ws = null; }
    isConnected = false; 
    const connBtn = document.getElementById('connBtn'); 
    connBtn.innerText = "Connect"; 
    connBtn.classList.remove('danger-btn');
}

async function toggleConnect() { 
    if (isConnected) { await disconnectPort(); } else { await connectPort(); } 
}

async function setTarget(mode, inputId, minLim, maxLim) {
    let val = parseInt(document.getElementById(inputId).value, 10) || 0;
    const payload = { mode: mode, value: val, min_limit: minLim, max_limit: maxLim };
    await fetch('/set_target', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
}

async function setPWM() {
    let val = parseInt(document.getElementById('valPwm').value, 10) || 0;
    const payload = { value: val };
    await fetch('/set_pwm', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
}

async function startMotor() { await fetch('/start', { method: 'POST' }); }
async function coastStop() { await fetch('/stop', { method: 'POST' }); }

async function startSafeTransfer(mode, prefix) {
    const type = document.getElementById(prefix + 'CtrlType').value;
    const val1 = parseFloat(document.getElementById(prefix + '1').value) || 0;
    const val2 = parseFloat(document.getElementById(prefix + '2').value) || 0;
    const val3 = parseFloat(document.getElementById(prefix + '3').value) || 0;
    const limit_i = parseInt(document.getElementById(prefix + 'LimI').value) || 30000;

    let c_new0 = val1;
    let c_new1 = 0, c_new2 = 0, gain_B_new = 0;

    if (type === 'leadlag') {
        c_new1 = (val2 - val3) * val1 * CONTROL_PERIOD;
        gain_B_new = val3 * CONTROL_PERIOD;
    } else {
        c_new1 = val2;
        c_new2 = val3;
    }

    let d_new1 = 1.0 / (gain_B_new + 1.0);

    const payload = {
        mode: mode,
        c_pid0: currentActiveParams[mode].c0,
        c_pid1: currentActiveParams[mode].c1,
        c_pid2: currentActiveParams[mode].c2,
        c_new0: c_new0,
        c_new1: c_new1,
        c_new2: c_new2,
        d_new1: d_new1,
        limit_i: limit_i
    };

    document.getElementById('fadeProgress').value = 0;

    try {
        const res = await fetch('/safe_transfer', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        const data = await res.json();

        if (data.status === "aborted") {
            alert("⚠️ " + data.reason);
            document.getElementById('fadeProgress').value = 0;
        } else {
            currentActiveParams[mode] = { c0: c_new0, c1: c_new1, c2: c_new2, d1: d_new1 };
            document.getElementById('fadeProgress').value = 100;
        }
    } catch (e) {
        console.error("Transfer Error:", e);
    }
}

async function sendBlendData() {
    if (document.getElementById('tab-vel').classList.contains('active')) {
        setPID('velocity', 'vel');
    } else if (document.getElementById('tab-pos').classList.contains('active')) {
        setPID('position', 'pos');
    } else {
        setPID('current', 'cur');
    }
}
