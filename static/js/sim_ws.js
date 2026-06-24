function startTelemetry() {
    if (ws) ws.close();
    ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
    ws.onmessage = (event) => {
        const pts = JSON.parse(event.data);
        for (let i = 0; i < pts.length; i++) {
            if (pts[i].type === 'transfer_progress') {
                document.getElementById('fadeProgress').value = pts[i].progress;
            } else if (pts[i].type === 'tuning_update') {
                if (document.getElementById('vel_adrc_wc')) document.getElementById('vel_adrc_wc').value = pts[i].wc;
                if (document.getElementById('slider_vel_adrc_wc')) document.getElementById('slider_vel_adrc_wc').value = pts[i].wc;
                if (document.getElementById('vel_adrc_b0')) document.getElementById('vel_adrc_b0').value = pts[i].b0;
                if (document.getElementById('slider_vel_adrc_b0')) document.getElementById('slider_vel_adrc_b0').value = pts[i].b0;
                if (document.getElementById('blendDisplay')) document.getElementById('blendDisplay').value = pts[i].blend;
                if (document.getElementById('sliderBlend')) document.getElementById('sliderBlend').value = pts[i].blend;
            } else {
                let pt = pts[i];
                if (lastTime !== null) {
                    const dt = pt.time - lastTime;
                    if (dt > 0.0001) {
                        const rawAcc = (pt.velocity - lastVel) / dt;
                        smoothedAcc = (accAlpha * rawAcc) + ((1 - accAlpha) * smoothedAcc);
                    }
                }
                pt.acceleration = smoothedAcc;
                lastTime = pt.time;
                lastVel = pt.velocity;

                // Sync Target Velocity to UI if user is not actively typing
                const numVel = document.getElementById('numVel');
                const valVel = document.getElementById('valVel');
                if (numVel && valVel && document.activeElement !== numVel && document.activeElement !== valVel) {
                    numVel.value = pt.target_velocity;
                    valVel.value = pt.target_velocity;
                }

                const sliderBlend = document.getElementById('sliderBlend');
                pt.fading = sliderBlend ? sliderBlend.value : 0;

                const wc = document.getElementById('vel_adrc_wc');
                const b0 = document.getElementById('vel_adrc_b0');
                const ramp = document.getElementById('vel_adrc_ramp');
                pt.adrc_wc = wc ? wc.value : 0;
                pt.adrc_b0 = b0 ? b0.value : 0;
                pt.adrc_ramp = ramp ? ramp.value : 0;

                exportDataBuffer.push(pt);

                while (exportDataBuffer.length > 0 && (pt.time - exportDataBuffer[0].time > 60.0)) {
                    exportDataBuffer.shift();
                }

                incomingBuffer.push(pt);
            }
        }
        chartDirty = true;
    };
}

function renderLoop() {
    if (chartDirty && incomingBuffer.length > 0) {
        chartDirty = false;
        const pts = incomingBuffer; incomingBuffer = [];
        const visualDecimation = 10;

        for (let i = 0; i < pts.length; i++) {
            renderDecimationCounter++;
            if (renderDecimationCounter % visualDecimation !== 0) continue;

            const pt = pts[i]; const label = pt.time.toFixed(2);
            velChart.data.labels.push(label); velChart.data.datasets[0].data.push(pt.velocity);
            accChart.data.labels.push(label); accChart.data.datasets[0].data.push(pt.acceleration);
            curChart.data.labels.push(label); curChart.data.datasets[0].data.push(pt.current);

            z1Chart.data.labels.push(label); z1Chart.data.datasets[0].data.push(pt.z1);
            z2Chart.data.labels.push(label); z2Chart.data.datasets[0].data.push(pt.z2);
            z3Chart.data.labels.push(label); z3Chart.data.datasets[0].data.push(pt.z3);
        }

        if (velChart.data.labels.length > 0) {
            const latestTime = parseFloat(velChart.data.labels[velChart.data.labels.length - 1]);
            const threshold = latestTime - chartWindowSec;

            let excess = 0;
            for (let i = 0; i < velChart.data.labels.length; i++) {
                if (parseFloat(velChart.data.labels[i]) < threshold) {
                    excess++;
                } else {
                    break;
                }
            }

            if (excess > 0) {
                velChart.data.labels.splice(0, excess); velChart.data.datasets[0].data.splice(0, excess);
                accChart.data.labels.splice(0, excess); accChart.data.datasets[0].data.splice(0, excess);
                curChart.data.labels.splice(0, excess); curChart.data.datasets[0].data.splice(0, excess);

                z1Chart.data.labels.splice(0, excess); z1Chart.data.datasets[0].data.splice(0, excess);
                z2Chart.data.labels.splice(0, excess); z2Chart.data.datasets[0].data.splice(0, excess);
                z3Chart.data.labels.splice(0, excess); z3Chart.data.datasets[0].data.splice(0, excess);
            }
        }

        velChart.update('none'); accChart.update('none'); curChart.update('none');
        z1Chart.update('none'); z2Chart.update('none'); z3Chart.update('none');
    }
    requestAnimationFrame(renderLoop);
}
