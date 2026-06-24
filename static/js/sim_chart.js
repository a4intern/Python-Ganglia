Chart.defaults.color = '#64748b';
Chart.defaults.borderColor = 'rgba(0,0,0,0.05)';

const createChart = (ctxId, label, color) => {
    return new Chart(document.getElementById(ctxId), {
        type: 'line',
        data: { labels: [], datasets: [{ label: label, borderColor: color, data: [], fill: false, pointRadius: 0, borderWidth: 2, tension: 0.1 }] },
        options: { animation: false, responsive: true, maintainAspectRatio: false, scales: { x: { display: true }, y: { display: true } } }
    });
};

const velChart = createChart('velChart', 'Velocity (rpm)', '#0284c7');
const accChart = createChart('accChart', 'Acceleration (rpm/s)', '#d97706');
const curChart = createChart('curChart', 'Current (mA)', '#e11d48');
const z1Chart = createChart('z1Chart', 'z1 (Position)', '#10b981');
const z2Chart = createChart('z2Chart', 'z2 (Velocity)', '#8b5cf6');
const z3Chart = createChart('z3Chart', 'z3 (Accel Disturbance)', '#ec4899');

function updateChartWindow() {
    chartWindowSec = parseInt(document.getElementById('chartWindowSec').value) || 30;

    velChart.data.labels = []; velChart.data.datasets[0].data = [];
    accChart.data.labels = []; accChart.data.datasets[0].data = [];
    curChart.data.labels = []; curChart.data.datasets[0].data = [];
    z1Chart.data.labels = []; z1Chart.data.datasets[0].data = [];
    z2Chart.data.labels = []; z2Chart.data.datasets[0].data = [];
    z3Chart.data.labels = []; z3Chart.data.datasets[0].data = [];

    if (exportDataBuffer && exportDataBuffer.length > 0) {
        const latestTime = exportDataBuffer[exportDataBuffer.length - 1].time;
        const targetTime = latestTime - chartWindowSec;

        let startIndex = 0;
        for (let i = exportDataBuffer.length - 1; i >= 0; i--) {
            if (exportDataBuffer[i].time < targetTime) {
                startIndex = i + 1;
                break;
            }
        }

        for (let i = startIndex; i < exportDataBuffer.length; i++) {
            if (i % 5 !== 0) continue; 
            const pt = exportDataBuffer[i];
            const label = pt.time.toFixed(2);
            velChart.data.labels.push(label); velChart.data.datasets[0].data.push(pt.velocity);
            accChart.data.labels.push(label); accChart.data.datasets[0].data.push(pt.acceleration);
            curChart.data.labels.push(label); curChart.data.datasets[0].data.push(pt.current);
            z1Chart.data.labels.push(label); z1Chart.data.datasets[0].data.push(pt.z1);
            z2Chart.data.labels.push(label); z2Chart.data.datasets[0].data.push(pt.z2);
            z3Chart.data.labels.push(label); z3Chart.data.datasets[0].data.push(pt.z3);
        }
    }

    velChart.update('none'); accChart.update('none'); curChart.update('none');
    z1Chart.update('none'); z2Chart.update('none'); z3Chart.update('none');
}
