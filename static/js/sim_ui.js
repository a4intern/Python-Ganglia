function syncInput(sourceId, targetId) { 
    document.getElementById(targetId).value = document.getElementById(sourceId).value; 
}

async function switchTab(tabPrefix, opModeInt) {
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab-buttons button').forEach(el => el.classList.remove('active'));
    document.getElementById('tab-' + tabPrefix).classList.add('active');
    document.getElementById('btn-' + tabPrefix).classList.add('active');
    await fetch('/set_op_mode', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode: opModeInt }) });
}

function updateLabels(prefix) {
    const type = document.getElementById(prefix + 'CtrlType').value;
    if (type === 'leadlag') {
        document.getElementById(prefix + 'L1').innerText = 'K';
        document.getElementById(prefix + 'L2').innerText = 'a';
        document.getElementById(prefix + 'L3').innerText = 'b';
    } else if (type === 't_pid') {
        document.getElementById(prefix + 'L1').innerText = 'Kc';
        document.getElementById(prefix + 'L2').innerText = 'Ti';
        document.getElementById(prefix + 'L3').innerText = 'Td';
    } else {
        document.getElementById(prefix + 'L1').innerText = 'P';
        document.getElementById(prefix + 'L2').innerText = 'I';
        document.getElementById(prefix + 'L3').innerText = 'D';
    }
}

function exportCSV() {
    if (exportDataBuffer.length === 0) {
        alert("No data to export!");
        return;
    }
    let csvContent = "data:text/csv;charset=utf-8,UnixEpoch,Time(s),Velocity(RPM),Acceleration(RPM/s),Current(mA),z1,z2,z3,FadingValue(%),ADRC_Wc,ADRC_b0,ADRC_Ramp\n";
    for (let i = 0; i < exportDataBuffer.length; i++) {
        const pt = exportDataBuffer[i];
        csvContent += `${pt.unix_time || ''},${pt.time},${pt.velocity},${pt.acceleration},${pt.current},${pt.z1},${pt.z2},${pt.z3},${pt.fading || 0},${pt.adrc_wc || 0},${pt.adrc_b0 || 0},${pt.adrc_ramp || 0}\n`;
    }

    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", `telemetry_export_${new Date().getTime()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

async function refreshPorts() {
    const res = await fetch('/ports'); const data = await res.json();
    const select = document.getElementById('portSelect'); select.innerHTML = '';
    data.ports.forEach(p => select.add(new Option(p, p)));
}

document.addEventListener('DOMContentLoaded', () => {
    const aiHeader = document.querySelector('.ai-chat-header'); const aiPanel = document.getElementById('aiChatPanel');
    aiHeader.addEventListener('mousedown', (e) => { 
        if (e.target.id === 'aiChatToggleIcon') return; 
        isDraggingAiChat = true; aiChatDragDist = 0; aiChatStartX = e.clientX; 
        aiChatStartLeft = parseInt(window.getComputedStyle(aiPanel).left, 10) || 20; 
        aiHeader.style.cursor = 'grabbing'; 
    });
    document.addEventListener('mousemove', (e) => { 
        if (!isDraggingAiChat) return; 
        const dx = e.clientX - aiChatStartX; aiChatDragDist += Math.abs(e.movementX); 
        let newLeft = aiChatStartLeft + dx; const maxLeft = window.innerWidth - aiPanel.offsetWidth; 
        if (newLeft < 0) newLeft = 0; if (newLeft > maxLeft) newLeft = maxLeft; 
        aiPanel.style.left = newLeft + 'px'; aiPanel.style.right = 'auto'; 
    });
    document.addEventListener('mouseup', () => { 
        if (isDraggingAiChat) { isDraggingAiChat = false; aiHeader.style.cursor = 'pointer'; } 
    });
});

function toggleChat() { 
    if (aiChatDragDist > 3) return; 
    const panel = document.getElementById('aiChatPanel'); 
    const icon = document.getElementById('aiChatToggleIcon'); 
    panel.classList.toggle('closed'); 
    icon.innerText = panel.classList.contains('closed') ? '▲' : '▼'; 
}

function handleChatKeyPress(e) { if (e.key === 'Enter') { sendChatMessage(); } }

async function sendChatMessage() {
    const input = document.getElementById('aiChatInput'); const msg = input.value.trim(); if (!msg) return; input.value = '';
    const body = document.getElementById('aiChatBody');
    const userDiv = document.createElement('div'); userDiv.className = 'chat-message user-message'; userDiv.innerText = msg; body.appendChild(userDiv); body.scrollTop = body.scrollHeight;

    let activeTab = 'Unknown';
    if (document.getElementById('tab-pos').classList.contains('active')) activeTab = 'Position';
    if (document.getElementById('tab-vel').classList.contains('active')) activeTab = 'Velocity';

    const context = { activeTab: activeTab, target: document.getElementById('numPos').value };

    const loadingDiv = document.createElement('div'); loadingDiv.className = 'chat-message ai-message'; loadingDiv.innerText = 'Thinking...'; body.appendChild(loadingDiv); body.scrollTop = body.scrollHeight;
    try { 
        const res = await fetch('/chat', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ message: msg, context: context }) }); 
        const data = await res.json(); let cleanText = data.response.replace(/\*\*/g, ''); loadingDiv.innerText = cleanText; 
    } catch (e) { 
        loadingDiv.innerText = "Error connecting to AI Tutor."; 
    }
    body.scrollTop = body.scrollHeight;
}

document.addEventListener('DOMContentLoaded', () => {
    const resizer = document.getElementById('dragResizer'); 
    const leftPanel = document.getElementById('controlsLeft'); 
    let isResizing = false;
    resizer.addEventListener('mousedown', function (e) { e.preventDefault(); isResizing = true; document.body.style.cursor = 'col-resize'; });
    document.addEventListener('mousemove', function (e) { 
        if (!isResizing) return; 
        const newWidth = e.clientX - leftPanel.getBoundingClientRect().left; 
        if (newWidth > 250 && newWidth < window.innerWidth - 200) { 
            leftPanel.style.width = newWidth + 'px'; leftPanel.style.maxWidth = 'none'; 
        } 
    });
    document.addEventListener('mouseup', function (e) { if (isResizing) { isResizing = false; document.body.style.cursor = 'default'; } });

    const graphResizer = document.getElementById('graphDragResizer');
    const graphLeftPanel = document.getElementById('graphsLeftColumn');
    let isGraphResizing = false;

    graphResizer.addEventListener('mousedown', function (e) {
        e.preventDefault();
        isGraphResizing = true;
        document.body.style.cursor = 'col-resize';
    });

    document.addEventListener('mousemove', function (e) {
        if (!isGraphResizing) return;
        const graphsWrapper = document.getElementById('graphsWrapper');
        const newWidth = e.clientX - graphsWrapper.getBoundingClientRect().left;
        if (newWidth > 150 && newWidth < graphsWrapper.clientWidth - 150) {
            graphLeftPanel.style.flex = 'none';
            graphLeftPanel.style.width = newWidth + 'px';
        }
    });

    document.addEventListener('mouseup', function (e) {
        if (isGraphResizing) {
            isGraphResizing = false;
            document.body.style.cursor = 'default';
        }
    });
});
