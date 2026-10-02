document.addEventListener('DOMContentLoaded', async () => {
    requestAnimationFrame(renderLoop);
    await refreshPorts(); 
    switchTab('vel', -2);
    
    // Auto-connect to Virtual Motor
    const portSelect = document.getElementById('portSelect');
    if (portSelect && portSelect.options.length > 0) {
        portSelect.value = "Virtual Motor";
        await connectPort();
    }
});
