const btn = document.querySelector('.btn-voice');
const waves = document.querySelector('.voice-waves');

btn.addEventListener('click', () => {
    const isRunning = btn.classList.toggle('paused');
    btn.textContent = isRunning ? 'PAUSE' : 'START';
    waves.classList.toggle('active', isRunning);

    fetch(isRunning ? '/voice/start' : '/voice/stop', { method: 'POST' });
});

function updateVoiceStatus() {
    fetch('/api/voice_status')
        .then(r => r.text())
        .then(status => {
            document.getElementById('voice-result').textContent = status;
        })
        .catch(error => console.error('Error fetching voice status:', error));
}

setInterval(updateVoiceStatus, 500);
