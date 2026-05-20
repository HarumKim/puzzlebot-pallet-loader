const btn = document.querySelector('.btn-voice');
const waves = document.querySelector('.voice-waves');

btn.addEventListener('click', () => {
    const isRunning = btn.classList.toggle('paused');
    btn.textContent = isRunning ? 'PAUSE' : 'START';
    waves.classList.toggle('active', isRunning);
});

function updateResult() {
  fetch('/api/result')
    .then(response => response.text())
    .then(result => {
      document.getElementById('result').textContent = result;
    })
    .catch(error => console.error('Error fetching data:', error));
}

setInterval(updateResult, 500);

