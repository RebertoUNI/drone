#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone Telemetry Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      background: #222;
      color: #fff;
      padding: 10px;
    }

    .card {
      background: #333;
      padding: 20px;
      margin: 10px auto;
      max-width: 380px;
      border-radius: 10px;
    }

    .btn {
      padding: 15px 25px;
      font-size: 16px;
      margin: 5px;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      color: white;
      font-weight: bold;
    }

    .btn-start {
      background: #2ecc71;
    }

    .btn-stop {
      background: #e74c3c;
      width: 90%;
    }

    .btn-pid {
      background: #3498db;
      width: 90%;
      margin-top: 15px;
    }

    input[type=range] {
      width: 80%;
      height: 30px;
    }

    input[type=number] {
      width: 55px;
      padding: 5px;
      text-align: center;
      margin: 4px 0;
      border: 1px solid #555;
      border-radius: 4px;
    }

    .telemetry {
      display: flex;
      justify-content: space-around;
      font-size: 1.2em;
      background: #444;
      padding: 10px;
      border-radius: 5px;
    }

    .pid-container {
      display: flex;
      justify-content: space-around;
      gap: 10px;
    }

    .pid-column {
      flex: 1;
      background: #2a2a2a;
      padding: 10px;
      border-radius: 8px;
    }

    .pid-column h4 {
      margin: 0 0 10px 0;
    }

    .pid-row {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      font-weight: bold;
    }
  </style>
</head>

<body>
  <h2>Quadcopter Control Interface</h2>

  <div class="card">
    <h3>Flight Dynamics</h3>
    <div class="telemetry">
      <div>Pitch: <br><strong id="pitchVal" style="color: #f39c12;">0.00</strong>&deg;</div>
      <div>Roll: <br><strong id="rollVal" style="color: #3498db;">0.00</strong>&deg;</div>
    </div>
    <div class="telemetry" style="margin-top: 10px; font-size: 0.9em; background: #2a2a2a;">
      <div>Gyro X offset: <br><strong id="gyroXCal" style="color: #1abc9c;">0.00</strong></div>
      <div>Gyro Y offset: <br><strong id="gyroYCal" style="color: #9b59b6;">0.00</strong></div>
    </div>
  </div>

  <div class="card">
    <h3>Motori</h3>
    <button class="btn btn-start" onclick="sendCmd('t')">START TEST</button>
    <button class="btn btn-stop" onclick="sendCmd('s')">EMERGENCY STOP</button>
  </div>

  <div class="card">
    <h3>Throttle: <span id="thVal">0</span></h3>
    <input type="range" min="0" max="255" value="0" oninput="sendThr(this.value)">
  </div>

  <div class="card">
    <h3>PID Tuning</h3>
    <div class="pid-container">
      <!-- Pitch PID -->
      <div class="pid-column">
        <h4 style="color: #f39c12;">Pitch</h4>
        <div class="pid-row">P (<span id="currentPitchP">1.00</span>): <input type="number" id="p_pitch" step="0.01" value="1.00"></div>
        <div class="pid-row">I (<span id="currentPitchI">0.00</span>): <input type="number" id="i_pitch" step="0.01" value="0.00"></div>
        <div class="pid-row">D (<span id="currentPitchD">0.00</span>): <input type="number" id="d_pitch" step="0.01" value="0.00"></div>
      </div>

      <!-- Roll PID -->
      <div class="pid-column">
        <h4 style="color: #3498db;">Roll</h4>
        <div class="pid-row">P (<span id="currentRollP">1.00</span>): <input type="number" id="p_roll" step="0.01" value="1.00"></div>
        <div class="pid-row">I (<span id="currentRollI">0.00</span>): <input type="number" id="i_roll" step="0.01" value="0.00"></div>
        <div class="pid-row">D (<span id="currentRollD">0.00</span>): <input type="number" id="d_roll" step="0.01" value="0.00"></div>
      </div>
    </div>

    <button class="btn btn-pid" onclick="sendPid()">Invia Costanti PID</button>
  </div>

  <script>
    function sendCmd(action) { fetch('/cmd?action=' + action); }
    function sendThr(val) { document.getElementById('thVal').innerText = val; fetch('/throttle?val=' + val); }
    function updateCurrentPid(id, value) {
      if (value !== undefined) {
        const numericValue = Number(value);
        document.getElementById(id).innerText = Number.isFinite(numericValue) ? numericValue.toFixed(2) : value;
      }
    }

    function sendPid() {
      const pp = document.getElementById('p_pitch').value;
      const ip = document.getElementById('i_pitch').value;
      const dp = document.getElementById('d_pitch').value;

      const pr = document.getElementById('p_roll').value;
      const ir = document.getElementById('i_roll').value;
      const dr = document.getElementById('d_roll').value;

      fetch(`/pid?pitch_p=${pp}&pitch_i=${ip}&pitch_d=${dp}&roll_p=${pr}&roll_i=${ir}&roll_d=${dr}`);
    }

    // Polling asincrono per l'aggiornamento degli angoli e calibrazione in background a 5Hz
    setInterval(() => {
      fetch('/telemetry')
        .then(response => response.json())
        .then(data => {
          document.getElementById('pitchVal').innerText = data.pitch;
          document.getElementById('rollVal').innerText = data.roll;
          if (data.gyro_x_cal !== undefined) {
            document.getElementById('gyroXCal').innerText = isNaN(Number(data.gyro_x_cal)) ? data.gyro_x_cal : Number(data.gyro_x_cal).toFixed(2);
          }
          if (data.gyro_y_cal !== undefined) {
            document.getElementById('gyroYCal').innerText = isNaN(Number(data.gyro_y_cal)) ? data.gyro_y_cal : Number(data.gyro_y_cal).toFixed(2);
          }
          updateCurrentPid('currentPitchP', data.pid_pitch_p);
          updateCurrentPid('currentPitchI', data.pid_pitch_i);
          updateCurrentPid('currentPitchD', data.pid_pitch_d);
          updateCurrentPid('currentRollP', data.pid_roll_p);
          updateCurrentPid('currentRollI', data.pid_roll_i);
          updateCurrentPid('currentRollD', data.pid_roll_d);
        })
        .catch(err => console.error(err));
    }, 200);
  </script>
</body>

</html>
)rawliteral";

#endif // INDEX_HTML_H
