#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Drone Telemetry Control</title>
    <style>
        :root {
            --accent-blue: #38bdf8;
            --accent-amber: #fbbf24;
            --accent-green: #22c55e;
            --accent-red: #ef4444;
            --accent-purple: #a78bfa;
            --text-dim: #94a3b8;
            --text-muted: #64748b;
        }

        * {
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
            text-align: center;
            background: #0a0e14;
            background-image: radial-gradient(ellipse at top, #111827 0%, #0a0e14 70%);
            background-attachment: fixed;
            color: #f1f5f9;
            min-height: 100vh;
            padding: 12px;
            -webkit-font-smoothing: antialiased;
            -webkit-tap-highlight-color: transparent;
        }

        .card {
            background: rgba(21, 27, 38, 0.8);
            padding: 16px;
            margin: 12px auto;
            max-width: 480px;
            border-radius: 14px;
            border: 1px solid rgba(38, 51, 69, 0.6);
            box-shadow: 0 4px 24px rgba(0, 0, 0, 0.4), 0 0 0 1px rgba(255, 255, 255, 0.02) inset;
            backdrop-filter: blur(12px);
            -webkit-backdrop-filter: blur(12px);
            position: relative;
            overflow: hidden;
        }

        .card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 1px;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.08), transparent);
        }

        .card h3 {
            font-size: 0.7rem;
            text-transform: uppercase;
            letter-spacing: 0.1em;
            color: var(--text-dim);
            margin-bottom: 14px;
            font-weight: 700;
            display: flex;
            align-items: center;
            gap: 8px;
            justify-content: flex-start;
        }

        .card h3::before {
            content: '';
            width: 3px;
            height: 12px;
            border-radius: 2px;
            background: var(--accent-blue);
            box-shadow: 0 0 8px var(--accent-blue);
        }

        .btn {
            padding: 12px 16px;
            font-size: 0.85rem;
            margin: 5px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            color: white;
            font-weight: 700;
            letter-spacing: 0.02em;
            min-height: 44px;
            transition: opacity 0.2s, transform 0.1s, box-shadow 0.2s, background 0.3s;
            font-family: inherit;
        }

        .btn:active {
            transform: scale(0.97);
        }

        /* Pulsante START/STOP toggle */
        .btn-toggle-motor {
            width: 100%;
            padding: 16px;
            font-size: 1rem;
            font-weight: 800;
            letter-spacing: 0.08em;
            text-transform: uppercase;
            min-height: 60px;
            border-radius: 8px;
            margin: 0;
            transition: all 0.25s ease;
        }

        /* Stato OFF (motori spenti) → pulsante verde "START" */
        .btn-toggle-motor.off {
            background: linear-gradient(135deg, #22c55e, #16a34a);
            color: white;
            box-shadow: 0 4px 16px rgba(34, 197, 94, 0.4);
        }

        .btn-toggle-motor.off:hover {
            box-shadow: 0 6px 24px rgba(34, 197, 94, 0.6);
            transform: translateY(-1px);
        }

        /* Stato ON (motori accesi) → pulsante rosso "STOP" pulsante */
        .btn-toggle-motor.on {
            background: linear-gradient(135deg, #ef4444, #dc2626);
            color: white;
            box-shadow: 0 4px 16px rgba(239, 68, 68, 0.4);
            animation: pulse-danger 2s ease-in-out infinite;
        }

        .btn-toggle-motor.on:hover {
            box-shadow: 0 6px 24px rgba(239, 68, 68, 0.6);
        }

        @keyframes pulse-danger {

            0%,
            100% {
                box-shadow: 0 4px 16px rgba(239, 68, 68, 0.4);
            }

            50% {
                box-shadow: 0 4px 24px rgba(239, 68, 68, 0.7);
            }
        }

        .btn-emergency {
            background: linear-gradient(135deg, #ef4444, #b91c1c);
            color: white;
            width: 100%;
            margin: 12px 0 0 0;
            box-shadow: 0 4px 12px rgba(239, 68, 68, 0.3);
            border: 1px solid rgba(255, 100, 100, 0.3);
        }

        .btn-emergency:hover {
            box-shadow: 0 6px 20px rgba(239, 68, 68, 0.5);
        }

        .btn-pid {
            background: linear-gradient(135deg, var(--accent-blue), #0ea5e9);
            color: #001;
            width: 100%;
            margin-top: 16px;
            box-shadow: 0 4px 12px rgba(56, 189, 248, 0.3);
        }

        .btn-pid:hover {
            box-shadow: 0 6px 20px rgba(56, 189, 248, 0.5);
        }

        /* ---------- THROTTLE ---------- */
        .thr-display {
            text-align: center;
            margin: 12px 0 8px;
            font-size: 0.85rem;
            color: var(--text-dim);
            font-weight: 600;
            letter-spacing: 0.05em;
        }

        .thr-display strong {
            font-family: "JetBrains Mono", ui-monospace, monospace;
            font-size: 1.6rem;
            color: #fff;
            margin-left: 4px;
            font-variant-numeric: tabular-nums;
        }

        .thr-display strong.disabled {
            color: var(--text-muted);
        }

        input[type=range] {
            width: 100%;
            height: 36px;
            background: transparent;
            -webkit-appearance: none;
            appearance: none;
            cursor: pointer;
            touch-action: pan-y;
        }

        input[type=range]::-webkit-slider-runnable-track {
            height: 8px;
            background: rgba(15, 20, 28, 0.9);
            border-radius: 4px;
            border: 1px solid rgba(30, 41, 59, 0.8);
            pointer-events: none;
        }

        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 28px;
            height: 28px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--accent-blue), #0ea5e9);
            margin-top: -11px;
            box-shadow: 0 0 12px rgba(56, 189, 248, 0.6), 0 2px 6px rgba(0, 0, 0, 0.5);
            border: 2px solid #fff;
            pointer-events: auto;
            cursor: grab;
        }

        input[type=range]::-webkit-slider-thumb:active {
            cursor: grabbing;
        }

        input[type=range]::-moz-range-track {
            height: 8px;
            background: rgba(15, 20, 28, 0.9);
            border-radius: 4px;
            border: 1px solid rgba(30, 41, 59, 0.8);
            pointer-events: none;
        }

        input[type=range]::-moz-range-thumb {
            width: 28px;
            height: 28px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--accent-blue), #0ea5e9);
            box-shadow: 0 0 12px rgba(56, 189, 248, 0.6);
            border: 2px solid #fff;
            cursor: grab;
            pointer-events: auto;
        }

        input[type=range]:disabled {
            opacity: 0.3;
            cursor: not-allowed;
        }

        input[type=range]:disabled::-webkit-slider-thumb {
            background: #475569;
            box-shadow: none;
        }

        /* Telemetria */
        .telemetry {
            display: flex;
            justify-content: space-around;
            font-size: 1.1em;
            background: rgba(15, 20, 28, 0.7);
            border: 1px solid rgba(30, 41, 59, 0.8);
            padding: 12px 10px;
            border-radius: 8px;
            font-weight: 600;
        }

        .telemetry strong {
            font-family: "JetBrains Mono", ui-monospace, Consolas, monospace;
            font-size: 1.3rem;
            font-variant-numeric: tabular-nums;
        }

        /* ============================================================
       PID TUNING
       ============================================================ */
        .pid-table {
            width: 100%;
            border-collapse: separate;
            border-spacing: 0 8px;
            margin: 0;
        }

        .pid-table thead th {
            font-size: 0.65rem;
            color: var(--text-muted);
            padding: 4px 8px;
            text-align: left;
            text-transform: uppercase;
            letter-spacing: 0.08em;
            font-weight: 700;
        }

        .pid-table tbody tr {
            background: rgba(15, 20, 28, 0.6);
        }

        .pid-table td {
            padding: 10px 8px;
            vertical-align: middle;
        }

        .pid-table tbody tr td:first-child {
            border-top-left-radius: 8px;
            border-bottom-left-radius: 8px;
            padding-left: 14px;
            font-weight: 700;
            font-size: 0.85rem;
        }

        .pid-table tbody tr td:last-child {
            border-top-right-radius: 8px;
            border-bottom-right-radius: 8px;
            padding-right: 14px;
        }

        .pid-input-group {
            display: flex;
            align-items: center;
            gap: 8px;
            flex-wrap: wrap;
        }

        .pid-table input[type=number] {
            width: 80px;
            background: rgba(11, 15, 23, 0.9);
            border: 1px solid rgba(51, 65, 85, 0.8);
            color: #fff;
            padding: 10px;
            border-radius: 6px;
            font-family: ui-monospace, monospace;
            text-align: right;
            font-size: 0.85rem;
            transition: border-color 0.2s, box-shadow 0.2s;
            -moz-appearance: textfield;
            min-height: 40px;
            margin: 0;
        }

        .pid-table input[type=number]::-webkit-outer-spin-button,
        .pid-table input[type=number]::-webkit-inner-spin-button {
            -webkit-appearance: none;
            margin: 0;
        }

        .pid-table input[type=number]:focus {
            border-color: var(--accent-blue);
            outline: none;
            box-shadow: 0 0 0 3px rgba(56, 189, 248, 0.15);
        }

        .live-pid {
            font-size: 0.65rem;
            color: var(--text-muted);
            font-family: ui-monospace, monospace;
            white-space: nowrap;
        }

        @media (max-width: 520px) {
            .card {
                padding: 12px 8px;
            }

            .pid-table {
                border-spacing: 0 6px;
            }

            .pid-table thead th {
                font-size: 0;
                text-align: center;
                padding: 2px 2px 8px;
                letter-spacing: 0;
            }

            .pid-table thead th:first-child {
                font-size: 0.7rem;
                text-align: left;
                padding-left: 10px;
                width: 50px;
            }

            .pid-table thead th:nth-child(2)::after {
                content: "P";
                font-size: 0.75rem;
                font-weight: 800;
                color: var(--accent-green);
            }

            .pid-table thead th:nth-child(3)::after {
                content: "I";
                font-size: 0.75rem;
                font-weight: 800;
                color: var(--accent-amber);
            }

            .pid-table thead th:nth-child(4)::after {
                content: "D";
                font-size: 0.75rem;
                font-weight: 800;
                color: var(--accent-purple);
            }

            .pid-table td {
                padding: 8px 2px;
            }

            .pid-table tbody tr td:first-child {
                padding-left: 10px;
                padding-right: 4px;
                font-size: 0.75rem;
                width: 50px;
            }

            .pid-table tbody tr td:last-child {
                padding-right: 8px;
            }

            .pid-input-group {
                flex-direction: column;
                justify-content: center;
                align-items: center;
                gap: 4px;
            }

            .pid-table input[type=number] {
                width: 100%;
                max-width: 72px;
                min-width: 56px;
                text-align: center;
                padding: 8px 2px;
                font-size: 0.8rem;
                min-height: 36px;
            }

            .live-pid {
                width: auto;
                text-align: center;
                font-size: 0.58rem;
                letter-spacing: -0.02em;
            }
        }

        @media (max-width: 360px) {
            .pid-table input[type=number] {
                font-size: 0.72rem;
                padding: 7px 1px;
                min-width: 48px;
            }

            .live-pid {
                font-size: 0.52rem;
            }

            .pid-table thead th:first-child,
            .pid-table tbody tr td:first-child {
                width: 42px;
                padding-left: 6px;
            }
        }
    </style>
</head>

<body>
    <h2>Quadcopter Control Interface</h2>

    <div class="card">
        <h3>Flight Dynamics</h3>
        <div class="telemetry">
            <div>Pitch: <br><strong id="pitchVal" style="color: #fbbf24;">0.00</strong>&deg;</div>
            <div>Roll: <br><strong id="rollVal" style="color: #38bdf8;">0.00</strong>&deg;</div>
        </div>
    </div>

    <!-- ============================================================
       CARD UNIFICATA: MOTORI + THROTTLE
       ============================================================ -->
    <div class="card">
        <h3>Motori &amp; Throttle</h3>

        <!-- Pulsante START/STOP toggle -->
        <button id="motorToggleBtn" class="btn btn-toggle-motor off" onclick="toggleMotor()">
            &#9654; START
        </button>

        <!-- Throttle -->
        <div class="thr-display">
            THR: <strong id="thVal">0</strong> / 255
        </div>
        <input type="range" id="throttleInput" min="0" max="255" value="0" oninput="sendThr(this.value)">

        <!-- Emergency stop -->
        <button class="btn btn-emergency" onclick="emergencyStop()">
            &#9888; EMERGENCY STOP
        </button>
    </div>

    <div class="card">
        <h3>Tuning PID</h3>
        <table class="pid-table">
            <thead>
                <tr>
                    <th>Asse</th>
                    <th>Proporzionale (P)</th>
                    <th>Integrativo (I)</th>
                    <th>Derivativo (D)</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td data-label="Asse" style="color: var(--accent-amber);">Pitch</td>
                    <td data-label="P">
                        <div class="pid-input-group">
                            <input type="number" id="p_pitch" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentPitchP">0.000</span></span>
                        </div>
                    </td>
                    <td data-label="I">
                        <div class="pid-input-group">
                            <input type="number" id="i_pitch" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentPitchI">0.000</span></span>
                        </div>
                    </td>
                    <td data-label="D">
                        <div class="pid-input-group">
                            <input type="number" id="d_pitch" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentPitchD">0.000</span></span>
                        </div>
                    </td>
                </tr>
                <tr>
                    <td data-label="Asse" style="color: var(--accent-blue);">Roll</td>
                    <td data-label="P">
                        <div class="pid-input-group">
                            <input type="number" id="p_roll" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentRollP">0.000</span></span>
                        </div>
                    </td>
                    <td data-label="I">
                        <div class="pid-input-group">
                            <input type="number" id="i_roll" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentRollI">0.000</span></span>
                        </div>
                    </td>
                    <td data-label="D">
                        <div class="pid-input-group">
                            <input type="number" id="d_roll" step="0.001" value="0.000" inputmode="decimal">
                            <span class="live-pid">Act: <span id="currentRollD">0.000</span></span>
                        </div>
                    </td>
                </tr>
            </tbody>
        </table>

        <button class="btn btn-pid" onclick="sendPid()">Invia Costanti PID</button>
    </div>

    <script>
        /* ============================================================
       STATO MOTORI
       ============================================================ */
        let motorOn = false;

        function toggleMotor() {
            if (motorOn) {
                stopMotor();
            } else {
                startMotor();
            }
        }

        function startMotor() {
            motorOn = true;

            const btn = document.getElementById('motorToggleBtn');
            btn.classList.remove('off');
            btn.classList.add('on');
            btn.innerHTML = '&#9608; STOP';

            const slider = document.getElementById('throttleInput');

            // Comando al backend: azione 't' (avvio motori)
            fetch('/cmd?action=t').catch(console.error);

            // Invia la potenza attualmente impostata sullo slider
            // (se era rimasta a un valore precedente, i motori ripartono da lì)
            fetch('/throttle?val=' + slider.value).catch(console.error);
        }

        function stopMotor() {
            motorOn = false;

            const btn = document.getElementById('motorToggleBtn');
            btn.classList.remove('on');
            btn.classList.add('off');
            btn.innerHTML = '&#9654; START';

            // Comando al backend: STOP motori (azione 's').
            // NON inviamo /throttle?val=0: lo stop è gestito dal firmware.
            fetch('/cmd?action=s').catch(console.error);
        }

        /* ============================================================
           EMERGENCY STOP
           Differenza rispetto a STOP normale:
           - Forza SEMPRE il throttle a 0 (indipendentemente dallo stato)
           - Invia il comando di stop di emergenza 's'
           - Resetta la UI allo stato OFF
           ============================================================ */
        function emergencyStop() {
            motorOn = false;

            const btn = document.getElementById('motorToggleBtn');
            btn.classList.remove('on');
            btn.classList.add('off');
            btn.innerHTML = '&#9654; START';

            const slider = document.getElementById('throttleInput');
            slider.value = 0;
            document.getElementById('thVal').innerText = '0';

            // Forza throttle a 0 e stop motori
            fetch('/throttle?val=0').catch(console.error);
            fetch('/cmd?action=s').catch(console.error);
        }

        /* ============================================================
           THROTTLE
           ============================================================ */
        function sendThr(val) {
            document.getElementById('thVal').innerText = val;
            // Invia solo se i motori sono accesi (evita comandi spuri)
            if (motorOn) {
                fetch('/throttle?val=' + val).catch(console.error);
            }
        }

        /* ============================================================
           PID
           ============================================================ */
        function sendPid() {
            const pp = document.getElementById('p_pitch').value;
            const ip = document.getElementById('i_pitch').value;
            const dp = document.getElementById('d_pitch').value;

            const pr = document.getElementById('p_roll').value;
            const ir = document.getElementById('i_roll').value;
            const dr = document.getElementById('d_roll').value;

            fetch(`/pid?pitch_p=${pp}&pitch_i=${ip}&pitch_d=${dp}&roll_p=${pr}&roll_i=${ir}&roll_d=${dr}`)
                .catch(console.error);
        }

        function updateCurrentPid(id, value) {
            if (value !== undefined) {
                const numericValue = Number(value);
                document.getElementById(id).innerText = Number.isFinite(numericValue) ? numericValue.toFixed(3) : value;
            }
        }

        /* ============================================================
           POLLING TELEMETRIA
           ============================================================ */
        setInterval(() => {
            fetch('/telemetry')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('pitchVal').innerText = data.pitch;
                    document.getElementById('rollVal').innerText = data.roll;
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