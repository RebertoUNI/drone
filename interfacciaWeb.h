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

        .btn-toggle-motor.off {
            background: linear-gradient(135deg, #22c55e, #16a34a);
            color: white;
            box-shadow: 0 4px 16px rgba(34, 197, 94, 0.4);
        }

        .btn-toggle-motor.off:hover {
            box-shadow: 0 6px 24px rgba(34, 197, 94, 0.6);
            transform: translateY(-1px);
        }

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

        .btn-secondary {
            background: rgba(51, 65, 85, 0.8);
            width: 100%;
            margin: 8px 0 0 0;
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

        input[type=range]:disabled {
            opacity: 0.3;
            cursor: not-allowed;
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

        /* PID TUNING */
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
            min-height: 40px;
        }

        .live-pid {
            font-size: 0.65rem;
            color: var(--text-muted);
            font-family: ui-monospace, monospace;
            white-space: nowrap;
        }

        /* TRIM MOTORI */
        .trim-front {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.1em;
            margin-bottom: 8px;
        }

        .trim-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .trim-cell {
            background: rgba(15, 20, 28, 0.6);
            border-radius: 8px;
            padding: 10px 6px;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 6px;
        }

        .trim-cell label {
            font-size: 0.7rem;
            font-weight: 700;
            color: var(--text-dim);
            letter-spacing: 0.05em;
        }

        .trim-cell input[type=number] {
            width: 100%;
            max-width: 100px;
            background: rgba(11, 15, 23, 0.9);
            border: 1px solid rgba(51, 65, 85, 0.8);
            color: #fff;
            padding: 8px;
            border-radius: 6px;
            font-family: ui-monospace, monospace;
            text-align: center;
            font-size: 0.9rem;
            min-height: 40px;
        }

        .card-note {
            font-size: 0.65rem;
            color: var(--text-muted);
            text-align: left;
            margin-top: 10px;
            line-height: 1.4;
        }

        /* ---------- TELECOMANDO ---------- */
        :root {
            --bg: #12171d;
            --panel: #1b222b;
            --line: #2d3946;
            --text: #d6dee6;
            --dim: #7d8b99;
            --stick: #8fb8c9;
            --armed: #f2a93b;
            --kill: #e5392f;
            /* Ingrandito: ora usa fino al 68% dell'altezza disponibile dello schermo */
            --s: min(44vw, 68vh);
        }

        #remoteView {
            position: fixed;
            inset: 0;
            background: var(--bg);
            color: var(--text);
            z-index: 1000;
            display: none;
            grid-template-rows: auto 1fr;
            padding: max(8px, env(safe-area-inset-top)) max(12px, env(safe-area-inset-right)) max(8px, env(safe-area-inset-bottom)) max(12px, env(safe-area-inset-left));
            gap: 8px;
            touch-action: none;
            user-select: none;
            -webkit-user-select: none;
            text-align: left;
        }

        #remoteView * {
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }

        #remoteView .top {
            display: grid;
            grid-template-columns: 1fr auto 1fr;
            align-items: center;
            gap: 8px;
        }

        #remoteView .status {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 13px;
            color: var(--dim);
            flex-wrap: wrap;
        }

        #remoteView .center {
            display: flex;
            gap: 10px;
            align-items: center;
            justify-content: center;
        }

        #remoteView button {
            font: inherit;
            color: inherit;
            border: 0;
            cursor: pointer;
            touch-action: none;
        }

        #remoteView .arm {
            min-width: 120px;
            height: 48px;
            padding: 0 16px;
            border-radius: 24px;
            font-weight: 600;
            font-size: 15px;
            background: var(--panel);
            border: 2px solid var(--line);
            color: var(--text);
            transition: background .15s, border-color .15s, color .15s;
        }

        #remoteView .arm.armed {
            background: var(--armed);
            border-color: var(--armed);
            color: #231705;
        }

        #remoteView .kill {
            justify-self: end;
            width: 78px;
            height: 78px;
            border-radius: 50%;
            background: var(--kill);
            color: #fff;
            font-weight: 700;
            font-size: 10px;
            line-height: 1.2;
            border: 4px solid #ffd2cf;
            box-shadow: 0 0 0 3px var(--kill);
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
        }

        #remoteView .kill:active {
            transform: scale(.94);
        }

        #remoteView .kill.fired {
            background: #7c1d18;
            box-shadow: 0 0 0 3px #7c1d18;
        }

        #remoteView .sticks {
            display: flex;
            justify-content: space-around;
            align-items: center;
            min-height: 0;
            flex: 1;
            flex-wrap: nowrap;
            gap: 16px;
        }

        #remoteView .stick-wrap {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 6px;
        }

        #remoteView .label {
            font-size: 11px;
            text-align: center;
            color: var(--dim);
        }

        #remoteView .pad {
            position: relative;
            width: var(--s);
            height: var(--s);
            border-radius: 24px;
            background: var(--panel);
            border: 2px solid var(--line);
            touch-action: none;
        }

        #remoteView .pad::before,
        #remoteView .pad::after {
            content: "";
            position: absolute;
            background: var(--line);
        }

        #remoteView .pad::before {
            left: 50%;
            top: 10px;
            bottom: 10px;
            width: 1px;
        }

        #remoteView .pad::after {
            top: 50%;
            left: 10px;
            right: 10px;
            height: 1px;
        }

        #remoteView .pad.throttle::after {
            display: none;
        }

        #remoteView .pad.throttle .ticks {
            position: absolute;
            left: 10px;
            right: 10px;
            bottom: 0;
            top: 0;
            background: linear-gradient(to top, rgba(143, 184, 201, .16) var(--fill, 0%), transparent var(--fill, 0%));
            border-radius: 20px;
            pointer-events: none;
        }

        #remoteView .knob {
            position: absolute;
            left: 0;
            top: 0;
            width: 26%;
            height: 26%;
            border-radius: 50%;
            background: var(--stick);
            border: 3px solid #cfe6ef;
            box-shadow: 0 4px 14px rgba(0, 0, 0, .5);
            transform: translate(-50%, -50%);
            pointer-events: none;
        }

        #remoteView .pad.disabled .knob {
            background: #566673;
            border-color: #6c7d8a;
        }

        #remoteView .readout {
            font-variant-numeric: tabular-nums;
            font-size: 12px;
            text-align: center;
            color: var(--text);
            min-height: 1.2em;
        }
    </style>
</head>

<body>
    <div id="mainView">
        <h2>Quadcopter Control Interface</h2>
        <div style="margin-bottom: 16px;">
            <button class="btn btn-pid"
                style="background: linear-gradient(135deg, var(--accent-green), #16a34a); color: white; width: auto; padding: 12px 24px; font-size: 1rem;"
                onclick="showRemote()">&#128187; Apri Telecomando</button>
        </div>

        <div class="card" id="controlCard">
            <button id="btnToggleMotorMain" class="btn btn-toggle-motor off" onclick="toggleMotor()">ACCENDI
                MOTORI</button>
            <button class="btn btn-emergency" onclick="emergencyStop()">STOP EMERGENZA (DISARMA)</button>

            <div class="thr-display">Gas: <strong id="thrDisplayMain" class="disabled">0</strong> <span
                    style="font-size:0.6em;color:var(--text-muted)">/ 255 (2000 &micro;s)</span></div>
            <input type="range" id="thrSliderMain" min="0" max="255" value="0" disabled
                oninput="document.getElementById('thrDisplayMain').innerText = this.value; sendThr(this.value)">
            <div class="card-note">Assicurati che il gas sia a 0 prima di accendere.</div>
        </div>

        <div class="card">
            <h3>Flight Dynamics</h3>
            <div class="telemetry">
                <div>Pitch: <br><strong id="pitchVal" style="color: #fbbf24;">0.00</strong>&deg;</div>
                <div>Roll: <br><strong id="rollVal" style="color: #38bdf8;">0.00</strong>&deg;</div>
                <div>Yaw rate: <br><strong id="yawRateVal" style="color: #a78bfa;">0.00</strong>&deg;/s</div>
                <div>Battery: <br><strong id="batteryVal" style="color: #22c55e;">0.00</strong>V</div>
            </div>
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
                        <td style="color: var(--accent-amber);">Pitch</td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="p_pitch" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentPitchP">0.000</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="i_pitch" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentPitchI">0.000</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="d_pitch" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentPitchD">0.000</span></span></div>
                        </td>
                    </tr>
                    <tr>
                        <td style="color: var(--accent-blue);">Roll</td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="p_roll" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentRollP">0.000</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="i_roll" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentRollI">0.000</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="d_roll" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentRollD">0.000</span></span></div>
                        </td>
                    </tr>
                </tbody>
            </table>
            <button class="btn btn-pid" onclick="sendPid()">Invia Costanti PID</button>
        </div>

        <div class="card">
            <h3>Tuning PID Yaw</h3>
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
                        <td style="color: var(--accent-purple);">Yaw</td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="p_yaw" step="0.001" value="1.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentYawP">-</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="i_yaw" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentYawI">-</span></span></div>
                        </td>
                        <td>
                            <div class="pid-input-group"><input type="number" id="d_yaw" step="0.001" value="0.000"
                                    inputmode="decimal"><span class="live-pid">Act: <span
                                        id="currentYawD">-</span></span></div>
                        </td>
                    </tr>
                </tbody>
            </table>
            <button class="btn btn-pid" onclick="sendPidYaw()">Invia Costanti PID Yaw</button>
        </div>

        <div class="card">
            <h3>Trim Motori</h3>
            <div class="trim-front">&#9650; Fronte del drone</div>
            <div class="trim-grid">
                <div class="trim-cell">
                    <label for="trim3">M3 &middot; Front Left</label>
                    <input type="number" id="trim3" step="0.005" min="0.8" max="1.2" value="1.170" inputmode="decimal">
                    <span class="live-pid">Act: <span id="currentTrim3">-</span></span>
                </div>
                <div class="trim-cell">
                    <label for="trim4">M4 &middot; Front Right</label>
                    <input type="number" id="trim4" step="0.005" min="0.8" max="1.2" value="1.080" inputmode="decimal">
                    <span class="live-pid">Act: <span id="currentTrim4">-</span></span>
                </div>
                <div class="trim-cell">
                    <label for="trim2">M2 &middot; Back Left</label>
                    <input type="number" id="trim2" step="0.005" min="0.8" max="1.2" value="1.060" inputmode="decimal">
                    <span class="live-pid">Act: <span id="currentTrim2">-</span></span>
                </div>
                <div class="trim-cell">
                    <label for="trim1">M1 &middot; Back Right</label>
                    <input type="number" id="trim1" step="0.005" min="0.8" max="1.2" value="0.900" inputmode="decimal">
                    <span class="live-pid">Act: <span id="currentTrim1">-</span></span>
                </div>
            </div>
            <button class="btn btn-pid" onclick="sendTrim()">Invia Trim</button>
            <button class="btn btn-secondary" onclick="resetTrim()">Reset ai valori di default</button>
        </div>

        <div class="card">
            <h3>Impostazioni di Sicurezza</h3>
            <div style="display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 10px;">
                <label style="font-size: 0.85rem; font-weight: 700; color: var(--text-dim);">Timeout Motori (s)</label>
                <input type="number" id="timeoutSec" step="1" min="5" max="300" value="45" style="width: 80px; background: rgba(11, 15, 23, 0.9); border: 1px solid rgba(51, 65, 85, 0.8); color: #fff; padding: 10px; border-radius: 6px; font-family: ui-monospace, monospace; text-align: right; font-size: 0.85rem;">
            </div>
            <button class="btn btn-pid" onclick="sendTimeout()" style="background: linear-gradient(135deg, var(--accent-red), #b91c1c); box-shadow: 0 4px 12px rgba(239, 68, 68, 0.3);">Aggiorna Timeout</button>
            <div class="card-note">Valore attuale sul drone: <strong style="color:#fff;" id="currentTimeout">45.0</strong>s</div>
        </div>
    </div> <!-- end mainView -->

    <div id="remoteView" class="app">
        <div class="top">
            <div class="status">
                <button class="btn btn-secondary" onclick="showMain()"
                    style="width: auto; padding: 6px 12px; margin: 0; border-radius: 20px; font-size: 12px;">&larr;
                    Indietro</button>
                <button class="btn btn-secondary" id="fsBtn" onclick="toggleFullscreen()"
                    style="width: auto; padding: 6px 12px; margin: 0; border-radius: 20px; font-size: 12px;">&#x26F6;
                    Schermo intero</button>
                <span id="stateTxt">Motori spenti</span>
                <span style="font-weight: 600; color: #22c55e;">Bat: <span id="remoteBatteryVal">0.00</span>V</span>
            </div>
            <div class="center">
                <button class="arm" id="armBtn">Accendi</button>
            </div>
            <div style="display:flex;justify-content:flex-end">
                <button class="kill" id="killBtn">STOP<br>EMERGENZA</button>
            </div>
        </div>

        <div class="sticks">
            <div class="stick-wrap">
                <div class="pad throttle disabled" id="padL" data-min="1">
                    <div class="ticks"></div>
                    <div class="knob"></div>
                </div>
                <div class="label">Gas (su/giù) &middot; Imbardata (sx/dx)</div>
                <div class="readout" id="roL">Gas 0% &middot; Imb 0</div>
            </div>

            <div class="stick-wrap">
                <div class="pad disabled" id="padR">
                    <div class="knob"></div>
                </div>
                <div class="label">Direzione (avanti/indietro &middot; sx/dx)</div>
                <div class="readout" id="roR">Beccheggio 0% &middot; Rollio 0%</div>
            </div>
        </div>
    </div>

    <script>
        let motorOn = false;
        let motorStartedAt = 0;
        let trimSynced = false;
        let yawSynced = false;
        const THR_MAX = 120;
        const TRIM_DEFAULTS = [0.900, 1.060, 1.170, 1.080];

        /* ============================================================
           FULLSCREEN TOGGLE
           ============================================================ */
        function toggleFullscreen() {
            const fsBtn = document.getElementById("fsBtn");
            const doc = document.documentElement;

            if (!document.fullscreenElement && !document.webkitFullscreenElement) {
                if (doc.requestFullscreen) {
                    doc.requestFullscreen().catch(err => console.log(err));
                } else if (doc.webkitRequestFullscreen) {
                    doc.webkitRequestFullscreen();
                }
                if (fsBtn) fsBtn.innerHTML = "&#x2716; Esci intero";
            } else {
                if (document.exitFullscreen) {
                    document.exitFullscreen().catch(err => console.log(err));
                } else if (document.webkitExitFullscreen) {
                    document.webkitExitFullscreen();
                }
                if (fsBtn) fsBtn.innerHTML = "&#x26F6; Schermo intero";
            }
        }

        /* ============================================================
           VIEW TOGGLE
           ============================================================ */
        function showRemote() {
            document.getElementById('mainView').style.display = 'none';
            document.getElementById('remoteView').style.display = 'grid';
            // Tentativo automatico di fullscreen al click (richiede gesto utente)
            toggleFullscreen();
        }

        function showMain() {
            if (motorOn) {
                stopMotor();
            }
            if (document.fullscreenElement || document.webkitFullscreenElement) {
                toggleFullscreen();
            }
            document.getElementById('remoteView').style.display = 'none';
            document.getElementById('mainView').style.display = 'block';
        }

        /* ============================================================
           TELECOMANDO STICKS LOGIC
           ============================================================ */
        const pct = v => Math.round(v * 100) + "%";
        function makeStick(el, { springX, springY, onChange }) {
            const knob = el.querySelector(".knob");
            let id = null, x = 0, y = 0;
            let startTx = 0, startTy = 0;
            let startKx = 0, startKy = 0;

            function draw() {
                knob.style.left = (50 + x * 37) + "%";
                knob.style.top = (50 - y * 37) + "%";
            }
            function getPos(e) {
                const r = el.getBoundingClientRect();
                return {
                    nx: ((e.clientX - r.left) / r.width - 0.5) / 0.37,
                    ny: -((e.clientY - r.top) / r.height - 0.5) / 0.37
                };
            }
            el.addEventListener("pointerdown", e => {
                if (el.classList.contains("disabled") || id !== null) return;
                id = e.pointerId; el.setPointerCapture(id);
                const pos = getPos(e);
                startTx = pos.nx;
                startTy = pos.ny;
                startKx = x;
                startKy = y;
            });
            el.addEventListener("pointermove", e => {
                if (e.pointerId === id) {
                    const pos = getPos(e);
                    x = Math.max(-1, Math.min(1, startKx + (pos.nx - startTx)));
                    y = Math.max(-1, Math.min(1, startKy + (pos.ny - startTy)));
                    draw(); onChange(x, y);
                }
            });
            const end = e => {
                if (e.pointerId !== id) return;
                id = null;
                if (springX) x = 0;
                if (springY) y = 0;
                draw(); onChange(x, y);
            };
            el.addEventListener("pointerup", end);
            el.addEventListener("pointercancel", end);

            return { reset(full) { x = 0; if (full || springY) y = full ? (el.dataset.min ? -1 : 0) : 0; draw(); onChange(x, y); }, draw };
        }

        let leftStick, rightStick;
        document.addEventListener('DOMContentLoaded', () => {
            leftStick = makeStick(document.getElementById("padL"), {
                springX: true, springY: false,
                onChange(x, y) {
                    const t = (y + 1) / 2;
                    const t_val = Math.round(t * THR_MAX);
                    let y_val = 0;
                    if (x > 0.3) y_val = -1;
                    else if (x < -0.3) y_val = 1;

                    document.getElementById("padL").querySelector(".ticks").style.setProperty("--fill", pct(t));
                    document.getElementById("roL").textContent = "Gas " + pct(t) + " · Imb " + y_val;

                    if (motorOn) {
                        thrPending = t_val;
                        flushThr();
                        setDir('y', y_val);
                    }
                }
            });

            rightStick = makeStick(document.getElementById("padR"), {
                springX: true, springY: true,
                onChange(x, y) {
                    const p = Math.round(y * 100);
                    const r = Math.round(x * 100);
                    document.getElementById("roR").textContent = "Beccheggio " + p + "% · Rollio " + r + "%";
                    if (motorOn) {
                        setStick(p, r);
                    }
                }
            });
            throttleToZero();
            rightStick.draw();

            const armBtn = document.getElementById("armBtn");
            const killBtn = document.getElementById("killBtn");
            let estopFired = false;

            armBtn.addEventListener("click", () => {
                if (estopFired) {
                    estopFired = false;
                    killBtn.classList.remove("fired");
                    killBtn.innerHTML = "STOP<br>EMERGENZA";
                }
                if (motorOn) {
                    stopMotor();
                } else {
                    const t_val = thrPending !== null ? thrPending : 0;
                    if (t_val < 5) {
                        startMotor();
                    } else {
                        document.getElementById("stateTxt").textContent = "Abbassa il gas per accendere";
                    }
                }
            });

            killBtn.addEventListener("pointerdown", e => {
                e.preventDefault();
                estopFired = true;
                emergencyStop();
                killBtn.classList.add("fired");
                killBtn.innerHTML = "FERMO<br>(tocca Accendi)";
                document.getElementById("stateTxt").textContent = "Emergenza: motori fermati";
            });
        });

        function throttleToZero() {
            if (leftStick) leftStick.reset(true);
            const slider = document.getElementById('thrSliderMain');
            if (slider) {
                slider.value = 0;
                document.getElementById('thrDisplayMain').innerText = "0";
            }
        }

        function lockSticks(locked) {
            const padL = document.getElementById('padL');
            const padR = document.getElementById('padR');
            if (padL) padL.classList.toggle("disabled", locked);
            if (padR) padR.classList.toggle("disabled", locked);
        }

        function setMotorUI(on) {
            motorOn = on;
            const btn = document.getElementById('armBtn');
            const btnMain = document.getElementById('btnToggleMotorMain');
            const thrSliderMain = document.getElementById('thrSliderMain');
            const thrDisplayMain = document.getElementById('thrDisplayMain');

            if (btn) {
                btn.classList.toggle('armed', on);
                btn.textContent = on ? "Spegni" : "Accendi";
                lockSticks(!on);
            }
            if (btnMain) {
                btnMain.classList.toggle('off', !on);
                btnMain.classList.toggle('on', on);
                btnMain.innerHTML = on ? "STOP MOTORI" : "ACCENDI MOTORI";
            }
            if (thrSliderMain) thrSliderMain.disabled = !on;
            if (thrDisplayMain) thrDisplayMain.classList.toggle('disabled', !on);

            const stateTxt = document.getElementById('stateTxt');
            if (stateTxt) stateTxt.textContent = on ? "Motori accesi" : "Motori spenti";

            if (!on) {
                throttleToZero();
                if (rightStick) rightStick.reset();
                clearDir();
            }
        }

        function resetThrottleUI() {
            thrPending = 0;
            throttleToZero();
        }

        function toggleMotor() {
            if (motorOn) stopMotor(); else startMotor();
        }

        function startMotor() {
            resetThrottleUI();
            setMotorUI(true);
            motorStartedAt = Date.now();
            fetch('/cmd?action=t').catch(console.error);
        }

        function stopMotor() {
            setMotorUI(false);
            resetThrottleUI();
            fetch('/cmd?action=s').catch(console.error);
        }

        function emergencyStop() {
            setMotorUI(false);
            resetThrottleUI();
            const send = () => fetch('/estop', { cache: 'no-store' }).catch(console.error);
            send();
            setTimeout(send, 150);
            setTimeout(send, 400);
        }

        let thrInFlight = false;
        let thrPending = null;

        function sendThr(val) {
            if (!motorOn) return;
            thrPending = val;
            flushThr();
        }

        function flushThr() {
            if (thrInFlight || thrPending === null) return;
            const v = thrPending;
            thrPending = null;
            thrInFlight = true;
            fetch('/throttle?val=' + v)
                .catch(console.error)
                .finally(() => { thrInFlight = false; flushThr(); });
        }

        const dir = { p: 0, r: 0, y: 0 };
        let dirInFlight = false;
        let dirDirty = false;

        function flushDir() {
            if (dirInFlight || !dirDirty) return;
            dirDirty = false;
            dirInFlight = true;
            fetch(`/dir?p=${dir.p}&r=${dir.r}&y=${dir.y}`, { cache: 'no-store' })
                .catch(console.error)
                .finally(() => { dirInFlight = false; flushDir(); });
        }

        function setDir(axis, val) {
            if (dir[axis] === val) return;
            dir[axis] = val;
            dirDirty = true;
            flushDir();
        }

        function clearDir() {
            dir.p = dir.r = dir.y = 0;
            dirDirty = false;
            document.querySelectorAll('.dir-btn.active').forEach(b => b.classList.remove('active'));
        }

        setInterval(() => {
            if (dir.p || dir.r || dir.y) { dirDirty = true; flushDir(); }
        }, 100);

        function setStick(p, r) {
            if (dir.p === p && dir.r === r) return;
            dir.p = p;
            dir.r = r;
            dirDirty = true;
            flushDir();
        }

        const KEYMAP = {
            ArrowUp: ['p', 100], ArrowDown: ['p', -100],
            ArrowLeft: ['r', -100], ArrowRight: ['r', 100],
            KeyQ: ['y', 1], KeyE: ['y', -1]
        };
        document.addEventListener('keydown', e => {
            if (e.target.type === 'number') return;
            const k = KEYMAP[e.code];
            if (!k) return;
            e.preventDefault();
            setDir(k[0], k[1]);
        });
        document.addEventListener('keyup', e => {
            const k = KEYMAP[e.code];
            if (k && dir[k[0]] === k[1]) setDir(k[0], 0);
        });
        window.addEventListener('blur', () => { clearDir(); dirDirty = true; flushDir(); });

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

        function sendPidYaw() {
            const yp = parseFloat(document.getElementById('p_yaw').value);
            const yi = parseFloat(document.getElementById('i_yaw').value);
            const yd = parseFloat(document.getElementById('d_yaw').value);
            if ([yp, yi, yd].some(x => !Number.isFinite(x))) {
                alert('Inserisci valori numerici validi per il PID Yaw');
                return;
            }
            fetch(`/pidyaw?yaw_p=${yp.toFixed(3)}&yaw_i=${yi.toFixed(3)}&yaw_d=${yd.toFixed(3)}`)
                .catch(console.error);
        }

        function updateCurrentPid(id, value) {
            if (value !== undefined && value !== '') {
                const numericValue = Number(value);
                document.getElementById(id).innerText = Number.isFinite(numericValue) ? numericValue.toFixed(3) : value;
            }
        }

        function sendTrim() {
            const v = [1, 2, 3, 4].map(i => parseFloat(document.getElementById('trim' + i).value));
            if (v.some(x => !Number.isFinite(x) || x < 0.8 || x > 1.2)) {
                alert('Valori ammessi: da 0.800 a 1.200');
                return;
            }
            const q = v.map(x => x.toFixed(3));
            fetch(`/trim?m1=${q[0]}&m2=${q[1]}&m3=${q[2]}&m4=${q[3]}`).catch(console.error);
        }

        function resetTrim() {
            [1, 2, 3, 4].forEach(i => document.getElementById('trim' + i).value = TRIM_DEFAULTS[i - 1].toFixed(3));
            sendTrim();
        }

        function sendTimeout() {
            const t = parseFloat(document.getElementById('timeoutSec').value);
            if (!Number.isFinite(t) || t < 5) {
                alert('Inserisci un timeout valido in secondi (min 5)');
                return;
            }
            fetch('/timeout?ms=' + Math.floor(t * 1000)).catch(console.error);
        }

        let telBusy = false;
        setInterval(() => {
            if (telBusy) return;
            telBusy = true;
            const ctl = new AbortController();
            const to = setTimeout(() => ctl.abort(), 1000);
            fetch('/telemetry', { signal: ctl.signal })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('pitchVal').innerText = data.pitch;
                    document.getElementById('rollVal').innerText = data.roll;
                    if (data.yaw_rate !== undefined && data.yaw_rate !== '') {
                        document.getElementById('yawRateVal').innerText = data.yaw_rate;
                    }
                    if (data.battery !== undefined) {
                        const batMain = document.getElementById('batteryVal');
                        if (batMain) batMain.innerText = data.battery;
                        const batRemote = document.getElementById('remoteBatteryVal');
                        if (batRemote) batRemote.innerText = data.battery;
                    }
                    updateCurrentPid('currentPitchP', data.pid_pitch_p);
                    updateCurrentPid('currentPitchI', data.pid_pitch_i);
                    updateCurrentPid('currentPitchD', data.pid_pitch_d);
                    updateCurrentPid('currentRollP', data.pid_roll_p);
                    updateCurrentPid('currentRollI', data.pid_roll_i);
                    updateCurrentPid('currentRollD', data.pid_roll_d);

                    updateCurrentPid('currentYawP', data.pid_yaw_p);
                    updateCurrentPid('currentYawI', data.pid_yaw_i);
                    updateCurrentPid('currentYawD', data.pid_yaw_d);
                    if (!yawSynced && data.pid_yaw_p !== undefined && data.pid_yaw_p !== '' &&
                        data.pid_yaw_i !== '' && data.pid_yaw_d !== '') {
                        document.getElementById('p_yaw').value = Number(data.pid_yaw_p).toFixed(3);
                        document.getElementById('i_yaw').value = Number(data.pid_yaw_i).toFixed(3);
                        document.getElementById('d_yaw').value = Number(data.pid_yaw_d).toFixed(3);
                        yawSynced = true;
                    }

                    for (let i = 1; i <= 4; i++) {
                        updateCurrentPid('currentTrim' + i, data['trim_m' + i]);
                    }
                    if (!trimSynced && data.trim_m1 !== undefined && data.trim_m1 !== '') {
                        for (let i = 1; i <= 4; i++) {
                            document.getElementById('trim' + i).value = Number(data['trim_m' + i]).toFixed(3);
                        }
                        trimSynced = true;
                    }
                    if (data.timeout_ms !== undefined) {
                        document.getElementById('currentTimeout').innerText = (parseInt(data.timeout_ms) / 1000).toFixed(1);
                    }

                    if (data.motors !== undefined && motorOn &&
                        String(data.motors) === '0' &&
                        Date.now() - motorStartedAt > 1500) {
                        setMotorUI(false);
                        resetThrottleUI();
                    }
                })
                .catch(err => console.error(err))
                .finally(() => { clearTimeout(to); telBusy = false; });
        }, 200);
    </script>
</body>

</html>
)rawliteral";

#endif
