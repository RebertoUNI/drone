Nel progetto del drone l'**heartbeat** (battito cardiaco) è una misura di sicurezza fondamentale (**failsafe a catena**) progettata per garantire che i motori vengano spenti immediatamente nel caso in cui cada la connessione o uno dei componenti smetta di rispondere.

Il sistema è strutturato a 3 livelli:
$$\text{Browser / Web Client} \xrightarrow{\text{Wi-Fi}} \text{ESP32} \xrightarrow{\text{UART}} \text{STM32}$$

---

### 1. Il livello iniziale: Web Client (Browser)
Nella pagina web, uno script JavaScript esegue un loop periodico a **100 ms**:

```javascript
/* ---------- HEARTBEAT: se si ferma, il drone si spegne ---------- */
let hbBusy = false;
setInterval(() => {
    if (hbBusy) return;
    hbBusy = true;
    const ctl = new AbortController();
    const to = setTimeout(() => ctl.abort(), 400);
    fetch('/hb', { cache: 'no-store', signal: ctl.signal })
        .catch(() => { })
        .finally(() => { clearTimeout(to); hbBusy = false; });
}, 100);
```

* **Cosa fa:** Invia costantemente una richiesta HTTP `GET /hb` all'ESP32.
* **Cosa succede se crolla:** Se chiudi la scheda del browser, il browser si blocca o il telefono esce dal raggio Wi-Fi, le richieste `/hb` cessano all'istante.

---

### 2. Come funziona l'Heartbeat sull'ESP32
Nel firmware dell'ESP32, l'ESP svolge il ruolo di **ponte e validatore**:

#### A. Ricezione dal client e verifica di presenza
Quando l'ESP riceve la richiesta web `/hb`, aggiorna il timestamp in handleHb():
```cpp
void handleHb() {
  last_client_ms = millis();
  server.send(200, "text/plain", "OK");
}
```

La funzione clientAlive() valuta se il client è effettivamente "vivo":
```cpp
bool clientAlive() {
  return WiFi.softAPgetStationNum() > 0 &&
         (millis() - last_client_ms < CLIENT_TIMEOUT_MS); // CLIENT_TIMEOUT_MS = 500 ms
}
```
Affinché il client sia considerato vivo, devono verificarsi due condizioni:
1. Almeno un dispositivo è connesso all'Access Point Wi-Fi (`softAPgetStationNum() > 0`).
2. L'ultimo heartbeat dalla pagina è arrivato meno di **500 ms** fa.

#### B. Generazione dell'heartbeat verso l'STM32
Nel loop(), l'ESP decide cosa trasmettere sulla porta seriale `Serial2`:
```cpp
// Heartbeat verso l'STM32: lo mandiamo SOLO se il client e' vivo.
if (clientAlive()) {
  if (millis() - last_hb_tx_ms >= HB_PERIOD_MS) { // HB_PERIOD_MS = 100 ms
    last_hb_tx_ms = millis();
    Serial2.print("HB:1\n");
  }
} else if ((esp_motors_on || current_motors == "1") &&
           millis() - last_kill_tx_ms >= 200) {
  // Client perso con motori accesi: kill esplicito (ripetuto ogni 200 ms)
  last_kill_tx_ms = millis();
  sendKillToStm();
}
```
* **Se il client è vivo:** Ogni **100 ms**, l'ESP invia sulla seriale verso l'STM32 la stringa `"HB:1\n"`.
* **Se il client non è più vivo:** L'ESP smette di mandare `"HB:1\n"` e invia attivamente a raffica il comando di emergenza [sendKillToStm()] (`"CMD:e\n"` e `"THR:0\n"`).

---

### 3. Come funziona l'Heartbeat sull'STM32
L'STM32 è il controller di volo vero e proprio. Poiché ha il controllo diretto sui segnali PWM dei motori, è l'arbitro finale della sicurezza.

#### A. Ricezione immediata tramite Interrupt UART (ISR)
Quando l'STM32 riceve i caratteri dalla seriale via interrupt, la callback [HAL_UART_RxCpltCallback] esamina la riga completata:
```c
} else if (strncmp(rx_line, "HB:", 3) == 0) {
    link_last_tick = HAL_GetTick(); // heartbeat: non va in coda
}
```
* **Zero latenza:** L'heartbeat **non viene inserito nella coda dei comandi** `rx_queue` (dove finiscono invece setpoint, trim o PID).
* Aggiorna direttamente la variabile `link_last_tick` con il millisecondo corrente (`HAL_GetTick()`). In questo modo, anche se la coda dei comandi fosse occupata, il watchdog del collegamento riceve il battito istantaneamente.

#### B. Watchdog del collegamento nel ciclo principale
Nel `while(1)` di [main()], ad ogni iterazione (prima ancora del ciclo di controllo PID a 250 Hz) viene controllato il timeout:
```c
// WATCHDOG COLLEGAMENTO: nessun heartbeat valido -> spegni i motori.
// Cast a int32_t: link_last_tick viene aggiornato in ISR e puo' essere
// "piu' nuovo" di current_time (la sottrazione unsigned darebbe un kill falso)
if (motors_enabled &&
    ((int32_t)(current_time - link_last_tick) >= LINK_TIMEOUT_MS)) { // LINK_TIMEOUT_MS = 500 ms
  Motors_Kill();
}
```
* Se i motori sono attivi (`motors_enabled == 1`) e sono passati più di **500 ms** dall'ultimo heartbeat valido, viene chiamata Motors_Kill().
* Motors_Kill() azzera istantaneamente l'abilitazione dei motori, porta i registri di compare dei 4 timer PWM al minimo (`1000 µs`), azzera la manetta e cancella tutti gli accumulatori integrali del PID (`integral_pitch`, `integral_roll`, `integral_yaw`) per azzerare l'inerzia di calcolo.

---

### Riepilogo scenari di guasto coperti

| Scenario di Guasto | Cosa succede | Risultato |
| :--- | :--- | :--- |
| **Cade il Wi-Fi o si chiude la pagina** | Il browser smette di fare `fetch('/hb')`. L'ESP32 nota il timeout (> 500 ms) e **smette di inviare `HB:1`** all'STM32. | Dopo 500 ms l'STM32 entra in timeout e spegne i motori. |
| **L'ESP32 si blocca / crasha / si riavvia** | L'ESP32 smette di trasmettere sulla porta seriale UART. | Dopo 500 ms l'STM32 non rileva più `HB:` ed esegue `Motors_Kill()`. |
| **Si scollega il filo seriale tra ESP e STM** | I byte seriali smettono di raggiungere l'interrupt dell'STM32. | Dopo 500 ms l'STM32 spegne automaticamente i motori. |