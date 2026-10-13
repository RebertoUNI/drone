

Nel tuo progetto la trovi definita alla riga 610 di : **`HAL_UART_RxCpltCallback()`**.

---

### Come funziona nel dettaglio

La funzione:
```c
HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
```

1. **`_IT` sta per Interrupt (Non bloccante)**:
   A differenza di `HAL_UART_Receive(...)` standard (che è bloccante e "congela" la CPU finché non arriva un byte o scade un timeout), la versione `_IT` configura solo i registri hardware della periferica USART2 e **ritorna immediatamente**. La CPU è libera di continuare a calcolare il loop del drone (MPU6050, PID, ecc.).

2. **I parametri**:
   - `&huart2`: l'handle della periferica UART collegata all'ESP32.
   - `&esp32_rx_byte`: il puntatore al buffer di memoria (nel tuo caso una variabile da 1 byte) dove depositare il dato ricevuto.
   - `1`: il numero di byte da ricevere prima di considerare completata l'operazione.

---

### Il ciclo di vita di un byte ricevuto

```mermaid
sequenceDiagram
    participant ESP32
    participant Hardware as STM32 UART2 (HW)
    participant HAL as HAL_UART_IRQHandler
    participant Callback as HAL_UART_RxCpltCallback
    participant Loop as While(1) Loop Drone

    Note over Hardware: HAL_UART_Receive_IT armato (1 byte)
    Loop->>Loop: Esegue algoritmo di volo (4ms)
    ESP32->>Hardware: Invia 1 byte (es. 'C')
    Hardware->>HAL: Solleva Interrupt (IRQ)
    HAL->>HAL: Salva il byte in esp32_rx_byte
    HAL->>Callback: Chiama automaticamente HAL_UART_RxCpltCallback()
    Callback->>Callback: Accoda il carattere o esegue Motors_Kill()
    Callback->>Hardware: HAL_UART_Receive_IT(...) (Riarma per il prossimo byte)
    HAL->>Loop: Ritorna al loop principale
```

1. **Arrivo del dato**: Quando l'ESP32 spedisce un byte sul pin RX, l'hardware della USART2 lo riceve e genera un'interruzione CPU (**IRQ**).
2. **Intervento dell'HAL**: La CPU sospende per qualche microsecondo il loop principale e salta alla routine `HAL_UART_IRQHandler()`.
3. **Copia in memoria**: Il driver HAL legge il registro hardware `DR`/`RDR` e copia il valore nella variabile `esp32_rx_byte`.
4. **Chiamata alla Callback**: Avendo raggiunto la dimensione richiesta (`1` byte), l'HAL invoca automaticamente la callback:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
   ```
   Nel tuo codice, questa callback legge `esp32_rx_byte`:
   - Se riceve caratteri, li accumula nel buffer `rx_line`.
   - Se riceve un terminatore `\n`, controlla subito se si tratta di un comando di arresto immediato (`CMD:s` o `CMD:e`) chiamando istantaneamente [`Motors_Kill()`](file:///Users/roberto/Università/Drone/drone/code%20stm.c#L622).
   - Se è un heartbeat (`HB:`), aggiorna subito il timestamp `link_last_tick`.

---

### Il dettaglio fondamentale: È "One-Shot" (Va riarmata!)

`HAL_UART_Receive_IT` **non resta attiva per sempre**: una volta ricevuto il byte richiesto, la ricezione via interrupt si disattiva.

Per questo motivo, nel tuo codice viene **riarmata esplicitamente**:
1. **Alla fine della callback stessa** ([code stm.c:639](file:///Users/roberto/Università/Drone/drone/code%20stm.c#L639)):
   ```c
   HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
   ```
   In questo modo, appena finisci di processare un byte, la UART si rimette in attesa del byte successivo.
2. **Nella gestione errori** ([code stm.c:653](file:///Users/roberto/Università/Drone/drone/code%20stm.c#L653)):
   In [`HAL_UART_ErrorCallback`](file:///Users/roberto/Università/Drone/drone/code%20stm.c#L647) viene ripulita e riarmata se si verifica rumore o overrun sulla linea.
3. **Come rete di sicurezza nel `while(1)`** ([code stm.c:220-222](file:///Users/roberto/Università/Drone/drone/code%20stm.c#L220-L222)):
   ```c
   if (huart2.RxState == HAL_UART_STATE_READY) {
     HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
   }
   ```
   Se per qualsiasi motivo anomalo la ricezione si fosse fermata, il loop principale la riavvia.

## Perché un byte alla volta?

Riceve **1 byte alla volta** per un motivo fondamentale: **i messaggi inviati dall'ESP32 hanno lunghezza variabile e non sai a priori quanti byte arriveranno.**

---

### 1. Il problema del parametro `Size` di `HAL_UART_Receive_IT`
La funzione `HAL_UART_Receive_IT` chiama la callback **solo ed esclusivamente quando ha ricevuto esattamente il numero di byte richiesto**.

Immagina di impostare ad esempio `Size = 10`:
* Se dall'ESP32 invii il comando di stop di emergenza: `"CMD:s\n"` (**6 byte**)
* L'STM32 riceve i 6 byte, ma **non chiama la callback**, perché ne sta ancora aspettando altri 4 per arrivare a 10!
* Il drone continuerebbe a far girare i motori finché non invii un altro comando per "sbloccare" il conteggio.

---

### 2. Le lunghezze dei tuoi comandi sono tutte diverse
Nel tuo drone hai comandi come:
* `"CMD:s\n"` &rarr; **6 byte** (Stop)
* `"CMD:e\n"` &rarr; **6 byte** (Emergency kill)
* `"HB:1234\n"` &rarr; **8 byte** (Heartbeat)
* `"DIR:15.5,-10.0,0\n"` &rarr; **17 byte** (Joystick direzionale)

Nessun numero fisso andrebbe bene per tutti.

---

### 3. Riconoscimento del terminatore `\n` in tempo reale
Ricevendo **1 byte alla volta**, l'STM32 può ispezionare ogni singolo carattere appena entra nel pin:
1. Riceve `'C'`, lo mette in buffer.
2. Riceve `'M'`, lo mette in buffer.
3. Riceve `'D'`, poi `':'`, poi `'s'`.
4. Riceve **`'\n'`** &rarr; Capisce che **la riga è terminata**, chiude la stringa con `\0` ed esegue subito:
   ```c
   // code stm.c riga 620-622
   else if (strstr(rx_line, "CMD:e") != NULL || strstr(rx_line, "CMD:s") != NULL) {
     Motors_Kill(); // Spegnimento immediato istantaneo!
   }
   ```

---

### Riassunto
| Approccio | Vantaggio | Svantaggio |
| :--- | :--- | :--- |
| **1 byte alla volta (il tuo attuale)** | Semplicissimo, reagisce istantaneamente a `\n` e spegne subito i motori. | Sollecita un interrupt per ogni singolo carattere (a 115200 baud per pochi comandi al secondo l'overhead sulla CPU è comunque trascurabile, < 0.1%). |
| **Multi-byte fisso (es. 10 byte)** | Meno interrupt. | **Inutilizzabile** con stringhe a lunghezza variabile (si blocca in attesa). |
| **DMA + Idle Line Detection** *(alternativa avanzata)* | Riceve pacchetti a lunghezza variabile a zero carico CPU. | Più complesso da configurare rispetto a 1 byte in interrupt. |