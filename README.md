# Progetto drone

Il drone viene controllato tramite un'interfaccia web, sfruttando la connessione Wi-Fi generata dall'ESP32. L'ESP32 funge quindi da ponte di collegamento tra l'utente e il controllore di volo (la scheda STM NUCLEO-H7A3ZI-Q). Il sistema acquisisce i dati dal sensore MPU6050 e li converte negli angoli di Pitch, Roll e Yaw, il cui errore viene poi gestito da un controllore PID.

- `main.c` -> codice H7
- `ESP_drone_control` -> Codice arduino per ESP32
- `interfaccia.h` -> codice html/javascript (pagina web ESP32)
- `test.html` -> preview pagina web

## Ai statement

Abbiamo utilizzato LLM per debug del codice della Nucleo, aiuto nella scrittura del codice dell'ESP32, infine l'abbiamo utilizzato in maniera estensiva per la realizzazione della pagina Web (interfaccia e javascript).
