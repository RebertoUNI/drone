Nel tuo caso il flip va implementato principalmente nel firmware STM32, cioè in `code stm.c`. L’ESP32 dovrebbe solo inoltrare il comando `CMD:f`; la STM32 deve decidere se accettarlo, eseguirlo e interromperlo in caso di errore.

## 1. Come funziona oggi il firmware

Il flusso attuale è:

```text
pagina web
   ↓ /cmd?action=f
ESP32
   ↓ CMD:f\n tramite UART
STM32
   ↓ HAL_UART_RxCpltCallback()
coda comandi
   ↓
Process_Command()
   ↓
PID_ComputeAndMix()
   ↓
PWM dei quattro motori
```

Nel ciclo principale della STM32, il controllo viene aggiornato ogni 4 ms:

```c
if (current_time - last_time >= 4) {
  MPU6050_Read_Filter_Compute();
  PID_ComputeAndMix();
}
```

Il comportamento attuale è pensato per volo stabile:

- `pitch` e `roll` sono angoli stimati dal filtro complementare;
- il PID cerca di mantenere `pitch = 0` e `roll = 0`;
- il comando `DIR:` modifica temporaneamente il setpoint di inclinazione;
- il mixer converte le correzioni PID in quattro segnali PWM.

Un flip non è semplicemente un `DIR:` molto grande.

## 2. Perché non basta impostare `DIR_TILT_DEG` a 360°

Questa sarebbe una soluzione sbagliata:

```c
sp_pitch = 360.0f;
```

Il tuo controllo attuale non è adatto direttamente a questo per tre motivi.

### Il filtro complementare non gestisce bene il volo invertito

In `MPU6050_Read_Filter_Compute`, l’angolo viene calcolato così:

```c
float acc_pitch =
    (atan2f(-ax_filt, sqrtf(ay_filt * ay_filt + az_filt * az_filt))
     * RAD_TO_DEG) - acc_pitch_cal;
```

Durante un flip l’accelerometro non misura soltanto la gravità: misura anche le accelerazioni prodotte dai motori. Quando il drone è capovolto, il contributo dell’accelerometro può spingere il filtro verso un angolo errato.

Inoltre l’angolo accelerometrico è limitato circa a `-180...180` gradi e presenta una discontinuità vicino al passaggio da +180 a -180.

### Il PID attuale è un PID di angolo

In `PID_ComputeAndMix` fai:

```c
error_pitch = sp_pitch - pitch;
float deriv_pitch = -gyro_rate_pitch;
```

Questo funziona per mantenere il drone inclinato di pochi gradi, ma non è ideale per una rotazione acrobatica. Per un flip è preferibile controllare direttamente la **velocità angolare** del giroscopio:

```text
velocità angolare richiesta → velocità angolare misurata
```

non:

```text
angolo richiesto → angolo misurato
```

### Il drone deve continuare a generare spinta anche quando è invertito

Il codice attuale interrompe il controllo se:

```c
current_throttle < 1050
```

e limita ogni motore tra `1000` e `2000` microsecondi. Durante un flip è necessario avere una sequenza di spinta controllata; non si può semplicemente lasciare che il normale PID faccia tutto da solo.

---

# 3. La struttura consigliata: macchina a stati

Aggiungerei una macchina a stati nella STM32:

```c
typedef enum {
  FLIP_IDLE,
  FLIP_ACCELERATE,
  FLIP_ROTATE,
  FLIP_RECOVER,
  FLIP_ABORT
} FlipState;

volatile FlipState flip_state = FLIP_IDLE;
uint32_t flip_start_tick = 0;
uint32_t flip_phase_tick = 0;

float flip_angle = 0.0f;
float flip_start_angle = 0.0f;
int8_t flip_direction = 1;
```

Gli stati hanno questo significato:

| Stato | Funzione |
|---|---|
| `FLIP_IDLE` | volo normale |
| `FLIP_ACCELERATE` | aumento della spinta prima della rotazione |
| `FLIP_ROTATE` | rotazione controllata tramite il giroscopio |
| `FLIP_RECOVER` | stabilizzazione del drone |
| `FLIP_ABORT` | interruzione sicura della manovra |

Non metterei la logica del flip dentro `HAL_UART_RxCpltCallback()`. La callback gira in interrupt e deve restare molto breve. Deve soltanto ricevere il comando e inserirlo nella coda.

## 4. Aggiungere il comando nel parser STM32

Nel parser attuale, in `Process_Command`, puoi riconoscere `CMD:f`:

```c
void Process_Command(char *line) {
  if (strncmp(line, "CMD:", 4) == 0) {
    if (line[4] == 't') {
      if (!motors_enabled) {
        motor_start_tick = HAL_GetTick();
        link_last_tick = HAL_GetTick();
        motors_enabled = 1;
      }
    } else if (line[4] == 'f') {
      Flip_Request(1);
    } else if (line[4] == 's' || line[4] == 'e') {
      Motors_Kill();
    }
  }

  // resto del parser...
}
```

Serve poi dichiarare la funzione:

```c
void Flip_Request(int8_t direction);
void Flip_Update(void);
void Flip_Abort(void);
```

La funzione `Flip_Request()` non dovrebbe iniziare il flip incondizionatamente.

## 5. Verifiche obbligatorie prima di accettare il flip

Un flip dovrebbe essere accettato solo se:

```c
void Flip_Request(int8_t direction) {
  if (!motors_enabled)
    return;

  if (flip_state != FLIP_IDLE)
    return;

  if (current_throttle < 150)
    return;

  if (sp_pitch != 0.0f ||
      sp_roll != 0.0f ||
      sp_yaw_rate != 0.0f)
    return;

  if ((int32_t)(HAL_GetTick() - link_last_tick) >= LINK_TIMEOUT_MS)
    return;

  flip_direction = direction >= 0 ? 1 : -1;
  flip_angle = 0.0f;
  flip_start_angle = 0.0f;
  flip_start_tick = HAL_GetTick();
  flip_phase_tick = flip_start_tick;
  flip_state = FLIP_ACCELERATE;
}
```

I controlli importanti sono:

- motori già armati;
- throttle sufficiente;
- nessun altro comando direzionale attivo;
- collegamento ESP32 ancora valido;
- nessun flip già in corso;
- richiesta accettata una sola volta.

Il valore `150` è solo un esempio: deve corrispondere a una spinta minima realmente sufficiente nel tuo sistema.

## 6. Misurare la rotazione con il giroscopio

Per la fase di rotazione non userei direttamente `pitch` o `roll`, perché durante il flip il filtro accelerometrico può diventare inattendibile.

Puoi integrare la velocità angolare del giroscopio:

```c
flip_angle += gyro_rate_pitch * dt_actual;
```

Per un flip attorno al roll:

```c
flip_angle += gyro_rate_roll * dt_actual;
```

Il verso deve essere applicato:

```c
flip_angle += flip_direction * gyro_rate_pitch * dt_actual;
```

La logica può essere:

```c
void Flip_Update(void) {
  uint32_t now = HAL_GetTick();

  if (flip_state == FLIP_IDLE)
    return;

  if (!motors_enabled) {
    flip_state = FLIP_ABORT;
    return;
  }

  if ((int32_t)(now - link_last_tick) >= LINK_TIMEOUT_MS) {
    flip_state = FLIP_ABORT;
    return;
  }

  if (now - flip_start_tick > 1200) {
    flip_state = FLIP_ABORT;
    return;
  }

  if (flip_state == FLIP_ACCELERATE) {
    if (now - flip_phase_tick >= 100) {
      flip_angle = 0.0f;
      flip_phase_tick = now;
      flip_state = FLIP_ROTATE;
    }
  } else if (flip_state == FLIP_ROTATE) {
    flip_angle += flip_direction * gyro_rate_pitch * dt_actual;

    if (fabsf(flip_angle) >= 330.0f) {
      flip_phase_tick = now;
      flip_state = FLIP_RECOVER;
    }
  } else if (flip_state == FLIP_RECOVER) {
    if (fabsf(pitch) < 15.0f &&
        fabsf(roll) < 15.0f &&
        now - flip_phase_tick > 100) {
      flip_state = FLIP_IDLE;
    }
  } else if (flip_state == FLIP_ABORT) {
    Flip_Abort();
  }
}
```

Questo è lo scheletro della gestione, non ancora una sequenza di volo pronta: le soglie e i tempi devono essere calibrati sul drone.

Nel ciclo principale dovresti chiamarla prima di `PID_ComputeAndMix()`:

```c
if (current_time - last_time >= 4) {
  MPU6050_Read_Filter_Compute();

  Flip_Update();

  PID_ComputeAndMix();
}
```

## 7. Durante il flip devi cambiare il controllo PID

La modifica più importante è dentro `PID_ComputeAndMix`.

Oggi calcoli sempre:

```c
error_pitch = sp_pitch - pitch;
error_roll = sp_roll - roll;
```

Durante il flip dovresti usare un controllo diverso.

Una struttura possibile è:

```c
if (flip_state == FLIP_ROTATE) {
  PID_ComputeFlipMix();
  return;
}
```

con una funzione separata:

```c
void PID_ComputeFlipMix(void) {
  float target_rate = 0.0f;
  float rate_error;
  float rotation_correction;
  float base_throttle;

  if (flip_state == FLIP_ACCELERATE) {
    base_throttle = 1700.0f;
    rotation_correction = 0.0f;
  } else if (flip_state == FLIP_ROTATE) {
    target_rate = flip_direction * 300.0f;
    rate_error = target_rate - gyro_rate_pitch;

    rotation_correction =
        FLIP_RATE_P * rate_error -
        FLIP_RATE_D * gyro_rate_pitch;

    base_throttle = FLIP_ROTATE_THROTTLE;
  } else {
    base_throttle = current_throttle;
    rotation_correction = 0.0f;
  }

  // Qui il segno dipende dal verso reale dei motori e dell'IMU.
  int16_t m1 = base_throttle + rotation_correction;
  int16_t m2 = base_throttle + rotation_correction;
  int16_t m3 = base_throttle - rotation_correction;
  int16_t m4 = base_throttle - rotation_correction;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  if (motors_enabled) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4);
  }
}
```

Attenzione: questa parte non è copiabile senza verificare il verso dei motori. Il segno e la coppia di motori da aumentare dipendono da:

- configurazione X reale;
- orientamento del sensore MPU6050;
- verso di rotazione degli elicotteri;
- segni `GYRO_PITCH_SIGN`, `GYRO_ROLL_SIGN`;
- segni del mixer attuale.

Le costanti dovrebbero essere definite separatamente:

```c
#define FLIP_RATE_P          2.0f
#define FLIP_RATE_D          0.05f
#define FLIP_ROTATE_THROTTLE 1550
```

I valori sono indicativi e non devono essere usati senza prove controllate.

## 8. Evitare che il normale PID combatta il flip

Se lasci il PID normale attivo mentre il controllo flip modifica i motori, i due controlli si ostacolano.

La struttura corretta dovrebbe essere:

```c
void PID_ComputeAndMix(void) {
  if (!motors_enabled || current_throttle < 1050) {
    Reset_PID_And_Stop_Motors();
    return;
  }

  if (flip_state != FLIP_IDLE) {
    PID_ComputeFlipMix();
    return;
  }

  PID_ComputeNormalMix();
}
```

Per fare questo sarebbe opportuno separare l’attuale funzione in:

```c
void PID_ComputeAndMix(void);
void PID_ComputeNormalMix(void);
void PID_ComputeFlipMix(void);
```

In questo modo il volo normale resta invariato e il flip è isolato.

## 9. Modificare `Motors_Kill()`

La tua funzione `Motors_Kill` deve sempre annullare anche lo stato del flip:

```c
void Motors_Kill(void) {
  motors_enabled = 0;
  current_throttle = 1000;

  flip_state = FLIP_IDLE;
  flip_angle = 0.0f;

  integral_pitch = 0;
  integral_roll = 0;
  integral_yaw = 0;

  sp_pitch = 0.0f;
  sp_roll = 0.0f;
  sp_yaw_rate = 0.0f;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
}
```

Questo è indispensabile perché `CMD:e` e `CMD:s` vengono gestiti direttamente dall’interrupt UART:

```c
else if (strstr(rx_line, "CMD:e") != NULL ||
         strstr(rx_line, "CMD:s") != NULL) {
  Motors_Kill();
}
```

Il comando di stop deve quindi sempre poter interrompere il flip immediatamente.

## 10. Modificare il filtro durante il flip

Durante la rotazione è preferibile non usare pienamente la correzione accelerometrica. Puoi modificare `MPU6050_Read_Filter_Compute` in modo da ridurre la correzione accelerometrica mentre `flip_state != FLIP_IDLE`.

Per esempio:

```c
if (flip_state == FLIP_IDLE) {
  roll = COMP_ALPHA * (roll + gyro_rate_roll * dt_actual) +
         (1.0f - COMP_ALPHA) * acc_roll;

  pitch = COMP_ALPHA * (pitch + gyro_rate_pitch * dt_actual) +
          (1.0f - COMP_ALPHA) * acc_pitch;
} else {
  roll += gyro_rate_roll * dt_actual;
  pitch += gyro_rate_pitch * dt_actual;
}
```

Questa soluzione è ancora semplificata: l’angolo integrato può avere deriva, ma durante una manovra breve il giroscopio è generalmente più utile dell’accelerometro.

Dopo il flip, nel recupero, dovresti riattivare gradualmente il filtro complementare e azzerare gli integrali del PID:

```c
integral_pitch = 0.0f;
integral_roll = 0.0f;
integral_yaw = 0.0f;
```

## 11. Sequenza consigliata per lo sviluppo

Non partire subito con il flip completo. Procederei così:

1. aggiungere `CMD:f` senza modificare i motori;
2. verificare via telemetria che la STM32 riceva il comando;
3. implementare la macchina a stati, ma con i motori disabilitati;
4. testare l’integrazione di `gyro_rate_pitch` e `gyro_rate_roll`;
5. provare soltanto una breve richiesta di velocità angolare con drone vincolato;
6. verificare i versi del mixer;
7. aggiungere timeout e abort;
8. eseguire prove senza eliche;
9. solo dopo provare una manovra reale con eliche protette.

La parte più delicata del tuo firmware non è il parser `CMD:f`: è la transizione da un controllo di assetto stabile a un controllo di velocità angolare durante il flip. Il comando web e la macchina a stati sono relativamente semplici; il mixer e il controllo durante il volo invertito devono invece essere calibrati sul comportamento fisico del drone.