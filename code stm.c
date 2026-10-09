/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Drone Flight Controller
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0 // Indirizzo I2C (AD0 a GND)

// Costanti Matematiche e di Sistema
#define RAD_TO_DEG 57.2957795131f
#define LPF_ALPHA                                                              
  0.1f // Fattore filtro Passa-Basso (aumentato per ridurre phase lag)
#define COMP_ALPHA 0.999f // Fattore filtro Complementare
#define GYRO_SCALE 65.5f  // Scala giroscopio per +/- 500 deg/s

// Mapping giroscopio -> angolo
#define GYRO_ROLL_SIGN 1.0f  // gyro X -> roll
#define GYRO_PITCH_SIGN 1.0f // gyro Y -> pitch
#define GYRO_YAW_SIGN 1.0f   // gyro Z -> yaw rate

// Yaw: segno di miscelazione e limiti
// Se il drone, con yaw attivo, tende a ruotare SEMPRE PIU' VELOCE invece di
// stabilizzarsi, inverti YAW_MIX_SIGN (1.0f <-> -1.0f).
#define YAW_MIX_SIGN 1.0f
#define YAW_OUT_MAX 150.0f // massima correzione yaw in us sui motori
#define YAW_I_MAX 60.0f    // limite anti-windup dell'integrale yaw
#define YAW_D_LPF 0.2f     // filtro passa-basso sulla derivata yaw (0..1)

// Controllo direzionale da web (inclinazione fissa, non graduale)
#define DIR_TILT_DEG 5.0f      // inclinazione MASSIMA (joystick a fondo corsa)
#define DIR_YAW_RATE_DPS 30.0f // velocita' di imbardata in deg/s
#define DIR_TIMEOUT_MS 400     // se non arrivano DIR, torna in piano
// Se un comando va nel verso sbagliato, metti -1.0f
#define DIR_PITCH_SIGN 1.0f
#define DIR_ROLL_SIGN 1.0f
#define DIR_YAW_SIGN 1.0f

// Sicurezza e trim motori
#define MOTOR_TIMEOUT_MS 12000 // spegnimento automatico dopo 8 s
#define TRIM_MIN 0.80f         // limiti moltiplicatori motori
#define TRIM_MAX 1.20f
#define RX_LINE_LEN 64
#define RX_QUEUE_LEN 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Variabili MPU6050 (Filtri)
float ax_filt = 0, ay_filt = 0, az_filt = 0;
float pitch = 0, roll = 0;
uint32_t last_time = 0;
float dt_actual = 0.004f;

// Variabili Calibrazione
float gyro_x_cal = 0.0f, gyro_y_cal = 0.0f, gyro_z_cal = 0.0f;
float acc_pitch_cal = -4.79f, acc_roll_cal = -0.89f; // -4.79   -0.89
float gyro_rate_pitch = 0.0f;
float gyro_rate_roll = 0.0f;
float gyro_rate_yaw = 0.0f;

// Variabili UART ESP32
uint8_t esp32_rx_byte;
char rx_line[RX_LINE_LEN]; // riga in costruzione (ISR)
uint8_t rx_index = 0;
volatile uint8_t rx_overflow = 0;
char rx_queue[RX_QUEUE_LEN][RX_LINE_LEN]; // coda comandi ISR -> main
volatile uint8_t rx_q_head = 0, rx_q_tail = 0;
char telemetry_tx_buffer[384];
volatile uint8_t telemetry_tx_busy = 0;

// Stato di sistema
uint32_t current_throttle = 1000;
float pid_pitch_p = 2.16f, pid_pitch_i = 0.7f, pid_pitch_d = 0.8f;
float pid_roll_p = 2.16f, pid_roll_i = 0.9f, pid_roll_d = 0.85f;
// PID Yaw (controllo sulla velocita' angolare, setpoint 0 deg/s).
// Parti con valori bassi e alzali dalla pagina web.
float pid_yaw_p = 0.3f, pid_yaw_i = 0.8f, pid_yaw_d = 0.1f;
volatile uint8_t motors_enabled = 0;
uint32_t motor_start_tick = 0;

// Moltiplicatori motori (1.0 = nessuna correzione) - VALORI DI DEFAULT
// 0 = M1 Front Left, 1 = M2 Front Right, 2 = M3 Back Right, 3 = M4 Back Left
float motor_trim[4] = {0.900f, 1.060f, 1.170f, 1.080f};

// Variabili PID
float error_pitch = 0, integral_pitch = 0;
float error_roll = 0, integral_roll = 0;
float error_yaw = 0, integral_yaw = 0;
float prev_gyro_rate_yaw = 0, yaw_d_filt = 0;

// Setpoint da comando direzionale (0 = hovering)
float sp_pitch = 0.0f, sp_roll = 0.0f, sp_yaw_rate = 0.0f;
uint32_t dir_last_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_Filter_Compute(void);
void PID_ComputeAndMix(void);
void Motors_Kill(void);
void Process_Command(char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  // Inizializzazione del sensore MPU6050
  MPU6050_Init();

  // Avvia i segnali PWM sui 4 canali del Timer 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Armamento ESC (1000 us minimo)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
  HAL_Delay(4000); // Attesa completamento boot ESC

  // Attiva la ricezione UART in modalità Interrupt
  HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);

  last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t current_time = HAL_GetTick();

    // WATCHDOG: spegnimento automatico dopo 8 s, controllato ad ogni giro
    // (fuori dal blocco a 4 ms)
    if (motors_enabled &&
        (current_time - motor_start_tick >= MOTOR_TIMEOUT_MS)) {
      Motors_Kill();
    }

    // Timeout comando direzionale: senza DIR recenti torna in piano
    if ((sp_pitch != 0.0f || sp_roll != 0.0f || sp_yaw_rate != 0.0f) &&
        (current_time - dir_last_tick >= DIR_TIMEOUT_MS)) {
      sp_pitch = 0.0f;
      sp_roll = 0.0f;
      sp_yaw_rate = 0.0f;
    }

    // Se la ricezione UART si fosse fermata (errore), riarmala
    if (huart2.RxState == HAL_UART_STATE_READY) {
      HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
    }

    // ==========================================
    // CONTROL LOOP: Eseguito ogni 4ms (250Hz)
    // ==========================================
    if (current_time - last_time >= 4) {
      // 1. Acquisizione I2C e applicazione filtri Digitali (Passa Basso +
      // Complementare)
      MPU6050_Read_Filter_Compute();

      // 2. Calcolo errori, computazione PID e miscelazione motori
      PID_ComputeAndMix();

      // 3. Telemetria verso ESP32 (Eseguita ogni 40ms -> 25Hz per evitare
      // saturazione UART)
      static uint8_t telemetry_counter = 0;
      if (++telemetry_counter >= 10) {
        if (!telemetry_tx_busy) {
          int len = snprintf(
              telemetry_tx_buffer, sizeof(telemetry_tx_buffer),
              "ANG:%.2f,%.2f,"
              "yaw_rate:%.2f,"
              "pid_pitch_p:%.3f,pid_pitch_i:%.3f,pid_pitch_d:%.3f,"
              "pid_roll_p:%.3f,pid_roll_i:%.3f,pid_roll_d:%.3f,"
              "pid_yaw_p:%.3f,pid_yaw_i:%.3f,pid_yaw_d:%.3f,"
              "trim1:%.3f,trim2:%.3f,trim3:%.3f,trim4:%.3f,"
              "motors:%d\n",
              pitch, roll, gyro_rate_yaw, pid_pitch_p, pid_pitch_i, pid_pitch_d,
              pid_roll_p, pid_roll_i, pid_roll_d, pid_yaw_p, pid_yaw_i,
              pid_yaw_d, motor_trim[0], motor_trim[1], motor_trim[2],
              motor_trim[3], (int)motors_enabled);

          if (len > 0 && len < (int)sizeof(telemetry_tx_buffer)) {
            telemetry_tx_busy = 1;
            if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)telemetry_tx_buffer,
                                     (uint16_t)len) != HAL_OK) {
              telemetry_tx_busy = 0;
            }
          }
        }
        telemetry_counter = 0;
      }
    }

    // ==========================================
    // PARSING DEI COMANDI ESP32 (coda riempita dalla ISR)
    // ==========================================
    while (rx_q_tail != rx_q_head) {
      char local_buffer[RX_LINE_LEN];
      strncpy(local_buffer, rx_queue[rx_q_tail], RX_LINE_LEN - 1);
      local_buffer[RX_LINE_LEN - 1] = '\0';
      rx_q_tail = (rx_q_tail + 1) % RX_QUEUE_LEN;
      Process_Command(local_buffer);
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Inizializzazione MPU6050 via I2C
 */
void MPU6050_Init(void) {
  uint8_t check, data;

  // Verifica presenza MPU6050 controllando il registro WHO_AM_I (0x75)
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
  // firma HAL_I2C_Mem_Read: HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

  if (check == 104) { // 104 = 0x68 (Default per MPU6050)
    // 1. Risveglio MPU6050
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
    // 2. Configurazione Giroscopio: Fondo scala +/- 500 deg/s
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
    // 3. Configurazione Accelerometro: Fondo scala +/- 2g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);

    // 4. Filtro passa basso integrato nel MPU6050 (0x03 = 42Hz) per ridurre
    // phase lag

    // modificato 188Hz 01

    data = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);

    // --- 4. CALIBRAZIONE GIROSCOPIO E ACCELEROMETRO ---
    HAL_Delay(4000);

    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float acc_pitch_sum = 0.0f, acc_roll_sum = 0.0f;
    uint8_t rec_data[14];
    const int num_samples = 2000;

    for (int i = 0; i < num_samples; i++) {
      HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, rec_data, 14, 100);

      // Somma Giroscopio
      gx_sum += (int16_t)(rec_data[8] << 8 | rec_data[9]);
      gy_sum += (int16_t)(rec_data[10] << 8 | rec_data[11]);
      gz_sum += (int16_t)(rec_data[12] << 8 | rec_data[13]);

      // Calcolo e somma angoli grezzi Accelerometro
      int16_t ax = (int16_t)(rec_data[0] << 8 | rec_data[1]);
      int16_t ay = (int16_t)(rec_data[2] << 8 | rec_data[3]);
      int16_t az = (int16_t)(rec_data[4] << 8 | rec_data[5]);

      acc_pitch_sum += atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      acc_roll_sum += atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;

      HAL_Delay(3); // Attesa per non saturare il bus I2C
    }

    // Salva gli offset medi
    gyro_x_cal = (float)gx_sum / num_samples;
    gyro_y_cal = (float)gy_sum / num_samples;
    gyro_z_cal = (float)gz_sum / num_samples;

    // Pre-carica i filtri LPF con l'ultima lettura
    ax_filt = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    ay_filt = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    az_filt = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    // Azzera gli angoli di partenza (l'errore lo sottrarremo a runtime)
    pitch = 0.0f;
    roll = 0.0f;
  }
}

/**
 * @brief Acquisizione, Filtro LPF (Accelerometro) e Filtro Complementare
 */
void MPU6050_Read_Filter_Compute(void) {
  uint8_t rec_data[14];

  // Lettura burst da 14 byte a partire dall'indirizzo base dei dati (0x3B)
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, rec_data, 14, 100);

  // Parsing registri grezzi
  int16_t accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
  int16_t accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
  int16_t accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);
  int16_t gyro_x_raw = (int16_t)(rec_data[8] << 8 | rec_data[9]);
  int16_t gyro_y_raw = (int16_t)(rec_data[10] << 8 | rec_data[11]);
  int16_t gyro_z_raw = (int16_t)(rec_data[12] << 8 | rec_data[13]);

  // Applicazione LPF Esponenziale (EMA) sui vettori accelerometrici
  ax_filt = LPF_ALPHA * accel_x_raw + (1.0f - LPF_ALPHA) * ax_filt;
  ay_filt = LPF_ALPHA * accel_y_raw + (1.0f - LPF_ALPHA) * ay_filt;
  az_filt = LPF_ALPHA * accel_z_raw + (1.0f - LPF_ALPHA) * az_filt;

  // Estrazione angoli grezzi accelerometro e SOTTRAZIONE BIAS (Offset)
  float acc_pitch =
      (atan2f(-ax_filt, sqrtf(ay_filt * ay_filt + az_filt * az_filt)) *
       RAD_TO_DEG) -
      acc_pitch_cal;
  float acc_roll =
      (atan2f(ay_filt, sqrtf(ax_filt * ax_filt + az_filt * az_filt)) *
       RAD_TO_DEG) -
      acc_roll_cal;

  // Aggiornamento tempo per il controllo del loop principale
  uint32_t current_time = HAL_GetTick();
  last_time = current_time;

  // Hardcode di dt_actual a 4ms per annullare il jitter (HAL_GetTick oscilla e
  // destabilizza la Derivata)
  dt_actual = 0.004f;

  // Velocita' angolari in Gradi/Secondo con offset rimosso
  // Gyro X ruota attorno a X -> ROLL ; Gyro Y ruota attorno a Y -> PITCH
  // Gyro Z ruota attorno a Z -> YAW
  gyro_rate_roll = GYRO_ROLL_SIGN * (gyro_x_raw - gyro_x_cal) / GYRO_SCALE;
  gyro_rate_pitch = GYRO_PITCH_SIGN * (gyro_y_raw - gyro_y_cal) / GYRO_SCALE;
  gyro_rate_yaw = GYRO_YAW_SIGN * (gyro_z_raw - gyro_z_cal) / GYRO_SCALE;

  // Filtro Complementare
  roll = COMP_ALPHA * (roll + gyro_rate_roll * dt_actual) +
         (1.0f - COMP_ALPHA) * acc_roll;
  pitch = COMP_ALPHA * (pitch + gyro_rate_pitch * dt_actual) +
          (1.0f - COMP_ALPHA) * acc_pitch;
}

/**
 * @brief Spegne subito tutti i motori. Chiamabile da ISR e da main.
 */
void Motors_Kill(void) {
  motors_enabled = 0;
  current_throttle = 1000;
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

/**
 * @brief Interpreta una riga di comando ricevuta dall'ESP32
 */
void Process_Command(char *line) {
  if (strncmp(line, "CMD:", 4) == 0) {
    if (line[4] == 't') {
      if (!motors_enabled) { // un secondo 't' NON rinnova il timer degli 8 s
        motor_start_tick = HAL_GetTick();
        motors_enabled = 1;
      }
    } else if (line[4] == 's' || line[4] == 'e') {
      Motors_Kill(); // (gia' gestito in ISR, qui per sicurezza)
    }
  } else if (strncmp(line, "THR:", 4) == 0) {
    int esp32_throttle = atoi(&line[4]);
    if (esp32_throttle >= 0 && esp32_throttle <= 255) {
      current_throttle = 1000 + ((esp32_throttle * 1000) / 255);
    }
  } else if (strncmp(line, "PID:", 4) == 0) {
    float v[6];
    if (sscanf(&line[4], "%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4],
               &v[5]) == 6) {
      pid_pitch_p = v[0];
      pid_pitch_i = v[1];
      pid_pitch_d = v[2];
      pid_roll_p = v[3];
      pid_roll_i = v[4];
      pid_roll_d = v[5];
    }
  } else if (strncmp(line, "YAW:", 4) == 0) {
    float y[3];
    if (sscanf(&line[4], "%f,%f,%f", &y[0], &y[1], &y[2]) == 3) {
      pid_yaw_p = y[0];
      pid_yaw_i = y[1];
      pid_yaw_d = y[2];
    }
  } else if (strncmp(line, "DIR:", 4) == 0) {
    int p, r, y;
    if (sscanf(&line[4], "%d,%d,%d", &p, &r, &y) == 3) {
      if (p > 100)
        p = 100;
      if (p < -100)
        p = -100;
      if (r > 100)
        r = 100;
      if (r < -100)
        r = -100;
      if (y > 1)
        y = 1;
      if (y < -1)
        y = -1;
      sp_pitch = DIR_PITCH_SIGN * ((float)p / 100.0f) * DIR_TILT_DEG;
      sp_roll = DIR_ROLL_SIGN * ((float)r / 100.0f) * DIR_TILT_DEG;
      sp_yaw_rate = DIR_YAW_SIGN * (float)y * DIR_YAW_RATE_DPS;
      dir_last_tick = HAL_GetTick();
    }
  } else if (strncmp(line, "TRIM:", 5) == 0) {
    float k[4];
    if (sscanf(&line[5], "%f,%f,%f,%f", &k[0], &k[1], &k[2], &k[3]) == 4) {
      for (int i = 0; i < 4; i++) {
        if (k[i] < TRIM_MIN)
          k[i] = TRIM_MIN;
        if (k[i] > TRIM_MAX)
          k[i] = TRIM_MAX;
        motor_trim[i] = k[i];
      }
    }
  }
}

/**
 * @brief Computazione PID e Miscelazione base motori (X-Configuration)
 */
void PID_ComputeAndMix(void) {
  // Safety check. Se motori disarmati O throttle basso: motori a 1000 us.
  // (prima in questo caso i motori restavano all'ultimo valore scritto!)
  if (!motors_enabled || current_throttle < 1050) {
    integral_pitch = 0;
    integral_roll = 0;
    integral_yaw = 0;
    prev_gyro_rate_yaw = gyro_rate_yaw;
    yaw_d_filt = 0;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
    return;
  }

  // Setpoint: 0 per hovering, oppure inclinazione fissa da comando web
  error_pitch = sp_pitch - pitch;
  error_roll = sp_roll - roll;

  // Componente Integrale
  integral_pitch += error_pitch * dt_actual;
  integral_roll += error_roll * dt_actual;

  // Componente Derivativa: d(setpoint - angolo)/dt = -velocita_angolare.
  float deriv_pitch = -gyro_rate_pitch;
  float deriv_roll = -gyro_rate_roll;

  // Uscita PID
  float pid_pitch_out = (pid_pitch_p * error_pitch) +
                        (pid_pitch_i * integral_pitch) +
                        (pid_pitch_d * deriv_pitch);
  float pid_roll_out = (pid_roll_p * error_roll) +
                       (pid_roll_i * integral_roll) + (pid_roll_d * deriv_roll);

  // ---------------- YAW ----------------
  // Il MPU6050 non ha magnetometro: lo yaw e' controllato sulla velocita'
  // angolare (gyro Z), setpoint 0 deg/s (o velocita' da comando web).
  // L'integrale dell'errore di velocita' equivale a un "heading hold" a
  // breve termine (deriva lenta del gyro).
  error_yaw = sp_yaw_rate - gyro_rate_yaw;

  integral_yaw += error_yaw * dt_actual;
  if (integral_yaw > YAW_I_MAX)
    integral_yaw = YAW_I_MAX;
  if (integral_yaw < -YAW_I_MAX)
    integral_yaw = -YAW_I_MAX;

  // Derivata dell'errore = -(variazione della velocita' angolare), filtrata
  float rate_d = (gyro_rate_yaw - prev_gyro_rate_yaw) / dt_actual;
  prev_gyro_rate_yaw = gyro_rate_yaw;
  yaw_d_filt = YAW_D_LPF * rate_d + (1.0f - YAW_D_LPF) * yaw_d_filt;
  float deriv_yaw = -yaw_d_filt;

  float pid_yaw_out = (pid_yaw_p * error_yaw) + (pid_yaw_i * integral_yaw) +
                      (pid_yaw_d * deriv_yaw);
  if (pid_yaw_out > YAW_OUT_MAX)
    pid_yaw_out = YAW_OUT_MAX;
  if (pid_yaw_out < -YAW_OUT_MAX)
    pid_yaw_out = -YAW_OUT_MAX;

  float yaw_mix = YAW_MIX_SIGN * pid_yaw_out;

  // TRIM: il moltiplicatore agisce sulla parte di throttle sopra il minimo
  // (1000 us), quindi il motore debole riceve piu' spinta in proporzione.
  float thr_above = (float)current_throttle - 1000.0f;
  float t1 = 1000.0f + thr_above * motor_trim[0]; // Front Left
  float t2 = 1000.0f + thr_above * motor_trim[1]; // Front Right
  float t3 = 1000.0f + thr_above * motor_trim[2]; // Back Right
  float t4 = 1000.0f + thr_above * motor_trim[3]; // Back Left

  // Motor Mixing X-Config (adattare i segni a seconda dell'orientamento
  // dell'IMU). Yaw: le due diagonali (M1+M3 e M2+M4) girano in versi opposti,
  // quindi la correzione yaw si somma su una diagonale e si sottrae
  // sull'altra.
  int16_t m1 =
      (int16_t)(t1 + pid_pitch_out + pid_roll_out + yaw_mix); // Front Left
  int16_t m2 =
      (int16_t)(t2 + pid_pitch_out - pid_roll_out - yaw_mix); // Front Right
  int16_t m3 =
      (int16_t)(t3 - pid_pitch_out - pid_roll_out + yaw_mix); // Back Right
  int16_t m4 =
      (int16_t)(t4 - pid_pitch_out + pid_roll_out - yaw_mix); // Back Left

  // Saturazione segnali (Anti-Windup Meccanico)
  m1 = (m1 > 2000) ? 2000 : (m1 < 1000 ? 1000 : m1);
  m2 = (m2 > 2000) ? 2000 : (m2 < 1000 ? 1000 : m2);
  m3 = (m3 > 2000) ? 2000 : (m3 < 1000 ? 1000 : m3);
  m4 = (m4 > 2000) ? 2000 : (m4 < 1000 ? 1000 : m4);

  // Applicazione al registro PWM, protetta: se un EMERGENCY STOP arriva (ISR)
  // mentre stavamo calcolando, NON deve essere sovrascritto con valori vecchi.
  __disable_irq();
  if (motors_enabled) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4);
  }
  __enable_irq();
}

/**
 * @brief Callback Interrupt UART RX
 *        Stop/emergency ("CMD:s" / "CMD:e") vengono eseguiti QUI, subito,
 *        senza aspettare il main loop. Gli altri comandi vanno in coda.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    char c = (char)esp32_rx_byte;

    if (c == '\n') {
      rx_line[rx_index] = '\0';
      rx_index = 0;

      if (rx_overflow) {
        rx_overflow = 0; // riga troppo lunga: scartata
      } else if (strstr(rx_line, "CMD:e") != NULL ||
                 strstr(rx_line, "CMD:s") != NULL) {
        Motors_Kill();
      } else {
        uint8_t next = (rx_q_head + 1) % RX_QUEUE_LEN;
        if (next != rx_q_tail) { // se la coda e' piena scarta
          strcpy(rx_queue[rx_q_head], rx_line);
          rx_q_head = next;
        }
      }
    } else if (c != '\r') {
      if (rx_index < (RX_LINE_LEN - 1)) {
        rx_line[rx_index++] = c;
      } else {
        rx_overflow = 1;
      }
    }
    HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
  }
}

/**
 * @brief Errori UART (overrun, framing...): senza questo la ricezione si
 *        ferma e l'emergency stop non arriverebbe piu'.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF | UART_CLEAR_FEF);
    rx_index = 0;
    rx_overflow = 0;
    HAL_UART_Receive_IT(huart, &esp32_rx_byte, 1);
  }
}

/**
 * @brief Callback Interrupt UART TX completata
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    telemetry_tx_busy = 0;
  }
}
/* USER CODE END 4 */

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC->CKGAENR = 0xE003FFFF;
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 */
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909FCE; // Configurazione per 400kHz (Fast Mode)
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM1 Initialization Function
 */
static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499; // 400Hz per gli ESC
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief USART2 Initialization Function
 */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin | STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* MPU Configuration */
void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif