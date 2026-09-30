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
#define DEG_TO_RAD 0.01745329252f
#define LPF_ALPHA                                                              \
  0.1f // Fattore filtro Passa-Basso (aumentato per ridurre phase lag)
#define GYRO_SCALE 65.5f       // Scala giroscopio per +/- 500 deg/s
#define ACC_LSB_PER_G 16384.0f // Sensibilita' accelerometro per +/- 2g

// Parametri filtro di Mahony (quaternioni)
// MAHONY_KP: [rad/s] velocita' con cui l'accelerometro corregge il giroscopio.
//   Costante di tempo ~ 1/Kp. Il vecchio complementare (alpha=0.999 a 250Hz)
//   equivale a Kp ~ 0.25. Piu' alto = segue prima l'accelerometro ma piu'
//   sensibile alle vibrazioni; piu' basso = piu' fiducia nel giroscopio.
// MAHONY_KI: stima del bias residuo del giroscopio (0 = disattivato).
// MAHONY_INT_LIMIT: limite anti-windup del bias stimato [rad/s].
// ACC_TRUST_BAND: fiducia nell'accelerometro = 1 quando |a| = 1g e scende a 0
//   quando |a| si discosta da 1g di questo valore (in g). Durante manovre
//   brusche/vibrazioni l'accelerometro viene ignorato e vince il giroscopio.
#define MAHONY_KP 0.5f
#define MAHONY_KI 0.02f
#define MAHONY_INT_LIMIT 0.05f
#define ACC_TRUST_BAND 0.30f

// Mapping giroscopio -> angolo
#define GYRO_ROLL_SIGN 1.0f  // gyro X -> roll
#define GYRO_PITCH_SIGN 1.0f // gyro Y -> pitch
#define GYRO_YAW_SIGN 1.0f   // gyro Z -> yaw
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
float gyro_x_cal = 0.0f, gyro_y_cal = 0.0f;
float acc_pitch_cal = -4.79f, acc_roll_cal = -0.89f; // -4.79   -0.89
float gyro_rate_pitch = 0.0f;
float gyro_rate_roll = 0.0f;

// Variabili Filtro di Mahony (quaternioni)
float gyro_z_cal = 0.0f;                          // Offset giroscopio Z
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternione (body->world)
float mahony_int_x = 0.0f, mahony_int_y = 0.0f,
      mahony_int_z = 0.0f; // Bias gyro stimato [rad/s]
float yaw = 0.0f; // Solo informativo: senza magnetometro deriva lentamente
float mount_R[3][3] = {
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // Correzione montaggio IMU

// Variabili UART ESP32
uint8_t esp32_rx_byte;
char rx_buffer[64];
uint8_t rx_index = 0;
volatile uint8_t packet_ready = 0;
char telemetry_tx_buffer[192];
volatile uint8_t telemetry_tx_busy = 0;

// Stato di sistema
uint32_t current_throttle = 1000;
float pid_pitch_p = 2.16f, pid_pitch_i = 0.7f, pid_pitch_d = 0.8f;
float pid_roll_p = 2.16f, pid_roll_i = 0.9f, pid_roll_d = 0.85f;
volatile uint8_t motors_enabled = 0;
uint32_t motor_start_tick = 0;

// Variabili PID
float error_pitch = 0, integral_pitch = 0;
float error_roll = 0, integral_roll = 0;
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
void IMU_BuildMountingMatrix(void);
void IMU_ApplyMounting(float v[3]);
void Quat_InitFromAccel(float ax, float ay, float az);
void Quat_ToEuler(void);
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az,
                   float dt);
void PID_ComputeAndMix(void);
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

    // ==========================================
    // CONTROL LOOP: Eseguito ogni 4ms (250Hz)
    // ==========================================
    if (current_time - last_time >= 4) {
      // --- INIZIO NUOVO BLOCCO TIMEOUT ---
      if (motors_enabled && (current_time - motor_start_tick >= 10000)) {
        motors_enabled = 0; // Disarma il sistema

        // Azzera istantaneamente i motori
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
      }
      // --- FINE NUOVO BLOCCO TIMEOUT ---

      // 1. Acquisizione I2C e applicazione filtri Digitali (Passa Basso +
      // Mahony su quaternioni)
      MPU6050_Read_Filter_Compute();

      // 2. Calcolo errori, computazione PID e miscelazione motori
      PID_ComputeAndMix();

      // 3. Telemetria verso ESP32 (Eseguita ogni 40ms -> 25Hz per evitare
      // saturazione UART)
      static uint8_t telemetry_counter = 0;
      if (++telemetry_counter >= 10) {
        if (!telemetry_tx_busy) {
          int len =
              snprintf(telemetry_tx_buffer, sizeof(telemetry_tx_buffer),
                       "ANG:%.2f,%.2f,"
                       "pid_pitch_p:%.3f,pid_pitch_i:%.3f,pid_pitch_d:%.3f,"
                       "pid_roll_p:%.3f,pid_roll_i:%.3f,pid_roll_d:%.3f\n",
                       pitch, roll, pid_pitch_p, pid_pitch_i, pid_pitch_d,
                       pid_roll_p, pid_roll_i, pid_roll_d);

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
    // PARSING DEI COMANDI ESP32 (Asincrono)
    // ==========================================
    if (packet_ready) {
      char local_buffer[64];

      __disable_irq();
      strncpy(local_buffer, rx_buffer, sizeof(local_buffer));
      memset(rx_buffer, 0, sizeof(rx_buffer));
      rx_index = 0;
      packet_ready = 0;
      __enable_irq();

      // Comandi Generici
      if (strncmp(local_buffer, "CMD:", 4) == 0) {
        if (local_buffer[4] == 't') {
          motors_enabled = 1;
          motor_start_tick = HAL_GetTick();
        } else if (local_buffer[4] == 's') {
          motors_enabled = 0;
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
        }
      }
      // Acceleratore
      else if (strncmp(local_buffer, "THR:", 4) == 0) {
        int esp32_throttle = atoi(&local_buffer[4]);
        if (esp32_throttle >= 0 && esp32_throttle <= 255) {
          current_throttle = 1000 + ((esp32_throttle * 1000) / 255);
        }
      }
      // Parametri PID
      else if (strncmp(local_buffer, "PID:", 4) == 0) {
        sscanf(&local_buffer[4], "%f,%f,%f,%f,%f,%f", &pid_pitch_p,
               &pid_pitch_i, &pid_pitch_d, &pid_roll_p, &pid_roll_i,
               &pid_roll_d);
      }
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

    // Matrice di correzione del montaggio (da acc_pitch_cal / acc_roll_cal)
    IMU_BuildMountingMatrix();

    // Inizializza il quaternione dall'accelerometro: il filtro parte gia'
    // allineato all'assetto reale, senza transitorio di convergenza.
    Quat_InitFromAccel(ax_filt, ay_filt, az_filt);
  }
}

/* ===================== BEGIN QUAT FILTER (math) ===================== */

/**
 * @brief Costruisce la matrice di correzione del montaggio dell'IMU.
 *        Sostituisce la vecchia sottrazione di acc_pitch_cal/acc_roll_cal
 *        sugli angoli: qui la correzione e' una rotazione applicata ai
 *        vettori grezzi (accelerometro e giroscopio), quindi resta corretta
 *        anche a inclinazioni elevate. R = Ry(pitch_cal) * Rx(roll_cal).
 */
void IMU_BuildMountingMatrix(void) {
  float th = acc_pitch_cal * DEG_TO_RAD;
  float ph = acc_roll_cal * DEG_TO_RAD;
  float cth = cosf(th), sth = sinf(th);
  float cph = cosf(ph), sph = sinf(ph);

  mount_R[0][0] = cth;
  mount_R[0][1] = sth * sph;
  mount_R[0][2] = sth * cph;
  mount_R[1][0] = 0.0f;
  mount_R[1][1] = cph;
  mount_R[1][2] = -sph;
  mount_R[2][0] = -sth;
  mount_R[2][1] = cth * sph;
  mount_R[2][2] = cth * cph;
}

/**
 * @brief Applica in-place la correzione di montaggio a un vettore 3D.
 */
void IMU_ApplyMounting(float v[3]) {
  float x = v[0], y = v[1], z = v[2];
  v[0] = mount_R[0][0] * x + mount_R[0][1] * y + mount_R[0][2] * z;
  v[1] = mount_R[1][0] * x + mount_R[1][1] * y + mount_R[1][2] * z;
  v[2] = mount_R[2][0] * x + mount_R[2][1] * y + mount_R[2][2] * z;
}

/**
 * @brief Converte il quaternione in angoli di Eulero (gradi) -> roll, pitch,
 *        yaw. Stesse convenzioni di segno del codice precedente.
 */
void Quat_ToEuler(void) {
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f)
    sinp = 1.0f;
  if (sinp < -1.0f)
    sinp = -1.0f;

  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
         RAD_TO_DEG;
  pitch = asinf(sinp) * RAD_TO_DEG;
  yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
        RAD_TO_DEG;
}

/**
 * @brief Inizializza il quaternione dall'accelerometro (yaw = 0).
 *        Ingresso: accelerometro grezzo/filtrato (prima della correzione di
 *        montaggio, qualsiasi unita').
 */
void Quat_InitFromAccel(float ax, float ay, float az) {
  float a[3] = {ax, ay, az};
  IMU_ApplyMounting(a);

  float r = atan2f(a[1], a[2]);
  float p = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));

  float cr = cosf(0.5f * r), sr = sinf(0.5f * r);
  float cp = cosf(0.5f * p), sp = sinf(0.5f * p);

  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;

  mahony_int_x = 0.0f;
  mahony_int_y = 0.0f;
  mahony_int_z = 0.0f;

  Quat_ToEuler();
}

/**
 * @brief Filtro di Mahony (PI su quaternione) con fiducia adattiva
 *        sull'accelerometro.
 * @param gx,gy,gz Velocita' angolari [rad/s], bias statico gia' rimosso
 * @param ax,ay,az Accelerometro (qualsiasi unita', in LSB per ACC_LSB_PER_G)
 * @param dt       Passo di integrazione [s]
 */
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az,
                   float dt) {
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;
  float trust = 0.0f;

  float a_norm = sqrtf(ax * ax + ay * ay + az * az);

  if (a_norm > 1.0f) {
    // Fiducia: 1 se |a| = 1g, 0 se |a| si discosta di ACC_TRUST_BAND g
    trust = 1.0f - fabsf(a_norm / ACC_LSB_PER_G - 1.0f) / ACC_TRUST_BAND;
    if (trust < 0.0f)
      trust = 0.0f;

    if (trust > 0.0f) {
      float inv = 1.0f / a_norm;
      ax *= inv;
      ay *= inv;
      az *= inv;

      // Direzione della gravita' stimata dal quaternione (frame body)
      float vx = 2.0f * (q1 * q3 - q0 * q2);
      float vy = 2.0f * (q0 * q1 + q2 * q3);
      float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

      // Errore = prodotto vettoriale tra gravita' misurata e stimata
      ex = ay * vz - az * vy;
      ey = az * vx - ax * vz;
      ez = ax * vy - ay * vx;

      // Termine integrale (stima del bias del giroscopio) con anti-windup
      if (MAHONY_KI > 0.0f) {
        mahony_int_x += MAHONY_KI * trust * ex * dt;
        mahony_int_y += MAHONY_KI * trust * ey * dt;
        mahony_int_z += MAHONY_KI * trust * ez * dt;

        if (mahony_int_x > MAHONY_INT_LIMIT)
          mahony_int_x = MAHONY_INT_LIMIT;
        if (mahony_int_x < -MAHONY_INT_LIMIT)
          mahony_int_x = -MAHONY_INT_LIMIT;
        if (mahony_int_y > MAHONY_INT_LIMIT)
          mahony_int_y = MAHONY_INT_LIMIT;
        if (mahony_int_y < -MAHONY_INT_LIMIT)
          mahony_int_y = -MAHONY_INT_LIMIT;
        if (mahony_int_z > MAHONY_INT_LIMIT)
          mahony_int_z = MAHONY_INT_LIMIT;
        if (mahony_int_z < -MAHONY_INT_LIMIT)
          mahony_int_z = -MAHONY_INT_LIMIT;
      }
    }
  }

  // Giroscopio corretto: rimuove il bias stimato + termine proporzionale
  gx += mahony_int_x + MAHONY_KP * trust * ex;
  gy += mahony_int_y + MAHONY_KP * trust * ey;
  gz += mahony_int_z + MAHONY_KP * trust * ez;

  // Integrazione del quaternione: q_dot = 0.5 * q (x) (0, gx, gy, gz)
  float hdt = 0.5f * dt;
  float qa = q0, qb = q1, qc = q2, qd = q3;
  q0 = qa + (-qb * gx - qc * gy - qd * gz) * hdt;
  q1 = qb + (qa * gx + qc * gz - qd * gy) * hdt;
  q2 = qc + (qa * gy - qb * gz + qd * gx) * hdt;
  q3 = qd + (qa * gz + qb * gy - qc * gx) * hdt;

  // Normalizzazione
  float qn = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= qn;
  q1 *= qn;
  q2 *= qn;
  q3 *= qn;
}

/* ====================== END QUAT FILTER (math) ====================== */

/**
 * @brief Acquisizione, Filtro LPF (Accelerometro) e Filtro di Mahony
 *        (quaternioni)
 */
void MPU6050_Read_Filter_Compute(void) {
  uint8_t rec_data[14];

  // Lettura burst da 14 byte a partire dall'indirizzo base dei dati (0x3B)
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, rec_data, 14, 100) !=
      HAL_OK) {
    // Lettura fallita: non usare dati non validi, mantieni la cadenza del loop
    last_time = HAL_GetTick();
    return;
  }

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

  // Aggiornamento tempo per il controllo del loop principale
  uint32_t current_time = HAL_GetTick();
  last_time = current_time;

  // Hardcode di dt_actual a 4ms per annullare il jitter (HAL_GetTick oscilla e
  // destabilizza la Derivata)
  dt_actual = 0.004f;

  // Velocita' angolari in Gradi/Secondo con offset rimosso
  // Gyro X ruota attorno a X -> ROLL ; Gyro Y ruota attorno a Y -> PITCH
  float gyro[3];
  gyro[0] = GYRO_ROLL_SIGN * (gyro_x_raw - gyro_x_cal) / GYRO_SCALE;
  gyro[1] = GYRO_PITCH_SIGN * (gyro_y_raw - gyro_y_cal) / GYRO_SCALE;
  gyro[2] = GYRO_YAW_SIGN * (gyro_z_raw - gyro_z_cal) / GYRO_SCALE;

  float acc[3] = {ax_filt, ay_filt, az_filt};

  // Correzione del montaggio dell'IMU (sostituisce la sottrazione degli
  // offset angolari sull'accelerometro)
  IMU_ApplyMounting(gyro);
  IMU_ApplyMounting(acc);

  // Rates in deg/s usati dal termine D del PID
  gyro_rate_roll = gyro[0];
  gyro_rate_pitch = gyro[1];

  // Filtro di Mahony: gyro in rad/s, accelerometro in LSB
  Mahony_Update(gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD,
                gyro[2] * DEG_TO_RAD, acc[0], acc[1], acc[2], dt_actual);

  // Angoli di assetto (gradi) per PID e telemetria
  Quat_ToEuler();
}

/**
 * @brief Computazione PID e Miscelazione base motori (X-Configuration)
 */
void PID_ComputeAndMix(void) {
  // Safety check
  if (!motors_enabled || current_throttle < 1050) {
    integral_pitch = 0;
    integral_roll = 0;
    return;
  }

  // Setpoint 0 per hovering
  error_pitch = 0 - pitch;
  error_roll = 0 - roll;

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

  // Motor Mixing X-Config (adattare i segni a seconda dell'orientamento
  // dell'IMU)
  int16_t m1 = current_throttle + pid_pitch_out + pid_roll_out; // Front Left
  int16_t m2 = current_throttle + pid_pitch_out - pid_roll_out; // Front Right
  int16_t m3 = current_throttle - pid_pitch_out - pid_roll_out; // Back Right
  int16_t m4 = current_throttle - pid_pitch_out + pid_roll_out; // Back Left

  // Saturazione segnali (Anti-Windup Meccanico)
  m1 = (m1 > 2000) ? 2000 : (m1 < 1000 ? 1000 : m1);
  m2 = (m2 > 2000) ? 2000 : (m2 < 1000 ? 1000 : m2);
  m3 = (m3 > 2000) ? 2000 : (m3 < 1000 ? 1000 : m3);
  m4 = (m4 > 2000) ? 2000 : (m4 < 1000 ? 1000 : m4);

  // Applicazione al registro PWM
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, m3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, m4);
}

/**
 * @brief Callback Interrupt UART RX
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    if (esp32_rx_byte == '\n') {
      rx_buffer[rx_index] = '\0';
      packet_ready = 1;
    } else if (esp32_rx_byte != '\r') {
      if (rx_index < (sizeof(rx_buffer) - 1)) {
        rx_buffer[rx_index++] = esp32_rx_byte;
      }
    }
    HAL_UART_Receive_IT(&huart2, &esp32_rx_byte, 1);
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
