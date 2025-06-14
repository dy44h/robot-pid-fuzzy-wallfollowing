#include <Wire.h>
#include <MPU6050.h> // Asumsi menggunakan library MPU6050 yang kompatibel dengan API Jeff Rowberg
#include <math.h>

// Definisi Pin (sesuai permintaan, tidak diubah)
#define in1 27
#define in2 26
#define in3 25
#define in4 33
#define enA 14
#define enB 32
#define trigPin 5  // Tidak akan digunakan
#define echoPin 18 // Tidak akan digunakan

MPU6050 accelgyro;
float Qacc = 0.002, Qgyro = 0.003, R_kalman = 0.03;
float kalmanAngle = 0;

// --- Parameter PID ---
float Kp, Ki, Kd;
float error, prevError = 0, integralError = 0, derivativeError;
float setpoint = -2.455;
float pidOutput;

// --- Parameter untuk Fuzzy Tuner PID ---
// Singleton values untuk Kp (contoh, perlu tuning)
float Kp_S = 110.0, Kp_M = 140.0, Kp_L = 200.0;
// Singleton values untuk Kd (contoh, perlu tuning)
float Kd_S = 0.087, Kd_M = 0.05, Kd_L = 1.0; // Nilai Kd_L mungkin perlu dinaikkan
// Ki akan dibuat konstan kecil
float Ki_fixed = 0.06; // Contoh, coba kurangi atau nolkan jika ada masalah

// Batas untuk input Fuzzy (error sudut dan kecepatan sudut)
// Error sudut (derajat)
float err_nf_limit = -15.0; // Negatif Jauh
float err_ns_limit = -5.0;  // Negatif Dekat
float err_ze_limit = 0.0;   // Nol
float err_ps_limit = 5.0;   // Positif Dekat
float err_pf_limit = 15.0;  // Positif Jauh

// Kecepatan sudut (dps - degrees per second)
float derr_n_limit = -80.0; // Negatif
float derr_z_limit = 0.0;   // Nol
float derr_p_limit = 80.0;  // Positif

void setup() {
  Serial.begin(115200);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);

  Wire.begin();
  accelgyro.initialize();
  if (!accelgyro.testConnection()) {
    Serial.println("Koneksi MPU6050 gagal!");
    while (1);
  }
  delay(500);
  calibrateMPU();
  Serial.println("Robot Siap.");
}

void loop() {
  float pitch = getPitchAngle();
  float gyroY_dps = getGyroRateY();
  kalmanAngle = updateKalman(pitch, gyroY_dps);

  error = setpoint - kalmanAngle;
  derivativeError = -gyroY_dps; 
  integralError += error * 0.01; 
  if (integralError > 100) integralError = 100;
  else if (integralError < -100) integralError = -100;

  getFuzzyPIDGains(error, gyroY_dps, Kp, Kd);
  Ki = Ki_fixed;

  pidOutput = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);

  pidOutput = constrain(pidOutput, -255, 255);
  if (abs(pidOutput) < 40 && abs(error) < 2.0) {
      pidOutput = 0;
      integralError = 0; 
  }
  
  kontrolMotor(pidOutput, pidOutput);

  Serial.print("Angle:"); Serial.print(kalmanAngle); Serial.print(",");
  Serial.print("Err:"); Serial.print(error); Serial.print(",");
  Serial.print("GyroY:"); Serial.print(gyroY_dps); Serial.print(",");
  Serial.print("Kp:"); Serial.print(Kp); Serial.print(",");
  Serial.print("Ki:"); Serial.print(Ki); Serial.print(",");
  Serial.print("Kd:"); Serial.print(Kd); Serial.print(",");
  Serial.print("PID:"); Serial.println(pidOutput);

  prevError = error;
  delay(10);
}

// ==== MPU6050 & Kalman Filter ====
float getPitchAngle() {
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  return atan2(-ax, az) * 180 / PI;
}

float getGyroRateY() {
  int16_t gx, gy, gz;
  accelgyro.getRotation(&gx, &gy, &gz);
  return gy / 131.0; 
}

void calibrateMPU() {
  Serial.println("Kalibrasi MPU (pemanasan)...");
  long gyro_y_sum = 0;
  for (int i = 0; i < 2000; i++) {
    int16_t gx, gy, gz;
    accelgyro.getRotation(&gx, &gy, &gz);
    gyro_y_sum += gy;
    delay(2);
  }
  Serial.println("Kalibrasi MPU selesai.");
}

float updateKalman(float newAngle, float newRate) {
  static float angle = 0.0;
  static float bias = 0.0; 
  static float P[2][2] = {{1, 0}, {0, 1}};

  float dt = 0.01; 

  angle += dt * (newRate - bias);

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Qacc);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Qgyro * dt;

  float S = P[0][0] + R_kalman; 
  float K[2]; 
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle; 
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

// ==== MOTOR ====
void kontrolMotor(int pwmKiri, int pwmKanan) {
  if (pwmKiri > 0) {
    analogWrite(enA, pwmKiri);
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else if (pwmKiri < 0) {
    analogWrite(enA, -pwmKiri);
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else {
    analogWrite(enA, 0);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }

  if (pwmKanan > 0) {
    analogWrite(enB, pwmKanan);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else if (pwmKanan < 0) {
    analogWrite(enB, -pwmKanan);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  } else {
    analogWrite(enB, 0);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }
}

// ==== FUZZY LOGIC untuk Tuning PID Gains (Kp, Kd) ====
float fuzzy_triangular(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x == b) return 1.0;
  if (x > a && x < b) return (x - a) / (b - a);
  if (x > b && x < c) return (c - x) / (c - b);
  return 0.0;
}

float fuzzy_trap_left(float x, float a, float b) {
    if (x <= a) return 1.0;
    if (x > a && x < b) return (b - x) / (b - a);
    return 0.0;
}

float fuzzy_trap_right(float x, float a, float b) {
    if (x >= b) return 1.0;
    if (x > a && x < b) return (x - a) / (b - a);
    return 0.0;
}

void getFuzzyPIDGains(float error_val, float d_error_val, float &outKp, float &outKd) {
  float mu_err_NB = fuzzy_trap_left(error_val, err_nf_limit, err_ns_limit);
  float mu_err_NS = fuzzy_triangular(error_val, err_nf_limit, err_ns_limit, err_ze_limit);
  float mu_err_ZE = fuzzy_triangular(error_val, err_ns_limit, err_ze_limit, err_ps_limit);
  float mu_err_PS = fuzzy_triangular(error_val, err_ze_limit, err_ps_limit, err_pf_limit);
  float mu_err_PB = fuzzy_trap_right(error_val, err_ps_limit, err_pf_limit);

  float mu_derr_N = fuzzy_trap_left(d_error_val, derr_n_limit, derr_z_limit);
  float mu_derr_Z = fuzzy_triangular(d_error_val, derr_n_limit, derr_z_limit, derr_p_limit);
  float mu_derr_P = fuzzy_trap_right(d_error_val, derr_z_limit, derr_p_limit);

  // Aturan untuk Kp (Error utama, d_error sbg modulasi)
  // Baris: error (NB, NS, ZE, PS, PB), Kolom: d_error (N, Z, P)
  // NB: kalmanAngle positif (error negatif) - Miring ke DEPAN
  // PB: kalmanAngle negatif (error positif) - Miring ke BELAKANG
  float Kp_rules_output[5][3] = {
    // d_error:     N     Z     P
    /* Error NB */{Kp_L, Kp_L, Kp_M},
    /* Error NS */{Kp_L, Kp_M, Kp_S},
    /* Error ZE */{Kp_M, Kp_S, Kp_M},
    /* Error PS */{Kp_S, Kp_M, Kp_L},
    /* Error PB */{Kp_M, Kp_L, Kp_L} 
  };

  // ====================================================================
  // === PERUBAHAN UTAMA ADA DI TABEL ATURAN Kd DI BAWAH INI ===
  // ====================================================================
  // Revisi Aturan untuk Kd
  // Baris: error (NB, NS, ZE, PS, PB), Kolom: d_error (N, Z, P)
  // NB: Miring DEPAN (error negatif)
  // PB: Miring BELAKANG (error positif)
  // d_error N: jatuh/bergerak ke BELAKANG
  // d_error P: jatuh/bergerak ke DEPAN
  float Kd_rules_output[5][3] = {
    // d_error:     N     Z     P
    /* Error NB */{Kd_L, Kd_M, Kd_L}, // Jika miring DEPAN: jatuh ke BELAKANG (L), diam (M), jatuh ke DEPAN (L)
    /* Error NS */{Kd_L, Kd_M, Kd_L},
    /* Error ZE */{Kd_M, Kd_S, Kd_M}, // Jika TEGAK: d_error N (M), Z (S), P (M)
    /* Error PS */{Kd_L, Kd_M, Kd_L},
    /* Error PB */{Kd_L, Kd_M, Kd_L}  // Jika miring BELAKANG: jatuh ke BELAKANG (L), diam (M), jatuh ke DEPAN (L)
  };
  // ====================================================================
  
  float mu_err[] = {mu_err_NB, mu_err_NS, mu_err_ZE, mu_err_PS, mu_err_PB};
  float mu_derr[] = {mu_derr_N, mu_derr_Z, mu_derr_P};

  float total_mu_strength_kp = 0.0;
  float weighted_sum_kp = 0.0;
  float total_mu_strength_kd = 0.0;
  float weighted_sum_kd = 0.0;

  for (int i = 0; i < 5; i++) {
    if (mu_err[i] == 0) continue;
    for (int j = 0; j < 3; j++) {
      if (mu_derr[j] == 0) continue;

      float rule_strength = min(mu_err[i], mu_derr[j]);

      if (rule_strength > 0) {
        weighted_sum_kp += rule_strength * Kp_rules_output[i][j];
        total_mu_strength_kp += rule_strength;
        weighted_sum_kd += rule_strength * Kd_rules_output[i][j];
        total_mu_strength_kd += rule_strength;
      }
    }
  }

  if (total_mu_strength_kp > 0) {
    outKp = weighted_sum_kp / total_mu_strength_kp;
  } else {
    outKp = Kp_M; 
  }

  if (total_mu_strength_kd > 0) {
    outKd = weighted_sum_kd / total_mu_strength_kd;
  } else {
    outKd = Kd_M; 
  }
}
