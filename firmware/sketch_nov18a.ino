// =========================================================================
// SEGUIDOR DE LINEA PD - TOLERANCIA A HUECOS + MAX_SPEED AJUSTABLE
// S1 (Izq) = P13 | S2 = P33 | S3 (Centro) = P14 | S4 = P27 | S5 (Der) = P26
// Sensores: LOW (0) = Negro | HIGH (1) = Blanco
// =========================================================================

// --- SENSORES ---
const int SENSORS[5] = {13, 33, 14, 27, 26};
const float WEIGHTS[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};

// --- DRIVER L298N (Mapeo a ruedas físicas) ---
const int MOT_IZQ_FWD = 22; 
const int MOT_IZQ_REV = 23; 
const int MOT_DER_FWD = 18; 
const int MOT_DER_REV = 19; 

// --- CONFIGURACION PWM ---
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;
const int CH_IZQ_F = 0;
const int CH_IZQ_R = 1;
const int CH_DER_F = 2;
const int CH_DER_R = 3;

// =========================================================================
// --- PARAMETROS AJUSTABLES POR EL USUARIO ---
// =========================================================================
const int BASE_SPEED = 170;  // Velocidad crucero en rectas
const int MAX_SPEED  = 200;  // <--- VELOCIDAD MAXIMA (Puedes cambiar este valor)
const int MAX_REV    = -120; // Tope de retroceso en giro cerrado

float Kp = 40.0;             // Fuerza de corrección proporcional
float Kd = 22.0;             // Amortiguación derivativa (antivadeo)
// =========================================================================

float error = 0.0;
float lastError = 0.0;

void driveMotors(int leftSpeed, int rightSpeed) {
  // Limitamos estrictamente al MAX_SPEED definido arriba
  leftSpeed  = constrain(leftSpeed,  MAX_REV, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MAX_REV, MAX_SPEED);

  int izq_fwd = (leftSpeed >= 0)  ? leftSpeed  : 0;
  int izq_rev = (leftSpeed < 0)   ? -leftSpeed : 0;
  int der_fwd = (rightSpeed >= 0) ? rightSpeed : 0;
  int der_rev = (rightSpeed < 0)  ? -rightSpeed : 0;

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(MOT_IZQ_FWD, izq_fwd);
    ledcWrite(MOT_IZQ_REV, izq_rev);
    ledcWrite(MOT_DER_FWD, der_fwd);
    ledcWrite(MOT_DER_REV, der_rev);
  #else
    ledcWrite(CH_IZQ_F, izq_fwd);
    ledcWrite(CH_IZQ_R, izq_rev);
    ledcWrite(CH_DER_F, der_fwd);
    ledcWrite(CH_DER_R, der_rev);
  #endif
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    pinMode(SENSORS[i], INPUT);
  }

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOT_IZQ_FWD, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_IZQ_REV, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_DER_FWD, PWM_FREQ, PWM_RES);
    ledcAttach(MOT_DER_REV, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(CH_IZQ_F, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_IZQ_FWD, CH_IZQ_F);
    ledcSetup(CH_IZQ_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_IZQ_REV, CH_IZQ_R);
    ledcSetup(CH_DER_F, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_DER_FWD, CH_DER_F);
    ledcSetup(CH_DER_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOT_DER_REV, CH_DER_R);
  #endif

  driveMotors(0, 0);
  delay(1000);
}

void loop() {
  float sumWeighted = 0.0;
  int sensoresEnNegro = 0;

  // 1. Leer sensores (LOW = 0 detecta línea negra)
  for (int i = 0; i < 5; i++) {
    if (digitalRead(SENSORS[i]) == LOW) {
      sumWeighted += WEIGHTS[i];
      sensoresEnNegro++;
    }
  }

  // 2. Si los 5 sensores ven negro (fin de pista o meta), frena
  if (sensoresEnNegro == 5) {
    driveMotors(0, 0);
    return;
  }

  // 3. Manejo de blanco total (espacio vacío o línea muy delgada entre sensores)
  if (sensoresEnNegro == 0) {
    // TOLERANCIA A HUECOS: Mantiene el último error para cruzar el espacio ciego
    error = lastError; 
  } else {
    // Cálculo normal del centro de masa de la línea
    error = sumWeighted / sensoresEnNegro;
  }

  // 4. Cálculo PD
  float P = Kp * error;
  float D = Kd * (error - lastError);
  float correction = P + D;

  lastError = error;

  // 5. Aplicación diferencial a las ruedas
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  driveMotors(leftSpeed, rightSpeed);

  delay(10);
}
