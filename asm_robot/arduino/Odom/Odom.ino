/*
  Arduino Mega + L298N (IN1..IN4) + 2 энкодера (A на прерываниях)
  Робот едет вперед с постоянным PWM.
  В Serial выводим: время(ms), скорость левого колеса (м/с), скорость правого (м/с), скорость робота (м/с).

  ВАЖНО по L298N:
  - У вас ENA/ENB постоянно подключены к 5V (драйвер всегда "разрешен").
  - Тогда скорость можно задавать PWM на одном из IN-пинов, а второй держать LOW.
    Но для РАЗНЫХ направлений PWM нужно подавать на РАЗНЫЙ IN (один из пары), второй — LOW.
    Это стандартная логика управления через IN при EN=HIGH. (см. ссылки в ответе)

  Пины по вашей схеме:
    Левый мотор: IN1=D8, IN2=D9
    Правый мотор: IN3=D5, IN4=D6

    Левый энкодер: A=D2 (interrupt), B=D4
    Правый энкодер: A=D3 (interrupt), B=D7

  pulses_per_revolution = 1123 "кликов" на 1 оборот КОЛЕСА.

  ПРИМЕЧАНИЕ:
  Если скорости будут со знаком "-" при движении вперед — просто инвертируйте соответствующий INVERT_*_ENCODER.
*/

#include <Arduino.h>

// -------------------- ПИНЫ L298N --------------------
const uint8_t L_IN1 = 8;
const uint8_t L_IN2 = 9;
const uint8_t R_IN3 = 5;
const uint8_t R_IN4 = 6;

// -------------------- ПИНЫ ЭНКОДЕРОВ --------------------
const uint8_t ENC_L_A = 2;   // прерывание
const uint8_t ENC_L_B = 4;
const uint8_t ENC_R_A = 3;   // прерывание
const uint8_t ENC_R_B = 7;

// -------------------- ПАРАМЕТРЫ КОЛЕС/ЭНКОДЕРОВ --------------------
const long  PULSES_PER_REV     = 1123;
const float WHEEL_DIAMETER_M   = 0.065f;
const float WHEEL_CIRCUM_M     = PI * WHEEL_DIAMETER_M;

// -------------------- НАСТРОЙКИ --------------------
const uint8_t  PWM_CMD          = 150;   // постоянный PWM (0..255)
const uint16_t PRINT_PERIOD_MS  = 100;   // период вывода

// Исправление "робот крутится на месте":
// Инвертируем направление ОДНОГО мотора (обычно достаточно правого).
const bool INVERT_RIGHT_MOTOR_DIR = true;   // <- если всё равно крутится на месте, попробуйте false и/или инвертировать левый
const bool INVERT_LEFT_MOTOR_DIR  = false;

// Инверсия знака энкодера (если скорость по знаку не совпадает с "вперед")
const bool INVERT_LEFT_ENCODER  = true;
const bool INVERT_RIGHT_ENCODER = false;

// -------------------- СЧЁТЧИКИ ЭНКОДЕРОВ --------------------
// volatile, потому что меняются в прерываниях
volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

// -------------------- ISR (прерывания по каналу A) --------------------
/*
  На каждом фронте RISING канала A читаем канал B:
    B=HIGH -> +1
    B=LOW  -> -1
  (какая сторона "плюс", зависит от того, как подключены A/B, поэтому есть INVERT_*_ENCODER)
*/
void isrLeftA() {
  bool b = digitalRead(ENC_L_B);
  long step = b ? +1 : -1;
  if (INVERT_LEFT_ENCODER) step = -step;
  encLeftCount += step;
}

void isrRightA() {
  bool b = digitalRead(ENC_R_B);
  long step = b ? +1 : -1;
  if (INVERT_RIGHT_ENCODER) step = -step;
  encRightCount += step;
}

// -------------------- УПРАВЛЕНИЕ МОТОРОМ ЧЕРЕЗ ДВА IN --------------------
/*
  Когда EN=HIGH, направление задаётся комбинацией IN1/IN2.
  Для регулировки скорости можно подать PWM на "активный" вход, а второй держать LOW.

  forward=true:
    inA = LOW
    inB = PWM

  forward=false:
    inA = PWM
    inB = LOW

  Это важно: при смене направления PWM "переезжает" на другой IN-пин.
*/
void driveMotor2IN(uint8_t inA, uint8_t inB, uint8_t pwm, bool forward) {
  if (forward) {
    digitalWrite(inA, LOW);
    analogWrite(inB, pwm);
  } else {
    analogWrite(inA, pwm);
    digitalWrite(inB, LOW);
  }
}

// Удобная обёртка: "ехать вперёд" с учётом инверсий сторон
void driveForward(uint8_t pwmLeft, uint8_t pwmRight) {
  bool leftForward  = true ^ INVERT_LEFT_MOTOR_DIR;
  bool rightForward = true ^ INVERT_RIGHT_MOTOR_DIR;

  driveMotor2IN(L_IN1, L_IN2, pwmLeft,  leftForward);
  driveMotor2IN(R_IN3, R_IN4, pwmRight, rightForward);
}

void setup() {
  Serial.begin(115200);

  // --- Моторные пины ---
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  // --- Энкодеры ---
  /*
    Даже если "подтяжки не нужны", на практике многие энкодеры имеют open-collector/open-drain выходы.
    Без pull-up вход может "плавать", что даёт случайные редкие импульсы и нули.
    Поэтому безопасно включить внутреннюю подтяжку INPUT_PULLUP.
  */
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // --- Прерывания ---
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, RISING);

  // Стартуем движение
  driveForward(PWM_CMD, PWM_CMD);

  // Заголовок CSV
  Serial.println(F("t_ms,v_left_mps,v_right_mps,v_robot_mps"));
}

void loop() {
  static uint32_t lastPrintMs = 0;

  // "прошлые" значения счетчиков, чтобы считать дельту
  static long lastLeft  = 0;
  static long lastRight = 0;

  uint32_t nowMs = millis();

  if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
    // dt берём от разницы millis: так интервал ровный и понятный
    float dt = (nowMs - lastPrintMs) / 1000.0f;
    lastPrintMs = nowMs;

    // атомарно копируем счетчики из ISR
    long leftNow, rightNow;
    noInterrupts();
    leftNow  = encLeftCount;
    rightNow = encRightCount;
    interrupts();

    long dLeft  = leftNow  - lastLeft;
    long dRight = rightNow - lastRight;
    lastLeft  = leftNow;
    lastRight = rightNow;

    // pulses -> revs -> rps -> m/s
    float leftRevs  = (float)dLeft  / (float)PULSES_PER_REV;
    float rightRevs = (float)dRight / (float)PULSES_PER_REV;

    float vLeft  = (dt > 0.0f) ? (leftRevs  / dt) * WHEEL_CIRCUM_M : 0.0f;
    float vRight = (dt > 0.0f) ? (rightRevs / dt) * WHEEL_CIRCUM_M : 0.0f;

    float vRobot = 0.5f * (vLeft + vRight);

    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(vLeft, 4);
    Serial.print(',');
    Serial.print(vRight, 4);
    Serial.print(',');
    Serial.println(vRobot, 4);

    // поддерживаем постоянное движение (на будущее, если будете менять PWM)
    driveForward(PWM_CMD, PWM_CMD);
  }
}
