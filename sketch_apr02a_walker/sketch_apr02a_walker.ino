// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"
#include "GyverButton.h"

#define RESET_BTN_PIN 8 // Кнопка для мягкого перезапуска

#define SERVO_MOT_L_PIN 2 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 4 // Пин правого серво мотора

#define LEFT_LINE_SENSOR_PIN A0 // Пин левого датчика линии
#define RIGHT_LINE_SENSOR_PIN A1 // Пин правого датчика линии

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

#define LEFT_RAW_REF_BLACK_LINE_SEN 756 // Значение чёрного левого датчика линии
#define LEFT_RAW_REF_WHITE_LINE_SEN 48 // Значение белого левого датчика линии

#define RIGHT_RAW_REF_BLACK_LINE_SEN 755 // Значение чёрного правого датчика линии
#define RIGHT_RAW_REF_WHITE_LINE_SEN 45 // Значение белого правого датчика линии

#define MIN_SPEED_FOR_START_SERVO_MOT 10 // Минимальное значение для старта серво мотора

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки

unsigned long currTime, prevTime, loopTime; // Время

float Kp = 0.5, Ki = 0, Kd = 0; // Коэффиценты регулятора при старте

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  //pinMode(RESET_BTN_PIN, INPUT_PULLUP); // Подключение кнопки Start/stop/reset
  btn.setType(HIGH_PULL); // LOW_PULL/HIGH_PULL
  btn.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка, NORM_CLOSE - нормально-замкнутая кнопка
  btn.setTickMode(AUTO); // MANUAL - нужно вызывать функцию tick() вручную, AUTO - tick() входит во все остальные функции и опрашивается сама!
  pinMode(LEFT_LINE_SENSOR_PIN, INPUT); // Настойка пина левого датчика линии
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
  lServoMot.attach(SERVO_MOT_L_PIN); rServoMot.attach(SERVO_MOT_R_PIN); // Подключение моторов
  MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE); // При старте моторы выключаем
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-90, 90); // Пределы регулятора
  Serial.println();
  Serial.println("Ready... Pres btn");
  while (!btn.isClick());
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  //if (!digitalRead(RESET_BTN_PIN)) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (btn.isClick()) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    // Считываем сырые значения с датчиков линии
    int lRawRefLineS = analogRead(LEFT_LINE_SENSOR_PIN);
    int rRawRefLineS = analogRead(RIGHT_LINE_SENSOR_PIN);
    // Калибруем/обрабатываем значения с датчиков линии
    int lRefLineS = GetCalibValColorS(lRawRefLineS, LEFT_RAW_REF_BLACK_LINE_SEN, LEFT_RAW_REF_WHITE_LINE_SEN);
    int rRefLineS = GetCalibValColorS(rRawRefLineS, RIGHT_RAW_REF_BLACK_LINE_SEN, RIGHT_RAW_REF_WHITE_LINE_SEN);
    Serial.print("lRawSensor: "); Serial.print(lRawRefLineS); Serial.print("\t"); // Для вывода сырых значений
    Serial.print("rRawSensor: "); Serial.print(rRawRefLineS); Serial.println("\t"); // Для вывода сырых значений
    Serial.print("lLineS: "); Serial.print(lRefLineS); Serial.print("\t");
    Serial.print("rLineS: "); Serial.print(rRefLineS); Serial.println("\t");
    int error = lRefLineS - rRefLineS; // Нахождение ошибки
    regulator.setpoint = error; // Передаём ошибку
    //regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    MotorsControl(0, 90);
    //MotorSpeed(lServoMot, 50 + u, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 50 - u, SERVO_MOT_R_DIR_MODE);
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  lServoMotSpeed = constrain(lServoMotSpeed, -90, 90), rServoMotSpeed = constrain(rServoMotSpeed, -90, 90);
  Serial.print(lServoMotSpeed); Serial.print(", "); Serial.println(rServoMotSpeed);
  MotorSpeed(lServoMot, lServoMotSpeed, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, rServoMotSpeed, SERVO_MOT_R_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int speed, int rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90) * rotateMode;
  Serial.print("servoMotSpeed "); Serial.print(speed);
  if (speed >= 0) {
    speed = map(speed, 0, 90, 90, 180);
  } else {
    speed = map(speed, 0, -90, 90, 0);
  }
  servoMot.write(speed);
  Serial.print(" convertedMotSpeed "); Serial.println(speed);
}

// Калибровка и нормализация значений с датчика линии
int GetCalibValColorS(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, 255);
  lineSensorVal = constrain(lineSensorVal, 0, 255);
  return lineSensorVal;
}
