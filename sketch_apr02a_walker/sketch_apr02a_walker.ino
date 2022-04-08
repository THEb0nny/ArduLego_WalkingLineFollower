// https://www.youtube.com/watch?time_continue=1&v=fG4Vc6EBjkM&feature=emb_logo
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/
// https://alexgyver.ru/gyverbutton/

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"
#include "GyverButton.h"

#define RESET_BTN_PIN 8 // Кнопка для мягкого перезапуска

#define SERVO_MOT_L_PIN 9 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 10 // Пин правого серво мотора

#define LEFT_LINE_SENSOR_PIN A0 // Пин левого датчика линии
#define RIGHT_LINE_SENSOR_PIN A1 // Пин правого датчика линии

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

#define LEFT_RAW_REF_BLACK_LINE_SEN 531 // Значение чёрного левого датчика линии
#define LEFT_RAW_REF_WHITE_LINE_SEN 34 // Значение белого левого датчика линии

#define RIGHT_RAW_REF_BLACK_LINE_SEN 533 // Значение чёрного правого датчика линии
#define RIGHT_RAW_REF_WHITE_LINE_SEN 33 // Значение белого правого датчика линии

#define MIN_SPEED_FOR_SERVO_MOT 10 // Минимальное значение для старта серво мотора

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(MS, 10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки

unsigned long currTime, prevTime, loopTime; // Время

float Kp = 0.25, Ki = 0, Kd = 0; // Коэффиценты регулятора при старте

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println();
  // Подключение кнопки start/stop/reset
  btn.setDebounce(50); // Настройка антидребезга кнопки (по умолчанию 80 мс)
  btn.setTimeout(300); // Настройка таймаута на удержание кнопки (по умолчанию 500 мс)
  btn.setClickTimeout(600); // Настройка таймаута между кликами по кнопке (по умолчанию 300 мс)
  btn.setType(HIGH_PULL); // HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC, LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  btn.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка, NORM_CLOSE - нормально-замкнутая кнопка
  btn.setTickMode(AUTO); // MANUAL - нужно вызывать функцию tick() вручную, AUTO - tick() входит во все остальные функции и опрашивается сама!
  pinMode(RIGHT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
  // Моторы
  lServoMot.attach(SERVO_MOT_L_PIN, 500, 2500); rServoMot.attach(SERVO_MOT_R_PIN, 500, 2500); // Подключение моторов
  MotorSpeed(lServoMot, 0, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 0, SERVO_MOT_R_DIR_MODE); // При старте моторы выключаем
  regulator.setDirection(NORMAL); // Направление регулирования (NORMAL/REVERSE)
  regulator.setLimits(-90, 90); // Пределы регулятора
  Serial.println("Ready... Press btn");
  while (!btn.isClick()); // Цикл, в котором проверяем, что нажали на кнопку
  Serial.println("Go!!!");
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
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
    Serial.print("error: "); Serial.println(error);
    regulator.setpoint = error; // Передаём ошибку
    //regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    Serial.print("u: "); Serial.println(u);
    MotorsControl(u, 30);
    //MotorSpeed(lServoMot, 50, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 50, SERVO_MOT_R_DIR_MODE);
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotSpeed = speed + dir, rServoMotSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotSpeed), abs(rServoMotSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotSpeed *= z, rServoMotSpeed *= z;
  lServoMotSpeed = constrain(lServoMotSpeed, -90, 90), rServoMotSpeed = constrain(rServoMotSpeed, -90, 90);
  //Serial.print(lServoMotSpeed); Serial.print(", "); Serial.println(rServoMotSpeed);
  MotorSpeed(lServoMot, lServoMotSpeed, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, rServoMotSpeed, SERVO_MOT_R_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int speed, int rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  speed = constrain(speed, -90, 90) * rotateMode;
  //Serial.print("servoMotSpeed "); Serial.print(speed);
  if (speed >= 0) {
    speed = map(speed, 0, 90, 90, 180);
  } else {
    speed = map(speed, 0, -90, 90, 0);
  }
  servoMot.write(speed);
  //Serial.print(" convertedMotSpeed "); Serial.println(speed);
}

// Калибровка и нормализация значений с датчика линии
int GetCalibValColorS(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, 255);
  lineSensorVal = constrain(lineSensorVal, 0, 255);
  return lineSensorVal;
}
