// https://www.youtube.com/watch?time_continue=1&v=fG4Vc6EBjkM&feature=emb_logo
// https://alexgyver.ru/gyverpid/
// https://alexgyver.ru/gyvertimer/
// https://alexgyver.ru/gyverbutton/

#define PID_OPTIMIZED_I // Параметр для оптимизации суммы регулятора

#include <SoftwareSerial.h>
#include <Servo.h>
#include "GyverPID.h"
#include "GyverTimer.h"
#include "GyverButton.h"

#define DEBUG false // Дебаг true/false

#define RESET_BTN_PIN 3 // Пин кнопки для старта, мягкого перезапуска

#define SERVO_MOT_L_PIN 9 // Пин левого серво мотора
#define SERVO_MOT_R_PIN 10 // Пин правого серво мотора

#define CENTER_LEFT_LINE_SENSOR_PIN A0 // Пин центрального левого датчика линии
#define CENTER_RIGHT_LINE_SENSOR_PIN A1 // Пин центрального правого датчика линии
#define SIDE_LEFT_LINE_SENSOR_PIN A2 // Пин крайнего левого датчика
#define SIDE_RIGHT_LINE_SENSOR_PIN A3 // Пин крайнего левого датчика

#define SERVO_MOT_L_DIR_MODE 1 // Режим вращения левого мотора, где нормально 1, реверс -1
#define SERVO_MOT_R_DIR_MODE -1 // Режим вращения правого мотора

#define CENTER_LEFT_RAW_REF_BLACK_LINE_SEN 375 // Значение чёрного центральнего левого датчика линии
#define CENTER_LEFT_RAW_REF_WHITE_LINE_SEN 34 // Значение белого левого датчика линии
#define SIDE_LEFT_RAW_REF_BLACK_LINE_SEN 310 // Значение чёрного крайнего левого датчика линии
#define SIDE_LEFT_RAW_REF_WHITE_LINE_SEN 34 // Значение белого крайнего левого датчика линии

#define CENTER_RIGHT_RAW_REF_BLACK_LINE_SEN 265 // Значение чёрного правого датчика линии
#define CENTER_RIGHT_RAW_REF_WHITE_LINE_SEN 33 // Значение белого правого датчика линии
#define SIDE_RIGHT_RAW_REF_BLACK_LINE_SEN 431 // Значение чёрного крайнего правого датчика линии
#define SIDE_RIGHT_RAW_REF_WHITE_LINE_SEN 32 // Значение белого крайнего правого датчика линии

#define COEFF_SIDE_LINE_SEN 1.75 // Коэффицент крайних датчиков линии

#define MIN_SPEED_FOR_SERVO_MOT 10 // Минимальное значение для старта серво мотора

#define NEED_ADAPT_BLACK_WHITE_LINE_SEN_VAL false // Нужно ли вызывать функцию адаптации значений чёрного и белого датчиков

Servo lServoMot, rServoMot; // Инициализация объектов моторов
GTimer myTimer(MS, 10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки

unsigned long currTime, prevTime, loopTime; // Время

float Kp = 0.3, Ki = 0, Kd = 0; // Коэффиценты регулятора при старте

GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора

int speed = 90; // Скорость

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println();
  // Подключение кнопки start/stop/reset
  btn.setDebounce(50); // Настройка антидребезга кнопки (по умолчанию 80 мс)
  btn.setTimeout(300); // Настройка таймаута на удержание кнопки (по умолчанию 500 мс)
  btn.setClickTimeout(600); // Настройка таймаута между кликами по кнопке (по умолчанию 300 мс)
  btn.setType(HIGH_PULL); // HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC, LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
  btn.setDirection(NORM_OPEN); // NORM_OPEN - нормально-разомкнутая кнопка, NORM_CLOSE - нормально-замкнутая кнопка
  btn.setTickMode(AUTO); // MANUAL - нужно вызывать функцию tick() вручную, AUTO - tick() входит во все остальные функции и опрашивается сама!
  pinMode(CENTER_LEFT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
  pinMode(CENTER_RIGHT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
  pinMode(SIDE_LEFT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
  pinMode(SIDE_RIGHT_LINE_SENSOR_PIN, INPUT); // Настойка пина правого датчика линии
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
  if (Serial.available() > 2) {
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String command = Serial.readStringUntil('\n');    
    command.trim();
    char incoming = command[0];
    command.remove(0, 1);
    float value = command.toFloat();
    switch (incoming) {
      case 'p':
        regulator.Kp = value;
        break;
      case 'i':
        regulator.Ki = value;
        break;
      case 'd':
        regulator.Kd = value;
        break;
      case 's':
        speed = value;
        break;
      default:
        break;
    }
  }
  if (btn.isClick()) softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  if (myTimer.isReady()) { // Раз в 10 мсек выполнять
    // Считываем сырые значения с датчиков линии
    int cLeftRawRefLineS = analogRead(CENTER_LEFT_LINE_SENSOR_PIN);
    int cRightRawRefLineS = analogRead(CENTER_RIGHT_LINE_SENSOR_PIN);
    int sLeftRawRefLineS = analogRead(SIDE_LEFT_LINE_SENSOR_PIN);
    int sRightRawRefLineS = analogRead(SIDE_RIGHT_LINE_SENSOR_PIN);
    // Калибруем/обрабатываем значения с датчиков линии
    int cLeftRefLineS = GetCalibValColorS(cLeftRawRefLineS, CENTER_LEFT_RAW_REF_BLACK_LINE_SEN, CENTER_LEFT_RAW_REF_WHITE_LINE_SEN);
    int cRightRefLineS = GetCalibValColorS(cRightRawRefLineS, CENTER_RIGHT_RAW_REF_BLACK_LINE_SEN, CENTER_RIGHT_RAW_REF_WHITE_LINE_SEN);
    int sLeftRefLineS = GetCalibValColorS(sLeftRawRefLineS, SIDE_LEFT_RAW_REF_BLACK_LINE_SEN, SIDE_LEFT_RAW_REF_WHITE_LINE_SEN);
    int sRightRefLineS = GetCalibValColorS(sRightRawRefLineS, SIDE_RIGHT_RAW_REF_BLACK_LINE_SEN, SIDE_RIGHT_RAW_REF_WHITE_LINE_SEN);
    float error = CalcLineSensorsError(1, sLeftRefLineS, cLeftRefLineS, cRightRefLineS, sRightRefLineS); // Нахождение ошибки
    regulator.setpoint = error; // Передаём ошибку
    regulator.setDt(loopTime); // Установка dt для регулятора
    float u = regulator.getResult(); // Управляющее воздействие с регулятора
    MotorsControl(u, speed);
    //MotorSpeed(lServoMot, 50, SERVO_MOT_L_DIR_MODE); MotorSpeed(rServoMot, 50, SERVO_MOT_R_DIR_MODE);
    if (DEBUG) {
      // Для отладки значений серого
      Serial.print("sLeftRawRefLineS: "); Serial.print(sLeftRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("cLeftRawRefLineS: "); Serial.print(cLeftRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("cRightRawRefLineS: "); Serial.print(cRightRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("sRightRawRefLineS: "); Serial.print(sRightRawRefLineS); Serial.println("\t"); // Для вывода сырых значений
      Serial.print("sLeftRefLineS: "); Serial.print(sLeftRefLineS); Serial.print(", ");
      Serial.print("cLeftRefLineS: "); Serial.print(cLeftRefLineS); Serial.print("\t");
      Serial.print("cRightRefLineS: "); Serial.print(cRightRefLineS); Serial.print(", ");
      Serial.print("sRightRefLineS: "); Serial.print(sRightRefLineS); Serial.println("\t");
    }
    Serial.print("error: "); Serial.println(error);
    Serial.print("u: "); Serial.println(u);
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
  if (speed >= 0) speed = map(speed, 0, 90, 90, 180);
  else speed = map(speed, 0, -90, 90, 0);
  servoMot.write(speed);
  //Serial.print(" convertedMotSpeed "); Serial.println(speed);
}

// Калибровка и нормализация значений с датчика линии
int GetCalibValColorS(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  if (NEED_ADAPT_BLACK_WHITE_LINE_SEN_VAL) AdaptLineSenVal(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS);
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, 90);
  lineSensorVal = constrain(lineSensorVal, 0, 90);
  return lineSensorVal;
}

// Адаптация значений белого и чёрного датчиков
int AdaptLineSenVal(int rawRefValLineS, int blackRawRefLineS, int whiteRawRefLineS) {
  // To Do
  // Знаки меняются местами, потому что в режиме сырых значений меньше и больше - наоборот
  if (rawRefValLineS > blackRawRefLineS) blackRawRefLineS = rawRefValLineS;
  else if (rawRefValLineS < whiteRawRefLineS) whiteRawRefLineS = rawRefValLineS;
  else return; // return -1;
}

float CalcLineSensorsError(byte calcMetod, int sLeftLineSensorRefVal, int cLeftLineSensorRefVal, int cRightLineSensorRefVal, int sRightLineSensorRefVal) {
  float error = 0;
  if (calcMetod == 0) {
    error = cLeftLineSensorRefVal - cRightLineSensorRefVal;
  } else if (calcMetod == 1) {
    error = (COEFF_SIDE_LINE_SEN * sLeftLineSensorRefVal + 1 * cLeftLineSensorRefVal) - (1 * cRightLineSensorRefVal + COEFF_SIDE_LINE_SEN * sRightLineSensorRefVal);
  }
  return error;
}
