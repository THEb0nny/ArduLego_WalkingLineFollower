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

#define DEBUG_LEVEL 1 // Уровень дебага

#define RESET_BTN_PIN 3 // Пин кнопки для старта, мягкого перезапуска

#define SERVO_MOT_L1_PIN 7 // Пин левого первого серво мотора
#define SERVO_MOT_L2_PIN 4 // Пин левого второго серво мотора
#define SERVO_MOT_R1_PIN 10 // Пин правого серво мотора
#define SERVO_MOT_R2_PIN 8 // Пин правого серво мотора

#define GEEKSERVO_STEPPING_PULSE 1500 // Значение импулста для остановки мотора, нулевой скорости geekservo
#define GEEKSERVO_CW_LEFT_BOARD_PULSE_WIDTH 1595 // Левая граница ширины импульса вравщения по часовой geekservo
#define GEEKSERVO_CW_RIGHT_BOARD_PULSE_WIDTH 2500 // Правая граница ширины импульса вращения по часовой geekservo
#define GEEKSERVO_CCW_LEFT_BOARD_PULSE_WIDTH 500 // Минимальное значение ширины импульса вравщения против часовой geekservo
#define GEEKSERVO_CCW_RIGHT_BOARD_PULSE_WIDTH 1365 // Максимальное значение ширины импульса вращения против часовой geekservo

#define SERVO_MOT_L1_DIR_MODE false // Режим вращения первого левого сервомотора, где нормально 1, реверс -1
#define SERVO_MOT_L2_DIR_MODE false // Режим вращения второго левого сервомотора
#define SERVO_MOT_R1_DIR_MODE true // Режим вращения первого правого сервомотора
#define SERVO_MOT_R2_DIR_MODE true // Режим вращения второго правого сервомотора

#define CENTER_LEFT_LINE_SENSOR_PIN A0 // Пин центрального левого датчика линии
#define CENTER_RIGHT_LINE_SENSOR_PIN A1 // Пин центрального правого датчика линии
#define SIDE_LEFT_LINE_SENSOR_PIN A2 // Пин крайнего левого датчика
#define SIDE_RIGHT_LINE_SENSOR_PIN A3 // Пин крайнего правого датчика

#define CENTER_LEFT_RAW_REF_BLACK_LINE_SEN 375 // Значение чёрного центральнего левого датчика линии
#define CENTER_LEFT_RAW_REF_WHITE_LINE_SEN 34 // Значение белого левого датчика линии
#define SIDE_LEFT_RAW_REF_BLACK_LINE_SEN 310 // Значение чёрного крайнего левого датчика линии
#define SIDE_LEFT_RAW_REF_WHITE_LINE_SEN 34 // Значение белого крайнего левого датчика линии

#define CENTER_RIGHT_RAW_REF_BLACK_LINE_SEN 265 // Значение чёрного правого датчика линии
#define CENTER_RIGHT_RAW_REF_WHITE_LINE_SEN 33 // Значение белого правого датчика линии
#define SIDE_RIGHT_RAW_REF_BLACK_LINE_SEN 431 // Значение чёрного крайнего правого датчика линии
#define SIDE_RIGHT_RAW_REF_WHITE_LINE_SEN 32 // Значение белого крайнего правого датчика линии

#define COEFF_SIDE_LINE_SEN 1.75 // Коэффицент усиления для крайних датчиков линии

unsigned long currTime, prevTime, loopTime; // Время
float Kp = 0.3, Ki = 0, Kd = 0; // Коэффиценты регулятора при старте
int speed = 90; // Инициализируем переменную скорости

Servo l1ServoMot, l2ServoMot, r1ServoMot, r2ServoMot; // Инициализация объектов моторов
GTimer myTimer(MS, 10); // Инициализация объекта таймера
GButton btn(RESET_BTN_PIN); // Инициализация кнопки
GyverPID regulator(Kp, Ki, Kd, 10); // Инициализируем коэффициенты регулятора

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
  l1ServoMot.attach(SERVO_MOT_L1_PIN); l2ServoMot.attach(SERVO_MOT_L2_PIN); r1ServoMot.attach(SERVO_MOT_R1_PIN); r2ServoMot.attach(SERVO_MOT_R2_PIN); // Подключение моторов
  MotorSpeed(l1ServoMot, 0, SERVO_MOT_L1_DIR_MODE); MotorSpeed(r1ServoMot, 0, SERVO_MOT_R1_DIR_MODE); // При старте моторы выключаем
  MotorSpeed(l2ServoMot, 0, SERVO_MOT_L2_DIR_MODE); MotorSpeed(r2ServoMot, 0, SERVO_MOT_R2_DIR_MODE); // При старте моторы выключаем
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
  ParseSerialInputValues(); // Парсинг значений из Serial
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
    //MotorsControl(u, speed);
    MotorSpeed(l1ServoMot, 90, SERVO_MOT_L1_DIR_MODE); MotorSpeed(l2ServoMot, 90, SERVO_MOT_L2_DIR_MODE);
    MotorSpeed(r1ServoMot, 90, SERVO_MOT_R1_DIR_MODE); MotorSpeed(r2ServoMot, 90, SERVO_MOT_R2_DIR_MODE);
    if (DEBUG_LEVEL == 3) {
      // Для отладки значений серого
      Serial.print("sLeftRawRefLineS: "); Serial.print(sLeftRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("cLeftRawRefLineS: "); Serial.print(cLeftRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("cRightRawRefLineS: "); Serial.print(cRightRawRefLineS); Serial.print(", "); // Для вывода сырых значений
      Serial.print("sRightRawRefLineS: "); Serial.print(sRightRawRefLineS); Serial.println(); // Для вывода сырых значений
      Serial.print("sLeftRefLineS: "); Serial.print(sLeftRefLineS); Serial.print(", ");
      Serial.print("cLeftRefLineS: "); Serial.print(cLeftRefLineS); Serial.print(", ");
      Serial.print("cRightRefLineS: "); Serial.print(cRightRefLineS); Serial.print(", ");
      Serial.print("sRightRefLineS: "); Serial.print(sRightRefLineS); Serial.println();
    }
    if (DEBUG_LEVEL >= 1 && DEBUG_LEVEL < 3) {
      Serial.print("error: "); Serial.print(error); Serial.print(", ");
      Serial.print("u: "); Serial.println(u);
    }
  }
}

// Управление двумя моторами
void MotorsControl(int dir, int speed) {
  int lServoMotorsSpeed = speed + dir, rServoMotorsSpeed = speed - dir;
  float z = (float) speed / max(abs(lServoMotorsSpeed), abs(rServoMotorsSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  lServoMotorsSpeed *= z, rServoMotorsSpeed *= z;
  lServoMotorsSpeed = constrain(lServoMotorsSpeed, -90, 90), rServoMotorsSpeed = constrain(rServoMotorsSpeed, -90, 90);
  //Serial.print(lServoMotorsSpeed); Serial.print(", "); Serial.println(rServoMotorsSpeed);
  MotorSpeed(l2ServoMot, lServoMotorsSpeed, SERVO_MOT_L1_DIR_MODE); MotorSpeed(l2ServoMot, lServoMotorsSpeed, SERVO_MOT_L2_DIR_MODE);
  MotorSpeed(r1ServoMot, rServoMotorsSpeed, SERVO_MOT_R1_DIR_MODE); MotorSpeed(r2ServoMot, rServoMotorsSpeed, SERVO_MOT_R2_DIR_MODE);
}

// Управление серво мотором
void MotorSpeed(Servo servoMot, int inputSpeed, bool rotateMode) {
  // Servo, 0->FW, 90->stop, 180->BW
  inputSpeed = constrain(inputSpeed, -90, 90) * (rotateMode? -1 : 1);
  if (DEBUG_LEVEL >= 1 && DEBUG_LEVEL < 3) {
    Serial.print("inputSpeed "); Serial.print(inputSpeed); Serial.print(", "); 
  }
  int speed = map(inputSpeed, -90, 90, 0, 180);
  if (DEBUG_LEVEL >= 1 && DEBUG_LEVEL < 3) {
    Serial.print("speed "); Serial.println(speed);
  }
  if (inputSpeed > 0) speed = map(speed, 90, 180, GEEKSERVO_CW_LEFT_BOARD_PULSE_WIDTH, GEEKSERVO_CW_RIGHT_BOARD_PULSE_WIDTH);
  else if (inputSpeed < 0) speed = map(speed, 0, 90, GEEKSERVO_CCW_LEFT_BOARD_PULSE_WIDTH, GEEKSERVO_CCW_RIGHT_BOARD_PULSE_WIDTH);
  else speed = GEEKSERVO_STEPPING_PULSE;
  servoMot.writeMicroseconds(speed);
  if (DEBUG_LEVEL == 2) {
    Serial.print("inputServoMotSpeed "); Serial.print(inputSpeed); Serial.print(" ");
    Serial.print("servoMotSpeed "); Serial.println(speed);
  }
}

// Калибровка и нормализация значений с датчика линии
int GetCalibValColorS(int rawRefLineSenVal, int blackRawRefLineS, int whiteRawRefLineS) {
  int lineSensorVal = map(rawRefLineSenVal, blackRawRefLineS, whiteRawRefLineS, 0, 90);
  lineSensorVal = constrain(lineSensorVal, 0, 90);
  return lineSensorVal;
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

// Парсинг значений из Serial
void ParseSerialInputValues() {
    if (Serial.available() > 2) {
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = Serial.readStringUntil('\n');    
    inputStr.trim();
    inputStr.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = inputStr.length(); // Переменая для хронения индекса вхождения цифры в входной строке, изначально равна размеру строки
    for (byte i = 0; i < 10; i++) { // Поиск первого вхождения цифры от 0 по 9 в подстроку
      byte index = inputStr.indexOf(String(i)); // Узнаём индекс, где нашли цифру параметра цикла
      if (index < strIndex && index != 255) strIndex = index;  // Если индекс цифры меньше strIndex, то обновляем strIndex 
    }
    String key = inputStr.substring(0, strIndex); // Записываем ключ с начала строки до первой цицры
    float value = inputStr.substring(strIndex, inputStr.length()).toFloat(); // Записываем значение с начала цифры до конца строки
    if (key == "p") {
      regulator.Kp = value;
    } else if (key == "i") {
      regulator.Ki = value;
      regulator.integral = 0;
    } else if (key == "d") {
      regulator.Kd = value;
    } else if (key == "s") {
      speed = value;
    }
    if (DEBUG_LEVEL >= 1) { // Печать информации о ключе и значении
      Serial.print(key); Serial.print(" = "); Serial.println(value);
    }
  }
}
