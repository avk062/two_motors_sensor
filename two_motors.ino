#include "GyverStepper.h"


#define MAX_SPEED 5000
#define MIN_SPEED 1
#define INI_SPEED 1000
#define AccelerationK1 1.1
#define AccelerationK2 1.1

#define STEPS1 800
#define pin_PUL1 2            // PUL+ первого шаговика
#define pin_DIR1 3            // DIR+ первого шаговика
#define pin_START1 4          // Пин датчика мешка - 1
#define pin_POSITION1 5       // Пин Позиционирования - 1
#define pin_SETT1 14          // Пин потенциометра скорости - 1

#define STEPS2 800
#define pin_PUL2 6            // PUL+ 2 шаговика
#define pin_DIR2 7            // DIR+ 2 шаговика
#define pin_START2  8         // Пин датчика мешка - 2
#define pin_POSITION2 9       // Пин Позиционирования - 2
#define pin_SETT2 17          // Пин потенциометра скорости - 2

//Инициалихируем моторы
GStepper<STEPPER2WIRE> stepper1(STEPS1, pin_PUL1, pin_DIR1);
GStepper<STEPPER2WIRE> stepper2(STEPS2, pin_PUL2, pin_DIR2);

int32_t speedM1 = 0;
int32_t speedM2 = 0;
void setup() {
  pinMode(pin_SETT1, INPUT);
  pinMode(pin_START1, INPUT_PULLUP);
  pinMode(pin_POSITION1, INPUT_PULLUP);
  pinMode(pin_SETT2, INPUT);
  pinMode(pin_START2, INPUT_PULLUP);
  pinMode(pin_POSITION2, INPUT_PULLUP);

  //Калибровка моторов
  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setSpeed(INI_SPEED);
  stepper1.setAcceleration(INI_SPEED *5);
  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setSpeed(INI_SPEED);
  stepper2.setAcceleration(INI_SPEED * 5);
  stepper2.reverse(true);
  while (!digitalReadFast(pin_POSITION1)) { // Пока не доехали до датчика
    stepper1.tick();
  }
  stepper1.reset();// останавливаем мотор

  while (!digitalReadFast(pin_POSITION2)) { // Пока не доехали до датчика
    stepper2.tick();
  }
  stepper2.reset();// останавливаем мотор
  //===== Калибровка моторов

  speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
  speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);


  Serial.begin(9600);
  Serial.print ("speed1 - ");
  Serial.println (speedM1);
  Serial.print ("speed2 - ");
  Serial.println (speedM2);

  // мотор 1 просто вращается
  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setMaxSpeed(speedM1);
  stepper1.setAcceleration(speedM1 * AccelerationK1);

  // мотор 2 просто вращается
  stepper2.setRunMode(FOLLOW_POS);
  stepper2.setMaxSpeed(speedM2);
  stepper2.setAcceleration(speedM2 * AccelerationK2);
  stepper2.reverse(true);
}

boolean startM1 = false;
boolean startM2 = false;
void loop() {

//  static uint32_t tmr2;
//  if (millis() - tmr2 > 1000) {
//    tmr2 = millis();
//    Serial.begin(9600);
//    Serial.print ("speed1 - ");
//    Serial.println (speedM1);
//    Serial.print ("speed2 - ");
//    Serial.println (speedM2);
//
//  speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
//  stepper1.setMaxSpeed(speedM1);
//  stepper1.setAcceleration(speedM1 * AccelerationK1);
//
//  speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);
//  stepper1.setMaxSpeed(speedM2);
//  stepper1.setAcceleration(speedM2 * AccelerationK2);
//  }

  run_Stepper1();
  run_Stepper2();



}

void run_Stepper1() {
  // проверяем наличие старта для первого мотора
  if (!digitalReadFast(pin_START1)) {
    startM1 = true;
  }
  if (startM1) {
    if (!stepper1.tick()) {
      stepper1.setTarget(3600);
    }
  }
  if (digitalReadFast(pin_POSITION1)) {
    startM1 = false;
    stepper1.stop();
    stepper1.reset();
  }

}

void run_Stepper2() {
  if (!digitalReadFast(pin_START2)) {
    startM2 = true;
  }
  if (startM2) {
    if (!stepper2.tick()) {
      stepper2.setTarget(3600);
    }
  }
  if (digitalReadFast(pin_POSITION2)) {
    startM2 = false;
    stepper2.stop();
    stepper2.reset();
  }
}




bool digitalReadFast(uint8_t pin) {
  if (pin < 8) {
    return bitRead(PIND, pin);
  } else if (pin < 14) {
    return bitRead(PINB, pin - 8);
  } else if (pin < 20) {
    return bitRead(PINC, pin - 14); // Return pin state
  }
}
