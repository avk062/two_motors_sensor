//#define DRIVER_STEP_TIME 5  // меняем задержку на 10 мкс
#include "GyverStepper.h"
#include "GyverTimers.h"

#define MAX_SPEED 5333      // Максимальная скорость для моторов
#define MIN_SPEED 400       // Ограничение минимальной скорости
#define AccelerationK1 1.5    // Ускорение мотора 1
#define AccelerationK2 1.5  // Ускорение мотора 2
//============================
/*    НАСТРОЙКА МОТОР 1*/
#define STEPS1 800
#define pin_PUL1 2            // PUL+ мотора 1
#define pin_DIR1 3            // DIR+ мотора 1
#define pin_START1 4          // Пин датчика мешка мотора 1
#define pin_POSITION1 5       // Пин Позиционирования мотора 1
#define pin_SETT1 14          // Пин потенциометра скорости мотора 1
//===========================
/*    НАСТРОЙКА МОТОР 2*/
#define STEPS2 800
#define pin_PUL2 6            // PUL+ мотора 2
#define pin_DIR2 7            // DIR+ мотора 2
#define pin_START2  8         // Пин датчика мешка мотора 2
#define pin_POSITION2 9       // Пин Позиционирования мотора 2
#define pin_SETT2 15          // Пин потенциометра скорости мотора 2

/// НАСТРОЙКА КИНЕМАТИКИ
#define Z_steper 19           // Количество зубьев малой звезды
#define Z_drum 39             // Количество зубьев большой звезды
#define deg_plus 45            // запас хода в градусах 5-45

uint16_t target_pos = Z_drum / Z_steper * (360 + deg_plus);

//Инициализируем моторы
GStepper<STEPPER2WIRE> stepper1(STEPS1, pin_PUL1, pin_DIR1);
GStepper<STEPPER2WIRE> stepper2(STEPS2, pin_PUL2, pin_DIR2);

uint32_t speedM1 = 0; //Расчетная скорость мотора 1
uint32_t speedM2 = 0; //Расчетная скорость мотора 2

bool startM1 = false;
bool calibr1 = false;
bool startM2 = false;
bool calibr2 = false;


void setup() {
  Serial.begin(9600);

//  Serial.print("цель - ");
//  Serial.print(target_pos);
//  Serial.println("");


  pinMode(pin_SETT1, INPUT);
  pinMode(pin_START1, INPUT_PULLUP);
  pinMode(pin_POSITION1, INPUT_PULLUP);
  pinMode(pin_SETT2, INPUT);
  pinMode(pin_START2, INPUT_PULLUP);
  pinMode(pin_POSITION2, INPUT_PULLUP);

  speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
  speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);

  setStartMotor();  //  Выводим моторы на стартовую позицию




  // мотор 1 // режим следования к целевй позиции
  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setCurrentDeg(0);

  // установка макс. скорости в шагах/сек
  stepper1.setMaxSpeed(speedM1);

  // установка ускорения в шагах/сек/сек
  stepper1.setAcceleration(speedM1 * AccelerationK1);

  // мотор 2 просто вращается
  stepper2.setRunMode(FOLLOW_POS);

  // установка макс. скорости в шагах/сек
  stepper2.setMaxSpeed(speedM2);

  // установка ускорения в шагах/сек/сек
  stepper2.setAcceleration(speedM2 * AccelerationK2);


  // настраиваем прерывания с периодом, при котором
  // система сможет обеспечить максимальную скорость мотора.
  // Для большей плавности лучше лучше взять период чуть меньше, например в два раза


  Timer2.setPeriod(200);
  //Timer2.setFrequency(20000);

  // взводим прерывание
  Timer2.enableISR(CHANNEL_A);
  Timer2.enableISR(CHANNEL_B);
 


}

// обработчик
ISR(TIMER2_A) {
  stepper1.tick(); // тикаем тут

}

ISR(TIMER2_B) {
  stepper2.tick(); // тикаем тут
}





void loop() {

  static uint32_t tmr2;
  if (millis() - tmr2 >= 10000) {
    tmr2 = millis();
    //    if (stepper1.getState() || stepper2.getState()) {
    //      Serial.print (millis() * 0.001);
    //      Serial.print ("---- ");
    //      Serial.print ("speed1 - ");
    //      Serial.print (stepper1.getSpeed());
    //      Serial.print("     |      ");
    //      Serial.print ("speed2 - ");
    //      Serial.println (stepper2.getSpeed());
    //    }

    speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
    stepper1.setMaxSpeed(speedM1);
    stepper1.setAcceleration(speedM1 * AccelerationK1);


    speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);
    stepper2.setMaxSpeed(speedM2);
    stepper2.setAcceleration(speedM2 * AccelerationK2);

  }

  run_Stepper1();
  run_Stepper2();

  


}

/*----  Установка начальной позиции для моторов  ---*/
void setStartMotor() {
  //Калибровка моторов
  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setAcceleration(speedM1 * 1.1 * 0.3);
  stepper1.setSpeed(speedM1 * 0.3);

  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setAcceleration(speedM2 * 1.1 * 0.3);
  //stepper2.reverse(false);
  stepper2.setSpeed(speedM2 * 0.3);


  while (!digitalReadFast(pin_POSITION1)) { // Пока не доехали до датчика
    stepper1.tick();
  }
  stepper1.reset();// останавливаем мотор

  while (!digitalReadFast(pin_POSITION2)) { // Пока не доехали до датчика
    stepper2.tick();
  }
  stepper2.reset();// останавливаем мотор
  //===== Калибровка моторов
}

void run_Stepper1() {
  // проверяем наличие старта для первого мотора

  if (!digitalReadFast(pin_START1) && !stepper1.getState()) {
    startM1 = true;
    calibr1 = true;
  }

  // если есть старт
  if (startM1) {
    if (!stepper1.getState()) {
      stepper1.setTargetDeg(target_pos);
      calibr1 = true;
    }
  }

  if (!stepper1.getState() && calibr1) {
    stepper1.setTargetDeg(target_pos - deg_plus * 3);
    calibr1 = false;



  }
  if (digitalReadFast(pin_POSITION1) && !calibr1) {
    startM1 = false;
    calibr1 = false;
    stepper1.reset();
    stepper1.setCurrent(0);
  }

}


void run_Stepper2() {

  // проверяем наличие старта для первого мотора

  if (!digitalReadFast(pin_START2) && !stepper2.getState()) {
    startM2 = true;
    calibr2 = true;
  }

  // если есть старт
  if (startM2) {
    if (!stepper2.getState()) {
      stepper2.setTargetDeg(target_pos);
      calibr2 = true;
    }
  }

  if (!stepper2.getState() && calibr2) {
    stepper2.setTargetDeg(target_pos - deg_plus * 3);
    calibr2 = false;



  }
  if (digitalReadFast(pin_POSITION2) && !calibr2) {
    startM2 = false;
    calibr2 = false;
    stepper2.reset();
    stepper2.setCurrent(0);
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
