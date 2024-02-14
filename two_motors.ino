#include "GyverStepper.h"
#include "GyverTimers.h"

#define MAX_SPEED 13300       // Максимальная скорость для моторов
#define MIN_SPEED 3000        // Ограничение минимальной скорости
#define AccelerationK1 1.8    // Ускорение мотора 1
#define AccelerationK2 1.8    // Ускорение мотора 2
//============================
/*    НАСТРОЙКА МОТОР 1*/
#define STEPS1 1600
#define pin_PUL1 2            // PUL+ мотора 1
#define pin_DIR1 3            // DIR+ мотора 1
#define pin_START1 4          // Пин датчика мешка мотора 1
#define pin_POSITION1 5       // Пин Позиционирования мотора 1
#define pin_SETT1 14          // Пин потенциометра скорости мотора 1
//===========================
/*    НАСТРОЙКА МОТОР 2*/
#define STEPS2 1600
#define pin_PUL2 6            // PUL+ мотора 2
#define pin_DIR2 7            // DIR+ мотора 2
#define pin_START2  8         // Пин датчика мешка мотора 2
#define pin_POSITION2 9       // Пин Позиционирования мотора 2
#define pin_SETT2 15          // Пин потенциометра скорости мотора 2

//Инициализируем моторы
GStepper<STEPPER2WIRE> stepper1(STEPS1, pin_PUL1, pin_DIR1);
GStepper<STEPPER2WIRE> stepper2(STEPS2, pin_PUL2, pin_DIR2);

uint32_t speedM1 = 0; //Расчетная скорость мотора 1
uint32_t speedM2 = 0; //Расчетная скорость мотора 2


void setup() {
  
  pinMode(pin_SETT1, INPUT);
  pinMode(pin_START1, INPUT_PULLUP);
  pinMode(pin_POSITION1, INPUT_PULLUP);
  pinMode(pin_SETT2, INPUT);
  pinMode(pin_START2, INPUT_PULLUP);
  pinMode(pin_POSITION2, INPUT_PULLUP);

  speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
  speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);

    setStartMotor();  //  Выводим моторы на стартовую позицию


  Serial.begin(9600);
 
  // мотор 1 просто вращается
  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setAcceleration(speedM1 * AccelerationK1);
  //stepper1.setSpeed(speedM1);


  // мотор 2 просто вращается
  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setAcceleration(speedM2 * AccelerationK2);
  //stepper2.setSpeed(speedM2);
  stepper2.reverse(true);


  // настраиваем прерывания с периодом, при котором
  // система сможет обеспечить максимальную скорость мотора.
  // Для большей плавности лучше лучше взять период чуть меньше, например в два раза


  Timer2.setPeriod(50);
  //Timer2.setFrequency(20000);

  // взводим прерывание
  Timer2.enableISR(CHANNEL_A);
  Timer2.enableISR(CHANNEL_B);
  Timer2.phaseShift(CHANNEL_B,180);


}

// обработчик
ISR(TIMER2_A) {
  stepper1.tick(); // тикаем тут
}

ISR(TIMER2_B) {
  stepper2.tick(); // тикаем тут
}

boolean startM1 = false;
boolean startM2 = false;


void loop() {

  static uint32_t tmr2;
  if (millis() - tmr2 >= 50) {
    tmr2 = millis();
   if (stepper1.getState() || stepper2.getState()){
    Serial.print (millis()*0.001);
    Serial.print ("---- ");
    Serial.print ("speed1 - ");
    Serial.print (stepper1.getSpeed());
    Serial.print("     |      ");
    Serial.print ("speed2 - ");
    Serial.println (stepper2.getSpeed());
   }
    
      speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
    stepper1.setAcceleration(speedM1 * AccelerationK1);
    if(stepper1.getState()) stepper1.setSpeed(speedM1);
    
      speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);
     stepper2.setAcceleration(speedM1 * AccelerationK2);
     if(stepper2.getState()) stepper2.setSpeed(speedM2);
  }

  run_Stepper1();
  run_Stepper2();



}

/*----  Установка начальной позиции для моторов  ---*/
void setStartMotor() {
  //Калибровка моторов
  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setAcceleration(speedM1*0.25);
  stepper1.setSpeed(speedM1*0.25);

  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setAcceleration(speedM2*0.5);
  stepper2.setSpeed(speedM2*0.25);
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
}

void run_Stepper1() {

  // проверяем наличие старта для первого мотора
  if (!digitalReadFast(pin_START1)) {
    startM1 = true;
   
  }
  if (startM1) {
    if (!stepper1.getState()) {
      //stepper1.setTarget(3600);
      stepper1.setSpeed(speedM1);
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
    if (!stepper2.getState()) {
      // stepper2.setTarget(3600);
      stepper2.setSpeed(speedM2);
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
