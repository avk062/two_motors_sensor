// включаем быстрый профиль, 10 участков
#define GS_FAST_PROFILE 10
#include <GyverStepper2.h>
#include <GyverTimers.h>

/*  СКОРОСТЬ И УСКОРЕНИЯ - основные настройки системы */
#define MAX_SPEED 4800     // Максимальная скорость для моторов
#define MIN_SPEED 1200       // Ограничение минимальной скорости
#define AccelerationK1 2.5  // Ускорение мотора 1
#define AccelerationK2 2.5   // Ускорение мотора 2
/*=================================================================*/

/*    НАСТРОЙКА МОТОР 1*/
#define STEPS1 800
#define pin_PUL1 2            // PUL+ мотора 1
#define pin_DIR1 3            // DIR+ мотора 1
#define pin_START1 4          // Пин датчика мешка мотора 1
#define pin_POSITION1 5       // Пин Позиционирования мотора 1
#define pin_SETT1 14          // Пин потенциометра скорости мотора 1
/*=================================================================*/

/*    НАСТРОЙКА МОТОР 2*/
#define STEPS2 800
#define pin_PUL2 6            // PUL+ мотора 2
#define pin_DIR2 7            // DIR+ мотора 2
#define pin_START2  8         // Пин датчика мешка мотора 2
#define pin_POSITION2 9       // Пин Позиционирования мотора 2
#define pin_SETT2 15          // Пин потенциометра скорости мотора 2
/*==================================================================*/

/*НАСТРОЙКА КИНЕМАТИКИ*/
#define Z_steper 19           // Количество зубьев малой звезды для передаточного числа
#define Z_drum 39             // Количество зубьев большой звезды для передаточного числа
#define deg_plus  45          // запас хода в градусах 5-45 для основного прохода сквозь тело пластины датчика

uint16_t target_pos = Z_drum / Z_steper * (360 + deg_plus);   // целевая позиция мотора в градусах
/*==================================================================*/

int16_t speedM1 = 0; //Расчетная скорость мотора 1
int16_t speedM2 = 0; //Расчетная скорость мотора 2
bool startM1 = false;
bool calibr1 = false;
bool startM2 = false;
bool calibr2 = false;

//Инициализируем моторы
GStepper2<STEPPER2WIRE> stepper1(STEPS1, pin_PUL1, pin_DIR1);
GStepper2<STEPPER2WIRE> stepper2(STEPS2, pin_PUL2, pin_DIR2);

void setup() {
  Serial.begin(115200);

  pinMode(pin_SETT1, INPUT);
  pinMode(pin_START1, INPUT_PULLUP);
  pinMode(pin_POSITION1, INPUT_PULLUP);
  pinMode(pin_SETT2, INPUT);
  pinMode(pin_START2, INPUT_PULLUP);
  pinMode(pin_POSITION2, INPUT_PULLUP);

  // определеям скорости моторов согласно потенциометра
  speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
  speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);

  // просмотр установленніх скоростей
  Serial.println("");
  Serial.print("Скорость малого ");
  Serial.print(speedM1 / STEPS1 * 60);
  Serial.print(" об/мин");
  Serial.println("");
  Serial.print("Скорость великого ");
  Serial.print(speedM2 / STEPS2 * 60);
  Serial.print(" об/мин");

  homing();// стартовая настройка моторов после включения

  // взводим прерывание
  Timer1.enableISR(CHANNEL_A);
  Timer2.enableISR(CHANNEL_A);

  stepper1.setMaxSpeed(speedM1);
  stepper1.setAcceleration(speedM1 * AccelerationK1);
  stepper2.setMaxSpeed(speedM2);
  stepper2.setAcceleration(speedM2 * AccelerationK2);
}

// обработчик
ISR(TIMER1_A) {
  if (stepper1.tickManual()) Timer1.setPeriod(stepper1.getPeriod());
  else Timer1.stop();

}

ISR(TIMER2_A) {
  if (stepper2.tickManual()) Timer2.setPeriod(stepper2.getPeriod());
  else Timer2.stop();
}

void loop() {
  while (1) {
    
    static uint32_t tmr2;
    if (millis() - tmr2 >= 500) {// раз в пол секунды читаем значение скоростей
      tmr2 = millis();
      speedM1 = map(analogRead(pin_SETT1), 0, 1023, MIN_SPEED, MAX_SPEED);
      speedM2 = map(analogRead(pin_SETT2), 0, 1023, MIN_SPEED, MAX_SPEED);
    }
    
    run_Stepper1(); //обработка для первого мотора и датчиков
    run_Stepper2(); //обработка для первого мотора и датчиков
  }
}

/*Стартовая настройка моторов*/
void homing() {

  if (digitalReadFast(pin_POSITION1)) {     // Если при включении датчик видно
    stepper1.setSpeed(speedM1 * 0.3);       // просто включаем мотор
    while (digitalReadFast(pin_POSITION1))   //  пока не уйдет из датчика
    { stepper1.tick();
    }
    stepper1.brake();
    delay(10);
  }
  if (!digitalReadFast(pin_POSITION1))
  { // если нет датчика
    stepper1.setSpeed(speedM1 * 0.3);           // просто включаем мотор
    while (!digitalReadFast(pin_POSITION1)) {   // пока не увидит
      stepper1.tick();               // крутим
    }
    stepper1.brake();                // тормозим, приехали к датчику начало
    delay(10);

    // и тут же проезжаем его насквозь
    if (digitalReadFast(pin_POSITION1)) {
      stepper1.setSpeed(speedM1 * 0.3);
      while (digitalReadFast(pin_POSITION1))   //  пока не уйдет из датчика
      {
        stepper1.tick();
      }
      stepper1.brake();                       //  останавливаем
      delay(10);
    }

    if (!digitalReadFast(pin_POSITION1)) {       // проехали датчик
      stepper1.setSpeed(-speedM1 * 0.3);          // вращаем назад
      while (!digitalReadFast(pin_POSITION1)) {  // пока нет датчика
        stepper1.tick();
      }
      // РАБОЧЕЕ ПОЛОЖЭЕНИЕ
      stepper1.brake();                // тормозим
    }
  } stepper1.reset();   // сбрасываем координаты в 0


  if (digitalReadFast(pin_POSITION2)) {     // Если при включении датчик видно
    stepper2.setSpeed(speedM2 * 0.3);       // просто включаем мотор
    while (digitalReadFast(pin_POSITION2))   //  пока не уйдет из датчика
    { stepper2.tick();
    }
    stepper2.brake();
    delay(10);
  }
  if (!digitalReadFast(pin_POSITION2))
  { // если нет датчика
    stepper2.setSpeed(speedM2 * 0.3);           // просто включаем мотор
    while (!digitalReadFast(pin_POSITION2)) {   // пока не увидит
      stepper2.tick();               // крутим
    }
    stepper2.brake();                // тормозим, приехали к датчику начало
    delay(10);

    // и тут же проезжаем его насквозь
    if (digitalReadFast(pin_POSITION2)) {
      stepper2.setSpeed(speedM2 * 0.3);
      while (digitalReadFast(pin_POSITION2))   //  пока не уйдет из датчика
      {
        stepper2.tick();
      }
      stepper2.brake();                       //  останавливаем
      delay(10);
    }

    if (!digitalReadFast(pin_POSITION2)) {       // проехали датчик
      stepper2.setSpeed(-speedM2 * 0.3);          // вращаем назад
      while (!digitalReadFast(pin_POSITION2)) {  // пока нет датчика
        stepper2.tick();
      }
      // РАБОЧЕЕ ПОЛОЖЭЕНИЕ
      stepper2.brake();                // тормозим
    }
  } stepper2.reset();   // сбрасываем координаты в 0
}

void run_Stepper1() {
  // проверяем наличие старта для первого мотора
  stepper1.setMaxSpeed(speedM1);
  stepper1.setAcceleration(speedM1 * AccelerationK1);

  if (!digitalReadFast(pin_START1) && stepper1.getStatus() == 0) {
    startM1 = true;
    calibr1 = true;
    Timer1.setPeriod(stepper1.getPeriod() / 2);
  }
  // если есть старт
  if (startM1) {
    if (stepper1.getStatus() == 0) {
      stepper1.setTargetDeg(long(target_pos));
      Timer1.setPeriod(stepper1.getPeriod() / 2);
      calibr1 = true;
    }
  }
  if (!digitalReadFast(pin_POSITION1) && calibr1) {       // проехали датчик
    // stepper1.setMaxSpeed(speedM1*0.5);
    stepper1.setAcceleration(speedM1 * 0.5);
    stepper1.setTargetDeg(long(target_pos / 4));
    Timer1.setPeriod(stepper1.getPeriod() / 2);       // вращаем назад
    while (!digitalReadFast(pin_POSITION1)) {  // пока нет датчика
      //stepper1.tick();
    }
    // РАБОЧЕЕ ПОЛОЖЭЕНИЕ
    stepper1.brake();
    startM1 = false;
    calibr1 = false;
    stepper1.reset();
  }
}

void run_Stepper2() {
  // проверяем наличие старта для первого мотора
  stepper2.setMaxSpeed(speedM2);
  stepper2.setAcceleration(speedM2 * AccelerationK2);

  if (!digitalReadFast(pin_START2) && stepper2.getStatus() == 0) {
    startM2 = true;
    calibr2 = true;
    Timer2.setPeriod(stepper2.getPeriod() / 2);
  }
  // если есть старт
  if (startM2) {
    if (stepper2.getStatus() == 0) {
      stepper2.setTargetDeg(long(target_pos));
      Timer2.setPeriod(stepper2.getPeriod() / 2);
      calibr2 = true;
    }
  }
  if (!digitalReadFast(pin_POSITION2) && calibr2) {       // проехали датчик
    // stepper2.setMaxSpeed(speedM2*0.5);
    stepper2.setAcceleration(speedM2 * 0.5);
    stepper2.setTargetDeg(long(target_pos / 4));
    Timer2.setPeriod(stepper2.getPeriod() / 2);       // вращаем назад
    while (!digitalReadFast(pin_POSITION2)) {  // пока нет датчика
      //stepper2.tick();
    }
    // РАБОЧЕЕ ПОЛОЖЭЕНИЕ
    stepper2.brake();
    startM2 = false;
    calibr2 = false;
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
