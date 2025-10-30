#include <Arduino.h>
#include "types.h"
#include "constants.h"
#include "funciones.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// para encoders
static Types::Motor* _motor1_isr_ptr = nullptr;
static Types::Motor* _motor2_isr_ptr = nullptr;


// ISR para el motor 1 (izquierda)
void isr_motor1() {
  if (_motor1_isr_ptr != nullptr) {
    // leemos el pin B para saber la direccion
    if (digitalRead(_motor1_isr_ptr->pinEncoderB) == LOW) {
      _motor1_isr_ptr->ticks++;  // girando en un sentido
    } else {
      _motor1_isr_ptr->ticks--;  // girando en sentido opuesto
    }
  }
}

// ISR para el motor 2 (derecha)
void isr_motor2() {
  if (_motor2_isr_ptr != nullptr) {
    // leemos el pin B para saber la direccion
    if (digitalRead(_motor2_isr_ptr->pinEncoderB) == LOW) {
      _motor2_isr_ptr->ticks++;  // girando en un sentido
    } else {
      _motor2_isr_ptr->ticks--;  // girando en sentido opuesto
    }
  }
}

namespace INIT {

// inicializar motor
Types::Motor init_motor(const int EN, const int IN1, const int IN2, const int pinA, const int pinB) {
  // 0 para inicializar ticks en 0
  return Types::Motor{ EN, IN1, IN2, pinA, pinB, 0 };
}
// inicializar IR
Types::IR init_IR(const int pin) {
  return Types::IR{ pin };
}

// inicializar ultrasonico
Types::UltraSonic init_ultrasonic(int echo, int trig) {
  return Types::UltraSonic{ echo, trig };
}


// inicializar robot
Types::Robot init_robot() {
  return Types::Robot{
    // motores
    init_motor(CONSTANTS::ENA, CONSTANTS::IN1, CONSTANTS::IN2, CONSTANTS::M1_ENCODER_A, CONSTANTS::M1_ENCODER_B),
    init_motor(CONSTANTS::ENB, CONSTANTS::IN3, CONSTANTS::IN4, CONSTANTS::M2_ENCODER_A, CONSTANTS::M2_ENCODER_B),
    // IRs
    init_IR(CONSTANTS::leftIR),
    init_IR(CONSTANTS::rightIR),
    init_IR(CONSTANTS::centerIR),
    // ultrasonics
    init_ultrasonic(CONSTANTS::echoLeft, CONSTANTS::trigLeft),
    init_ultrasonic(CONSTANTS::echoCenter, CONSTANTS::trigCenter),
    init_ultrasonic(CONSTANTS::echoRight, CONSTANTS::trigRight),

  };
}
}  // namespace INIT

namespace MOVIMIENTO {
// mover frente
void moverFrente(Types::Robot robot, int velocidad) {
  // ambos motores hacia frente
  // motor izquierdo
  analogWrite(robot.motor1.EN, velocidad);
  digitalWrite(robot.motor1.IN1, HIGH);
  digitalWrite(robot.motor1.IN2, LOW);
  // motor derecho
  analogWrite(robot.motor2.EN, velocidad);
  digitalWrite(robot.motor2.IN1, HIGH);
  digitalWrite(robot.motor2.IN2, LOW);
}
// gira derecha
void giraDerecha(Types::Robot robot, int velocidad) {
  // motor izquierdo atras, derecho frente
  // motor izquierdo
  analogWrite(robot.motor1.EN, velocidad);
  digitalWrite(robot.motor1.IN1, LOW);
  digitalWrite(robot.motor1.IN2, HIGH);
  // motor derecho
  analogWrite(robot.motor2.EN, velocidad);
  digitalWrite(robot.motor2.IN1, HIGH);
  digitalWrite(robot.motor2.IN2, LOW);
}

// gira izquierda
void giraIzquierda(Types::Robot robot, int velocidad) {
  // motor derecho atras, izquierdo frente
  // motor izquierdo
  analogWrite(robot.motor1.EN, velocidad);
  digitalWrite(robot.motor1.IN1, HIGH);
  digitalWrite(robot.motor1.IN2, LOW);
  // motor derecho
  analogWrite(robot.motor2.EN, velocidad);
  digitalWrite(robot.motor2.IN1, LOW);
  digitalWrite(robot.motor2.IN2, HIGH);
}


void frenar(Types::Robot robot) {
  analogWrite(robot.motor1.EN, 0);
  analogWrite(robot.motor2.EN, 0);
}

void avanzarDistancia(Types::Robot& robot, int velocidad, int ticks_objetivo){
  // reseteamos ticks
  ENCODERS::resetTicks(robot);
  
  analogWrite(robot.motor1.EN, velocidad);
  digitalWrite(robot.motor1.IN1, HIGH);
  digitalWrite(robot.motor1.IN2, LOW);

  analogWrite(robot.motor2.EN, velocidad);
  digitalWrite(robot.motor2.IN1, HIGH);
  digitalWrite(robot.motor2.IN2, LOW);

  while (robot.motor1.ticks < ticks_objetivo || robot.motor2.ticks < ticks_objetivo) {
    // solo esperamos y que siga avanzando
  }

  // cuando llegamos a los ticks, frenamos el robot
  frenar(robot);
  
}

}

namespace PANTALLA {

// este pointer es solo para aqui dentro
LiquidCrystal_I2C* _lcd_ptr = nullptr;

// esta funcion tambien solo es para el namespace
byte encontrarI2C() {
  byte error, address;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      return address;  // se encontro
    }
  }
  return 0;  // no se encontro
}

// --- Funciones públicas (implementación) ---

void init(byte cols, byte rows) {
  byte addr = encontrarI2C();

  if (addr == 0) {
    Serial.println("NO SE ENCONTRO I2C");
    Serial.println("Revisar conexiones.");
    while (true)
      ;  // CONGELAR si no se encuentra
  }

  Serial.print("Dispositivo I2C encontrado en 0x");
  Serial.println(addr, HEX);

  // crear objeto en variable escondida (pointer)
  _lcd_ptr = new LiquidCrystal_I2C(addr, cols, rows);

  // utilizar pointer para inicializar
  _lcd_ptr->init();
  _lcd_ptr->clear();
  _lcd_ptr->backlight();
  _lcd_ptr->setCursor(0, 0);
  _lcd_ptr->print("LCD Listo");
  delay(1000);
  _lcd_ptr->clear();
}

void write(int col, int row, const char* message) {
  // solo escribe si el puntero ha sido inicializado
  if (_lcd_ptr != nullptr) {
    _lcd_ptr->setCursor(col, row);
    _lcd_ptr->print(message);
  }
}

void clear() {
  // borrar la pantalla
  if (_lcd_ptr != nullptr) {
    _lcd_ptr->clear();
  }
}

}  // namespace PANTALLA

namespace ENCODERS {

// esta funcion guarda los punteros y configura las interrupciones
void setupInterrupciones(Types::Robot& robot) {
  // guarda los punteros para que las ISR los usen
  _motor1_isr_ptr = &robot.motor1;
  _motor2_isr_ptr = &robot.motor2;

  // configura los pines de los encoders como entrada
  pinMode(robot.motor1.pinEncoderA, INPUT);
  pinMode(robot.motor1.pinEncoderB, INPUT);
  pinMode(robot.motor2.pinEncoderA, INPUT);
  pinMode(robot.motor2.pinEncoderB, INPUT);

  // configura las interrupciones
  // digitalPinToInterrupt() convierte el numero de pin (ej. 2) al numero de interrupcion (ej. 0)
  // se activa en CADA cambio (CHANGE)
  attachInterrupt(digitalPinToInterrupt(robot.motor1.pinEncoderA), isr_motor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(robot.motor2.pinEncoderA), isr_motor2, CHANGE);
}

void resetTicks(Types::Robot& robot) {
  // detiene interrupciones temporalmente para evitar corrupcion
  noInterrupts();
  robot.motor1.ticks = 0;
  robot.motor2.ticks = 0;
  interrupts();  // reactiva interrupciones
}
}  // fin namespace ENCODERS