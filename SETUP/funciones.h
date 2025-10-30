# pragma once

#include "types.h"
#include "constants.h"
#include <Arduino.h>

namespace INIT {
  // inicializar los motores
  Types::Motor init_motor(const int EN, const int IN1, const int IN2, const int pinA, const int pinB);

  // inicializar IRs
  Types::IR init_IR(const int pin);

  // inicializar robot
  Types::Robot init_robot();

  // inicializar ultrasonico
  Types::UltraSonic init_ultrasonic();

}

namespace MOVIMIENTO {
  // considerando que motor1 es IZQUIERDA
  // mover frente 
  void moverFrente(Types::Robot, int velocidad);
  // gira derecha
  void giraDerecha(Types::Robot, int velocidad);
  // gira izquierda
  void giraIzquierda(Types::Robot, int velocidad);

  void frenar(Types::Robot);

}

namespace SENSORES {
  void trigUltraSonicos(Types::UltraSonic &left, Types::UltraSonic &center, Types::UltraSonic &right);
}

namespace PANTALLA {
  
void init(byte cols, byte rows);

void write(int col, int row, const char* message);
 
void clear();
}

namespace ENCODERS {
  
  // configura las interrupciones
  // debe llamarse en el setup() principal
  void setupInterrupciones(Types::Robot& robot);

  // resetea los contadores
  void resetTicks(Types::Robot& robot);
}

namespace PANTALLA {
  // ... (sin cambios) ...
  void init(byte cols, byte rows);
  void write(int col, int row, const char* message);
  void clear();
}