# pragma once

#include "types.h"
#include "constants.h"

namespace INIT {
  // inicializar los motores
  Types::Motor init_motor(const int EN, const int IN1, const int IN2);

  // inicializar IRs
  Types::IR init_IR(const int pin);

  // inicializar robot
  Types::Robot init_robot();

}

namespace MOVIMIENTO {
  // considerando que motor1 es IZQUIERDA
  // mover frente 
  void moverFrente(Types::Robot);
  // gira derecha
  void giraDerecha(Types::Robot);
  // gira izquierda
  void giraIzquierda(Types::Robot);

}