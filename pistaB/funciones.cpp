#include "types.h"
#include "constants.h"
#include "funciones.h"
#include <Arduino.h>

namespace INIT {

  // inicializar motor
Types::Motor init_motor(const int EN, const int IN1, const int IN2) {
    return Types::Motor{ EN, IN1, IN2 };
  }

  // inicializar IR 
Types::IR init_IR(const int pin) {
    return Types::IR{pin};
  }

  // inicializar robot 
Types::Robot init_robot() {
    return Types::Robot{
      // motores
      init_motor(CONSTANTS::ENA, CONSTANTS::IN1, CONSTANTS::IN2),
      init_motor(CONSTANTS::ENB, CONSTANTS::IN3, CONSTANTS::IN4),
      // IRs
      init_IR(CONSTANTS::leftIR),
      init_IR(CONSTANTS::rightIR),
      init_IR(CONSTANTS::centerIR)
    };
  
}
} // namespace INIT

namespace MOVIMIENTO {
  // mover frente 
  void moverFrente(Types::Robot robot){
    // ambos motores hacia frente
    // motor izquierdo
    analogWrite(robot.motor1.EN, 255);
    digitalWrite(robot.motor1.IN1, HIGH);
    digitalWrite(robot.motor2.IN2, LOW);
    // motor derecho
    analogWrite(robot.motor2.EN, 255);
    digitalWrite(robot.motor2.IN1, HIGH);
    digitalWrite(robot.motor2.IN2, LOW);

  }
  // gira derecha
  void giraDerecha(Types::Robot robot) {
    // motor izquierdo atras, derecho frente
    // motor izquierdo
    analogWrite(robot.motor1.EN, 255);
    digitalWrite(robot.motor1.IN1, LOW);
    digitalWrite(robot.motor2.IN2, HIGH);
    // motor derecho
    analogWrite(robot.motor2.EN, 255);
    digitalWrite(robot.motor2.IN1, HIGH);
    digitalWrite(robot.motor2.IN2, LOW);
  }
  
  // gira izquierda
  void giraIzquierda(Types::Robot robot) {
    // motor derecho atras, izquierdo frente
    // motor izquierdo
    analogWrite(robot.motor1.EN, 255);
    digitalWrite(robot.motor1.IN1, HIGH);
    digitalWrite(robot.motor2.IN2, LOW);
    // motor derecho
    analogWrite(robot.motor2.EN, 255);
    digitalWrite(robot.motor2.IN1, LOW);
    digitalWrite(robot.motor2.IN2, HIGH);
  }

}
