#include <funciones.h>
#include <constants.h>
#include <types.h>


Types::Robot robot = INIT::init_robot();

void setup() {

  // setup ines robot
  pinMode(CONSTANTS::ENA, OUTPUT);
  pinMode(CONSTANTS::IN1, OUTPUT);
  pinMode(CONSTANTS::IN2, OUTPUT);
  pinMode(CONSTANTS::ENB, OUTPUT);
  pinMode(CONSTANTS::IN3, OUTPUT);
  pinMode(CONSTANTS::IN4, OUTPUT);

  // setup pines IRs
  pinMode(CONSTANTS::centerIR, INPUT);
  pinMode(CONSTANTS::rightIR, INPUT);
  pinMode(CONSTANTS::leftIR, INPUT);

  //setup ultrasonicos
  // pinMode(CONSTANTS::echoLeft, INPUT);
  // pinMode(CONSTANTS::trigLeft, OUTPUT);

  // pinMode(CONSTANTS::echoCenter, INPUT);
  // pinMode(CONSTANTS::trigCenter, OUTPUT);

  // pinMode(CONSTANTS::echoRight, INPUT);
  // pinMode(CONSTANTS::echoCenter, OUTPUT);
}

void loop() {
  // infrarrojos
  const int leftIR = robot.leftIR.pin;
  const int rightIR = robot.rightIR.pin;
  const int centerIR = robot.centerIR.pin;

  // digital read de los sensores
  int leftState = digitalRead(leftIR);
  int centerState = digitalRead(centerIR);
  int rightState = digitalRead(rightIR);

  if (leftState == LOW && rightState == LOW && centerState == HIGH) {
    MOVIMIENTO::moverFrente(robot, 255);

  } else if (rightState == HIGH && leftState == LOW && centerState == LOW) {

    MOVIMIENTO::giraDerecha(robot, 255);

  } else if (rightState == LOW && leftState == HIGH && centerState == LOW) {

    MOVIMIENTO::giraIzquierda(robot, 255);
  } else {
    MOVIMIENTO::frenar(robot);
  }
}
