#include <funciones.h>
#include <constants.h>
#include <types.h>


Types::Robot robot = INIT::init_robot();

// creamos variables para PID
float Kp = 1.0; // proporcional
float Ki = 0.0; // integral
float Kd = 0.5; // derivada

// velocidad base de los motores 
int velocidadBase = 150;

//variables para el calculo
float error = 0;
float P = 0;
float I = 0;
float D = 0;
float errorAnterior = 0;
float correcionPID = 0;

void setup() {

  // setup pines robot
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

  // hacemos que los motores vayan hacia adelante desde el setup
  MOVIMIENTO::moverFrente(robot, velocidadBase);


  
}

void loop() {
  // la logica del loop tiene que estar basado en PID
  // leer sensores
  int stateLeft = digitalRead(robot.leftIR.pin);
  int stateCenter = digitalRead(robot.centerIR.pin);
  int stateRight = digitalRead(robot.rightIR.pin);

  // calculamos el error
  // convertimos las lecturas de los 3 sensores en un solo valor de error
  // (0, 1, 0) error es igual a 0, porque esta centrado
  // (0, 1, 1), error = 1 (desviado a la izq)
  // (0, 0, 1), error = 2 (muy desviado a la izq)
  // (1, 1, 0) error = -1 (desv derecha)
  // (1, 0, 0) error = -2 (muy desv derecha)
  // (0, 0, 0) perdimos la linea, entonces utilizamos el ultimo error
  // (1, 1, 1) es una interseccion, tenemos que parar

  if (stateLeft == LOW && stateCenter == HIGH && stateRight == LOW) {
    error = 0; // Centrado
  } else if (stateLeft == LOW && stateCenter == HIGH && stateRight == HIGH) {
    error = 1;
  } else if (stateLeft == LOW && stateCenter == LOW && stateRight == HIGH) {
    error = 2;
  } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == LOW) {
    error = -1;
  } else if (stateLeft == HIGH && stateCenter == LOW && stateRight == LOW) {
    error = -2;
  } else if (stateLeft == LOW && stateCenter == LOW && stateRight == LOW) {
    // perdimos la linea
    // tenemos que girar hacia la ultima direccion que conocemos, pero bruscamente (por eso 3)
    if (errorAnterior > 0) {
      error = 3; // estaba a la izquierda, tenemos que girar hacia la derecha
    } else {
      error = -3; // estaba a la derecha, entonces tenemos que girar hacia la izq
    }
  } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == HIGH) {
    // interseccion, stop
    error = 0;
    MOVIMIENTO::frenar(robot);
    return; // Detiene los motores y sale del loop
  }

  // ahora calculamos error 
  // error proporcional
  P = error;
  // error integral (acumula)
  I = I + error;
  // derivativo, reacciona a la razon de cambio del error
  D = error - errorAnterior;
  // guardamos error para siguienteloop
  errorAnterior = error;

  // calcular correcion total, multiplicando por constante de "afinacion"
  correcionPID = (Kp * P) + (Ki * I) + (Kd * D);

  //aplicamos correcion a los motores
  // positivo, esta a la izquierda
  // girar derecha, motor izq rapido, motor derecho lento
  // negativo, esta a la derecha
  // girar izquierda, motor izquierdo lento, motor derecho rapido

  int velocidadMotorIzquierdo = velocidadBase + correcionPID;
  int velocidadMotorDerecho = velocidadBase - correcionPID;

  // asegurar que velocidades esten dentro del rango
  velocidadMotorIzquierdo = constrain(velocidadMotorIzquierdo, 0, 255);
  velocidadMotorDerecho = constrain(velocidadMotorDerecho, 0, 255);

  // aplicamos esta velocidad a los motores
  // motor izquierdo (considerando que motor1 es el izquierdo)
  digitalWrite(robot.motor1.EN, velocidadMotorIzquierdo);
  digitalWrite(robot.motor2.EN, velocidadMotorDerecho);

  






}
