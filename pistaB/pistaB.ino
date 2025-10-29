#include <funciones.h>
#include <constants.h>
#include <types.h>
#include <Adafruit_TCS34725.h>

// inicializamos sensor RGB y robot
Types::Robot robot = INIT::init_robot();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// creamos variables para PID
float Kp = 1.0; // proporcional
float Ki = 0.0; // integral
float Kd = 0.5; // derivada

// velocidad base de los motores 
int velocidadBase = 150;

// variables para el calculo
float error = 0;
float P = 0;
float I = 0;
float D = 0;
float errorAnterior = 0;
float correcionPID = 0;

// nueva variable para estado de linea
// true = linea negra sobre fondo blanco
// false = linea blanca sobre fondo negro
bool lineaEsNegra = true; 

// colores 
enum COLORES {
  AMARILLO,
  DESCONOCIDO
};


// funcion para detectar colores
COLORES detectColor(Adafruit_TCS34725 tcs) {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Evitar división por cero
  float sum = r + g + b;
  if (sum == 0) return DESCONOCIDO;

  float R = r / sum;
  float G = g / sum;
  float B = b / sum;

  // normalizamos valores basado en porcentaje
  // estos valores pueden necesitar ajuste
  if (R > 0.38 && G > 0.35 && B < 0.25) return AMARILLO;

  return DESCONOCIDO;
}

void setup() {
  Serial.begin(9600); // iniciamos monitor serial

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

  // iniciamos sensor de color
  if (!tcs.begin()) {
    Serial.println("No se encontro sensor TCS34725");
    while (1); // parar si no hay sensor
  }

  // hacemos que los motores vayan hacia adelante
  MOVIMIENTO::moverFrente(robot, velocidadBase);
}

void loop() {
  
  // crear checkpoint
  // el checkpoint amarillo tiene prioridad
  if (detectColor(tcs) == AMARILLO) {
    MOVIMIENTO::frenar(robot); // frenamos
    while (1); // paramos por completo
    // no continuamos hasta reiniciar el robot
  }

  // leer infrarrojos
  // asumimos 1 = ve linea negra, 0 = ve fondo blanco
  int stateLeft = digitalRead(robot.leftIR.pin);
  int stateCenter = digitalRead(robot.centerIR.pin);
  int stateRight = digitalRead(robot.rightIR.pin);

  // calcular el error
  
  if (lineaEsNegra) {
    // para linea negra
    // 1 es linea, 0 es fondo
    
    if (stateLeft == LOW && stateCenter == HIGH && stateRight == LOW) {
      error = 0; // Centrado (0 1 0)
    } else if (stateLeft == LOW && stateCenter == HIGH && stateRight == HIGH) {
      error = 1; // (0 1 1)
    } else if (stateLeft == LOW && stateCenter == LOW && stateRight == HIGH) {
      error = 2; // (0 0 1)
    } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == LOW) {
      error = -1; // (1 1 0)
    } else if (stateLeft == HIGH && stateCenter == LOW && stateRight == LOW) {
      error = -2; // (1 0 0)
    } else if (stateLeft == LOW && stateCenter == LOW && stateRight == LOW) {
      // perdimos la linea (0 0 0)
      if (errorAnterior > 0) {
        error = 3; 
      } else {
        error = -3;
      }
    } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == HIGH) {
      // interseccion O punto de inversion (1 1 1)
      MOVIMIENTO::frenar(robot);
      lineaEsNegra = false; // invertimos el estado
      errorAnterior = 0; // reseteamos PID
      I = 0;
      delay(100); // pausa corta
      MOVIMIENTO::moverFrente(robot, velocidadBase); // seguimos
      return; // salimos del loop
    }
    
  } else {
    // para linea blanca
    // 0 es linea, 1 es fondo
    
    if (stateLeft == HIGH && stateCenter == LOW && stateRight == HIGH) {
      error = 0; // Centrado (1 0 1)
    } else if (stateLeft == HIGH && stateCenter == LOW && stateRight == LOW) {
      error = 1; // (1 0 0)
    } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == LOW) {
      error = 2; // (1 1 0)
    } else if (stateLeft == LOW && stateCenter == LOW && stateRight == HIGH) {
      error = -1; // (0 0 1)
    } else if (stateLeft == LOW && stateCenter == HIGH && stateRight == HIGH) {
      error = -2; // (0 1 1)
    } else if (stateLeft == HIGH && stateCenter == HIGH && stateRight == HIGH) {
      // perdimos la linea (1 1 1)
      if (errorAnterior > 0) {
        error = 3;
      } else {
        error = -3;
      }
    } else if (stateLeft == LOW && stateCenter == LOW && stateRight == LOW) {
      // interseccion O punto de inversion (0 0 0)
      MOVIMIENTO::frenar(robot);
      lineaEsNegra = true; // volvemos a linea negra
      errorAnterior = 0; // reseteamos PID
      I = 0;
      delay(100); // pausa corta
      MOVIMIENTO::moverFrente(robot, velocidadBase); // seguimos
      return; // salimos del loop
    }
  }

  // 4. CALCULAR CORRECCION PID
  // esta parte es igual que antes
  
  P = error;
  I = I + error;
  D = error - errorAnterior;
  errorAnterior = error;

  correcionPID = (Kp * P) + (Ki * I) + (Kd * D);

  // 5. APLICAR CORRECCION A MOTORES
  
  int velocidadMotorIzquierdo = velocidadBase + correcionPID;
  int velocidadMotorDerecho = velocidadBase - correcionPID;

  // asegurar que velocidades esten dentro del rango
  velocidadMotorIzquierdo = constrain(velocidadMotorIzquierdo, 0, 255);
  velocidadMotorDerecho = constrain(velocidadMotorDerecho, 0, 255);

  // aplicamos esta velocidad a los motores
  // IMPORTANTE: debe ser analogWrite para controlar velocidad (PWM)
  analogWrite(robot.motor1.EN, velocidadMotorIzquierdo);
  analogWrite(robot.motor2.EN, velocidadMotorDerecho);
  
  // (asumimos que los pines IN1, IN2, IN3, IN4 ya estan
  // configurados por MOVIMIENTO::moverFrente)
}
