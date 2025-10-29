#include <funciones.h>
#include <constants.h>
#include <types.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// inicializar robot y MPU
Types::Robot robot = INIT::init_robot();
Adafruit_MPU6050 mpu;

// variables para el filtro
float anguloPitch = 0.0;
unsigned long tiempoPrevio = 0;

// estados del robot
enum EstadoRobot {
  BUSCANDO_RAMPA,
  SUBIENDO,
  BAJANDO,
  FINALIZADO
};
EstadoRobot estado = BUSCANDO_RAMPA;


void setup() {
  Serial.begin(115200); // para ver el angulo en la compu

  // setup de motores
  pinMode(CONSTANTS::ENA, OUTPUT);
  pinMode(CONSTANTS::IN1, OUTPUT);
  pinMode(CONSTANTS::IN2, OUTPUT);
  pinMode(CONSTANTS::ENB, OUTPUT);
  pinMode(CONSTANTS::IN3, OUTPUT);
  pinMode(CONSTANTS::IN4, OUTPUT);

  // setup de mpu
  if (!mpu.begin()) {
    Serial.println("No se encontro MPU 6050");
    while (1) { // el bucle va adentro del if
      delay(10);
    }
  }
  Serial.println("MPU6050 encontrado");

  // arranca el tiempo
  tiempoPrevio = micros();
}

void loop() {
  
  // calcular el tiempo dt en segundos
  unsigned long tiempoActual = micros();
  float dt = (tiempoActual - tiempoPrevio) / 1000000.0;
  tiempoPrevio = tiempoActual;

  // conseguir datos del mpu
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  // --- calculo del filtro ---

  // angulo con acelerometro (en radianes)
  float anguloPitchAcc = atan2(a.acceleration.y, a.acceleration.z);

  // angulo con giroscopio (en radianes)
  float cambioAnguloGyro = g.gyro.x * dt;

  // fusion de angulos (filtro complementario)
  // este angulo ya es estable y no tiene ruido
  anguloPitch = 0.98 * (anguloPitch + cambioAnguloGyro) + 0.02 * (anguloPitchAcc);


  // --- logica de control por estado ---

  // convertir a grados para que sea mas facil de leer
  float anguloPitchGrados = anguloPitch * (180.0 / M_PI);

  Serial.println(anguloPitchGrados); // para depurar


  switch (estado) {
    
    case BUSCANDO_RAMPA:
      // el robot avanza normal
      MOVIMIENTO::moverFrente(robot, 180);
      
      // si se inclina mas de 15 grados, empezo a subir
      if (anguloPitchGrados > 15.0) {
        Serial.println("estado: subiendo");
        estado = SUBIENDO;
      }
      break;

    case SUBIENDO:
      // sigue subiendo con la misma velocidad
      MOVIMIENTO::moverFrente(robot, 180);

      // si el angulo se vuelve negativo, ya paso el centro
      if (anguloPitchGrados < -15.0) {
        Serial.println("estado: bajando");
        estado = BAJANDO;
      }
      break;

    case BAJANDO:
      // va de bajada, reducimos la velocidad para no chocar
      MOVIMIENTO::moverFrente(robot, 130);

      // si el angulo regresa a estar plano, ya llego
      if (abs(anguloPitchGrados) < 5.0) {
        Serial.println("estado: finalizado");
        estado = FINALIZADO;
      }
      break;

    case FINALIZADO:
      // llegamos al final, nos frenamos
      MOVIMIENTO::frenar(robot);
      break;
  }
}