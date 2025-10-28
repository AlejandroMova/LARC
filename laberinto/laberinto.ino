#include <funciones.h>
#include <constants.h>
#include <types.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// inicializar robot y MPU
Types::Robot robot = INIT::init_robot();
Adafruit_MPU6050 mpu;

void setup() {
  // setup de MPU y motores
  pinMode(CONSTANTS::ENA, OUTPUT);
  pinMode(CONSTANTS::IN1, OUTPUT);
  pinMode(CONSTANTS::IN2, OUTPUT);
  pinMode(CONSTANTS::ENB, OUTPUT);
  pinMode(CONSTANTS::IN3, OUTPUT);
  pinMode(CONSTANTS::IN4, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("No se encontro MPU 6050");
  } while (1) {
    delay(10);
  }
  Serial.println("MPU6050 encontrado");

}

void loop() {
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);
  // necesitamos calcular el pitch 

  // aceleraciones en m/s^2
  float AccY = a.acceleration.y;
  float AccZ = a.acceleration.z;

  // conseguir pitch con la tangente de y y z
  float anguloPitchAcc = atan2(a.acceleration.y, a.acceleration.z);

  if (anguloPitchAcc > 0) {
    // vamos empezando el sube y baja
    // nos movemos hacia enfrente normal, pero de todos modos velocidad baja
    MOVIMIENTO::moverFrente(robot, 180);
  } else if (anguloPitchAcc < 0) {
    // ya vamos de bajada, entonces tenemos que ir más lento
    MOVIMIENTO::moverFrente(robot, 130);

    if (anguloPitchAcc > -1 && anguloPitchAcc < 1) {
    // ya estamos en el final
      MOVIMIENTO::frenar(robot);
  }
  } 


}