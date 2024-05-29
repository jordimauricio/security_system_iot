/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);


Servo servo;
int temperatura = 0;
int aperturaPuerta = 0;
int encendidoLuces = 0;
int boton = 0;
int magneto = 8;
int rele = 6;
int servo1 = 13;
int termometro = 12;
int servoPosition = 0;
double lectura = 0;
double voltaje = 0;
double mvolts = 0;
double gradosC = 0;

void setup() {
    Particle.variable("gradosC", gradosC);
    Particle.variable("aperturaPuerta", aperturaPuerta);
    Particle.variable("encendidoLuces", encendidoLuces);
    Particle.function("puerta", puerta);
    Particle.function("luces", luces);
    Particle.syncTime();
    Time.zone(-6);
    pinMode(boton, INPUT_PULLUP);
    pinMode(magneto, INPUT_PULLUP);
    pinMode(rele, OUTPUT);
    pinMode(servo1, OUTPUT);
    servo.attach(servo1);
    servo.write(79);
    Serial.begin(9600);
    delay(1000);
}

void loop() {
    lectura = analogRead(termometro);
    voltaje = (lectura * 3.3) / 4095;
    mvolts = voltaje  * 1000;
    gradosC = mvolts / 10.0;
    
    
    servoPosition = servo.read();
    Serial.println(servoPosition);
    if(digitalRead(boton) == LOW){
        if (servoPosition > 75 && servoPosition < 85) { // Si la posicion del servo es igual a 180 grados
            puerta("on");   
        } 
        else if (servoPosition >= 0 && servoPosition <= 5){
            puerta("off");
        }
    } else if(digitalRead(boton) == HIGH)  {
        Serial.println("HIGH");
    }else {
        Serial.println("Valor incorrecto");
    }
    Serial.println(gradosC);
    delay(1000);
}

int puerta(String input){
    if (input == "on") {
        Serial.println("Encendido");
        for (int pos = 79; pos > 0; pos--) { 
          servo.write(pos);
          delay(20); 
        }    
        luces("on");
        aperturaPuerta = 1;
        encendidoLuces = 1;
        return 1;
    }
    // look for the matching argument "OFF"
    else if (input == "off") {
        Serial.println("Apagado");
        for (int pos = 0; pos < 79; pos++) {
            servo.write(pos); 
            delay(20); 
        }
        luces("off");
        aperturaPuerta = 0;
        encendidoLuces = 0;
        return 0;
    }
    return -1;
    delay(1000);
}

int luces(String input){
    if (input == "on") {
        digitalWrite(rele, HIGH);
        encendidoLuces = 1;
        return 1;
    }
    // look for the matching argument "OFF"
    else if (input == "off") {
        digitalWrite(rele, LOW);
        encendidoLuces = 0;
        return 0;
    }
    return -1;
    delay(1000);
}