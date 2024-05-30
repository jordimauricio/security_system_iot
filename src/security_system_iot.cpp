/*
 * Project Security System IOT
 * Author: Jordi Alejandro Mauricio de Leon
 * Date: 28/05/2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 *
 *
 * * MFRC522 - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * The library file MFRC522.h has a wealth of useful info. Please read it.
 * The functions are documented in MFRC522.cpp.
 *
 * Based on code Dr.Leong   ( WWW.B2CQSHOP.COM )
 * Created by Miguel Balboa (circuitito.com), Jan, 2012.
 * Rewritten by Søren Thing Andersen (access.thing.dk), fall of 2013 (Translation to English, refactored, comments, anti collision, cascade levels.)
 * Released into the public domain.
 *
 * Sample program showing how to read data from a PICC using a MFRC522 reader on the Arduino SPI interface.
 *----------------------------------------------------------------------------- empty_skull
 * Aggiunti pin per arduino Mega
 * add pin configuration for arduino mega
 * http://mac86project.altervista.org/
 ----------------------------------------------------------------------------- Nicola Coppola
 * Pin layout should be as follows:
 * Signal     Pin              Pin               Pin        Pin         Pin
 *            Arduino Uno      Arduino Mega      SPARK		  Photon 2    MFRC522 board   Colors
 * ---------------------------------------------------------------------------
 * Reset      9                5                 ANY (D2)	  D19         RST             White
 * SPI SS     10               53                ANY (A2)	  D18         SDA             Purple
 * SPI MOSI   11               51                A5         D15         MOSI            Green
 * SPI MISO   12               50                A4         D16         MISO            Brown
 * SPI SCK    13               52                A3		      D17         SCK             Blue
 *
 * The reader can be found on eBay for around 5 dollars. Search for "mf-rc522" on ebay.com.
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "SPI.h"
#include "MFRC522.h"
#include "Adafruit_DHT.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Declaraciones de funciones
int puerta(String input);
int luces(String input);
void toggleMode();
void soundBuzzer(int duration);

// RFID configuration
#define SS_PIN 18  // Define pin for Slave Select (SS)
#define RST_PIN 19 // Define pin for Reset (RST)

// Create MFRC522 instance with defined SS and RST pins
MFRC522 mfrc522(SS_PIN, RST_PIN);

// DHT11 configuration
#define DHTPIN 11     // what pin we're connected to
#define DHTTYPE DHT11 // DHT 11

DHT dht(DHTPIN, DHTTYPE);

const int maxUsers = 5;      // Maximum number of users
String storedUIDs[maxUsers]; // Array to store user UIDs
int userCount = 0;           // Counter for number of users

bool isWritingMode = false; // Mode flag: false for reading, true for writing

// Objects
Servo servo; // Servo object

// Pins to use on Particle Photon 2
int rele = 0;        // Relay pin
int servo1 = 1;      // Servo control pin
int buzzer = 6;      // Buzzer pin
int boton = 8;       // Button pin for open the door
int boton2 = 9;      // Button 2 pin for change RFID mode
int termometro = 10; // Thermometer pin
int magneto = 12;    // Magnet sensor pin

// Variables
String readUID = ""; // Variable to store read UID
// int temperatura = 0;    // Placeholder for temperature variable
int aperturaPuerta = 0; // Variable for door status
int encendidoLuces = 0; // Variable for lights status
int servoPosition = 0;  // Variable for servo position
double lectura = 0;     // Variable for temperature reading
double voltaje = 0;     // Variable for voltage calculation
double mvolts = 0;      // Variable for millivolts calculation
double gradosC = 0;     // Variable for temperature in Celsius

void setup()
{
  Serial.begin(9600); // Start serial communication at 9600 baud
  dht.begin();
  mfrc522.setSPIConfig();                              // Set SPI configuration for MFRC522
  SPI.begin();                                         // Initialize SPI bus
  mfrc522.PCD_Init();                                  // Initialize MFRC522
  Particle.variable("gradosC", gradosC);               // Expose gradosC variable to the cloud
  Particle.variable("aperturaPuerta", aperturaPuerta); // Expose aperturaPuerta variable to the cloud
  Particle.variable("encendidoLuces", encendidoLuces); // Expose encendidoLuces variable to the cloud
  Particle.function("puerta", puerta);                 // Register puerta function for cloud control
  Particle.function("luces", luces);                   // Register luces function for cloud control
  Particle.syncTime();                                 // Synchronize time with the cloud
  Time.zone(-6);                                       // Set time zone to -6
  pinMode(boton, INPUT_PULLUP);                        // Set button pin as input with pull-up
  pinMode(boton2, INPUT_PULLUP);                       // Set button 2 pin as input with pull-up
  pinMode(magneto, INPUT_PULLUP);
  pinMode(termometro, INPUT);
  pinMode(buzzer, OUTPUT);                                             // Set magnet sensor pin as input with pull-up
  pinMode(rele, OUTPUT);                                               // Set relay pin as output
  pinMode(servo1, OUTPUT);                                             // Set servo control pin as output
  servo.attach(servo1);                                                // Attach servo to pin
  servo.write(79);                                                     // Set initial servo position
  attachInterrupt(digitalPinToInterrupt(boton2), toggleMode, FALLING); // Attach interrupt to button 2 to toggle mode
  Serial.println("Ready to scan cards...");                            // Print ready message
  delay(1000);                                                         // Wait for 1 second
}

void loop()
{
  delay(3000); // Wait for 3 seconds
  // Read temperature from thermometer
  /*lectura = analogRead(termometro);
  voltaje = (lectura * 3.3) / 4095; // Calculate voltage
  mvolts = voltaje * 1000;          // Convert to millivolts
  gradosC = mvolts / 10.0;          // Convert to degrees Celsius

  Serial.println(gradosC);      // Print temperature in Celsius*/

  // Read temperature as Celsius
  gradosC = dht.getTempCelcius();
  Serial.println(gradosC);

  servoPosition = servo.read(); // Read the servo position
  // Check if a new card is present and if it can be read
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
  {
    return; // Exit loop if no new card or read fails
  }
  readUID = ""; // Restore de variable to empty
  //  Read card UID
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    readUID += String(mfrc522.uid.uidByte[i], HEX); // Append each byte of UID to readUID
  }
  Serial.println("Card UID: " + readUID); // Print card UID

  if (isWritingMode)
  {
    // Store UID if in writing mode
    if (userCount < maxUsers)
    {
      storedUIDs[userCount] = readUID; // Store UID in array
      userCount++;                     // Increment user count
      Serial.println("UID stored!");   // Print stored message
      soundBuzzer(100);                // Sound to indicate that the UID was stored
    }
    else
    {
      Serial.println("User limit reached!"); // Print limit reached message
    }
  }
  else
  {
    // Check UID if in reading mode
    for (int i = 0; i < userCount; i++)
    {
      if (storedUIDs[i] == readUID)
      {
        Serial.println("Access granted!"); // Print access granted message
        soundBuzzer(100);                  // Sound to indicate access granted
        if (servoPosition > 75 && servoPosition < 85)
        {
          puerta("on"); // Open door if within range
        }
        else if (servoPosition >= 0 && servoPosition <= 5)
        {
          puerta("off"); // Close door if within range
        }
        return; // Exit loop
      }
    }
    Serial.println("Access denied!"); // Print access denied message
    soundBuzzer(500);                 // Sound to indicate access denied
  }

  // Control servo position
  Serial.println(servoPosition);
  if (digitalRead(boton) == LOW)
  {
    if (servoPosition > 75 && servoPosition < 85)
    {
      puerta("on"); // Open door if within range
    }
    else if (servoPosition >= 0 && servoPosition <= 5)
    {
      puerta("off"); // Close door if within range
    }
  }
  else if (digitalRead(boton) == HIGH)
  {
    Serial.println("HIGH"); // Print high message if button is high
  }
  else
  {
    Serial.println("Incorrect value"); // Print error message for incorrect value
  }
}

void toggleMode()
{
  isWritingMode = !isWritingMode; // Toggle writing mode
  if (isWritingMode)
  {
    Serial.println("Writing mode activated"); // Print writing mode activated
  }
  else
  {
    Serial.println("Reading mode activated"); // Print reading mode activated
  }
  soundBuzzer(100); // Sound to indicate change of the mode
}

int puerta(String input)
{
  if (input == "on")
  {
    Serial.println("Opening"); // Print opening message
    for (int pos = 79; pos > 0; pos--)
    {
      servo.write(pos); // Move servo to open position
      delay(20);        // Wait 20 milliseconds
    }
    luces("on");        // Turn lights on
    aperturaPuerta = 1; // Set door status to open
    encendidoLuces = 1; // Set lights status to on
    return 1;           // Return success
  }
  else if (input == "off")
  {
    Serial.println("Closing"); // Print closing message
    for (int pos = 0; pos < 79; pos++)
    {
      servo.write(pos); // Move servo to closed position
      delay(20);        // Wait 20 milliseconds
    }
    luces("off");       // Turn lights off
    aperturaPuerta = 0; // Set door status to closed
    encendidoLuces = 0; // Set lights status to off
    return 0;           // Return success
  }
  return -1;   // Return error if input is invalid
  delay(3000); // Wait for 1 second
}

int luces(String input)
{
  if (input == "on")
  {
    digitalWrite(rele, HIGH); // Turn relay on
    encendidoLuces = 1;       // Set lights status to on
    return 1;                 // Return success
  }
  else if (input == "off")
  {
    digitalWrite(rele, LOW); // Turn relay off
    encendidoLuces = 0;      // Set lights status to off
    return 0;                // Return success
  }
  return -1;   // Return error if input is invalid
  delay(1000); // Wait for 1 second
}

void soundBuzzer(int duration)
{
  digitalWrite(buzzer, HIGH); // Turn on the buzzer
  delay(duration);            // Hold the buzzer on for a specific duration
  digitalWrite(buzzer, LOW);  // Turn off the buzzer
}