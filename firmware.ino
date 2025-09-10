#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include "GravityTDS.h"
#include <CapacitiveSensor.h>

/* Pin mapping
2 -> DHT22
3 -> Capacitive send
4 -> Capacitive receive (touch plate)
6 -> Pump led
7 -> Pump
9 -> HC-SR04 Trigger
10 -> HC-SR04 Echo
A1 -> TDS
A4 -> SDA
A5 -> SCL
*/

// === CONSTANTS === 
//#define DEBUG true
// Serial communications
#ifdef DEBUG
  #define BAUDRATE 9600
#endif
// Temperature and humidity sensor
#define DHT_TYPE DHT22
const int DHT_PIN = 2;
// LCD Display 20x4
#define LCD_ADDR 0x27 // May be 0x3F for some models
// Conductivity sensor
#define TDS_PIN A1
#define TDS_A_REF 5.0
#define ADC_RANGE 1024
// Water level sensor
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
// Pump control
const int PUMP_PIN = 7;
const int PUMP_LED_PIN = 6;
enum PUMP_STATE {PUMP_ON, PUMP_OFF};
#ifdef DEBUG
  #define ON_TIMEOUT 1UL // 1 minutes
  #define OFF_TIMEOUT 2UL // 2 minutes
#else
  #define ON_TIMEOUT 10UL // 10 minutes
  #define OFF_TIMEOUT 60UL // 60 minutes
#endif
// Capacitive (touch) sensor
const int CAP_SENS_SEND = 3; // Resistor
const int CAP_SENS_REC = 4; // Resistor + touch plate
const int CAP_THRES = 1000; // Detection threshold
const int CAP_READ_PERIOD = 500; // Check sensor every 500ms
// Misc
const unsigned long SENSORS_TIMEOUT = 30000UL; // Turn off LCD and sensors on timeout
const unsigned long SENSOR_READ_PERIOD = 5000UL; // Acquire every 5 seconds
const unsigned long MINUTE = 60000UL; // 1 minute in milliseconds
enum PRINT_MODE {PRINT_SENSORS, PRINT_PUMP};
enum SENSORS_STATUS {SENSORS_ENABLED, SENSORS_DISABLED};


// === SENSOR INITIALIZATION ===
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4); // LCD 20x4 I2C
GravityTDS gtds;
CapacitiveSensor capSensor = CapacitiveSensor(CAP_SENS_SEND, CAP_SENS_REC);
SENSORS_STATUS sensorsStatus = SENSORS_ENABLED;
PRINT_MODE printMode = PRINT_SENSORS; // Print pump or sensor data on display


// === GLOBAL VARIABLES ===
// Sensor readings
struct Sensor { 
  long touchValue; // Capacitive sensor
  float temperature; // DHT22
  int humidity; // DHT22
  float ppm; // TDS
  float ec25; // TDS (converted to conductivity)
  int level; // Water level (computed from distance to lid)
} sensor = {0, 0.0, 0, 0.0, 0.0, 0};

// Pump properties
struct Pump {
  PUMP_STATE state; // ON or OFF
  unsigned long countdown_minutes; // Minutes left to switch
  unsigned long countdown_seconds; // Secods left (not total, just to display MINS:SECS)
} pump = {PUMP_OFF, 0, 0};

const unsigned long pumpOnDuration = ON_TIMEOUT * MINUTE; // Pump timer to turn off
const unsigned long pumpOffDuration = OFF_TIMEOUT * MINUTE; // Pump timer to turn on

// === Timers ===
static unsigned long lastSensorRead = 0; // Timer for sensor acquiring and readings update
static unsigned long lastSensorSwitch = 0; // Timer for disabling sensors and display
static unsigned long lastCapRead = 0; // Timer for touch sensor read
static unsigned long lastPumpSwitch = 0; // Timer for pump control


// === HELPER FUNCTIONS ===
#ifdef DEBUG
  void printSerial() {
    Serial.print("Touch: ");
    Serial.println(sensor.touchValue);
    Serial.print("Temperatura: ");
    Serial.print(sensor.temperature);
    Serial.println(" C");
    Serial.print("Humedad: ");
    Serial.print(sensor.humidity);
    Serial.println(" %");
    Serial.print("TDS: ");
    Serial.print(sensor.ppm);
    Serial.println(" ppm");
    Serial.print("EC 25C: ");
    Serial.print(sensor.ec25);
    Serial.println(" uS/cm");
    Serial.print("Nivel de agua: ");
    Serial.print(sensor.level);
    Serial.println(" L");
    Serial.print("Bomba ");
    Serial.println(pump.state == PUMP_ON ? "encendida" : "apagada");
    Serial.print(pump.state == PUMP_ON ? "Apagado de bomba en " : "Encendido de bomba en ");
    Serial.print(pump.countdown_minutes);
    Serial.print(":");
    if (pump.countdown_seconds < 10) Serial.print("0"); 
    Serial.println(pump.countdown_seconds);
    Serial.println("");
    Serial.println("");
  }
#endif

void printLCD() {
  printMode = printMode == PRINT_SENSORS ? PRINT_PUMP : PRINT_SENSORS; // Switch mode
  lcd.clear();
  switch(printMode) {
    case PRINT_PUMP:
      lcd.setCursor(0, 0);
      lcd.print("Bomba ");
      lcd.print(pump.state == PUMP_ON ? "encendida" : "apagada");
      lcd.setCursor(0, 1);
      lcd.print("Tiempo rest.: ");
      lcd.print(pump.countdown_minutes);
      lcd.print(":");
      if (pump.countdown_seconds < 10) lcd.print("0");
      lcd.print(pump.countdown_seconds);
      lcd.setCursor(0,2);
      lcd.print("Nivel de agua: ");
      lcd.print(sensor.level);
      lcd.print(" L");
      return;
    case PRINT_SENSORS:
      lcd.setCursor(0, 0);
      lcd.print("Temperatura: ");
      lcd.print(sensor.temperature);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Hum. ambiente: ");
      lcd.print(sensor.humidity);
      lcd.print(" %");
      lcd.setCursor(0, 2);
      lcd.print("TDS: ");
      lcd.print(sensor.ppm);
      lcd.print(" ppm");
      lcd.setCursor(0, 3);
      lcd.print("EC: ");
      lcd.print(sensor.ec25);
      lcd.print(" uS/cm");
      return;
  }
  return;
}

int readVolume() { 
  // Measure distance with HC-SR04 and convert to volume
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float duration = pulseIn(ECHO_PIN, HIGH, 20000UL);
  float distance = duration*0.01715;
  int vol = (int) (26.847 - 0.7065*distance); // From cm to volume assuming 20L bucket
  return vol < 0 ? 0 : vol;
}

void readSensors() {
  // DHT22 reading (temperature and humidity)
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) sensor.temperature = t;
  if (!isnan(h)) sensor.humidity = (int) h;

  // TDS sensor reading (conductivity)
  gtds.setTemperature(sensor.temperature);
  gtds.update();
  sensor.ppm = gtds.getTdsValue();
  sensor.ec25 = gtds.getEcValue();

  // HC-SR04 sensor reading (distance converted to volume)
  sensor.level = readVolume();
}

// === PROGRAM INITIALIZATION ===
void setup() {

  // Serial communication
#ifdef DEBUG
  Serial.begin(BAUDRATE);
  Serial.println("Inicializando...");
#endif

  // DHT22 sensor initialization
  dht.begin();
  
  // LCD initialization
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Conductivity sensor initialization
  gtds.setPin(TDS_PIN);
  gtds.setAref(TDS_A_REF);
  gtds.setAdcRange(ADC_RANGE);

  // Distance sensor configuration
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Pump control pin configuration (off during configuration, then set on)
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(PUMP_LED_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(PUMP_LED_PIN, LOW);

  // Deactivate autocalibration for the capacitive sensor
  capSensor.set_CS_AutocaL_Millis(0xFFFFFFFF); 
  capSensor.set_CS_Timeout_Millis(30);

#ifdef DEBUG
  Serial.print("Inicializado. Encendiendo bomba... ");
#endif

  // Wait 3 seconds after initialization and then turn on pump
  delay(3000);
  pump.state = PUMP_ON;
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(PUMP_LED_PIN, HIGH);

#ifdef DEBUG
  Serial.print("listo.");
#endif
}


// === MAIN LOOP ===
void loop() {

  // Timers
  const unsigned long now = millis();
  const unsigned long sensorTimeout = now - lastSensorRead; // Read sensors
  const unsigned long capTimeout = now - lastCapRead; // Read touch sensor
  const unsigned long sensorDisableTimeout = now - lastSensorSwitch; // Disable sensors
  const unsigned long pumpTimeout = now - lastPumpSwitch; // Switch pump status

  const bool setPumpOn = (pump.state == PUMP_OFF) && (pumpTimeout >= pumpOffDuration);
  const bool setPumpOff = (pump.state == PUMP_ON) && (pumpTimeout >= pumpOnDuration);
  const bool readSensorsNow = (sensorsStatus == SENSORS_ENABLED) && (sensorTimeout >= SENSOR_READ_PERIOD);
  const bool readTouchSensor = (sensorsStatus == SENSORS_DISABLED) && (capTimeout >= CAP_READ_PERIOD);
  const bool disableSensors = (sensorsStatus == SENSORS_ENABLED) && (sensorDisableTimeout >= SENSORS_TIMEOUT);


  // Pump control - Toggle if timeouts
  if(setPumpOn) {
    pump.state = PUMP_ON;
    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(PUMP_LED_PIN, HIGH);
#ifdef DEBUG
    Serial.println("Bomba encendida.");
#endif
    lastPumpSwitch = now;
  } else if(setPumpOff) {
    pump.state = PUMP_OFF;
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(PUMP_LED_PIN, LOW);
#ifdef DEBUG
    Serial.println("Bomba apagada.");
#endif
    lastPumpSwitch = now;
  }
  // Compute countdowns for printing in display
  const unsigned long remainingMs = pump.state == PUMP_ON ? (pumpOnDuration - pumpTimeout) : (pumpOffDuration - pumpTimeout);
  pump.countdown_minutes = remainingMs / 60000UL;
  pump.countdown_seconds = (remainingMs % 60000UL) / 1000UL;
  
  // If sensor acquisition is enabled, read variables and print
  if (readSensorsNow) {
    readSensors();
    printLCD();
#ifdef DEBUG
    printSerial();
#endif
    lastSensorRead = now;
  }

  if(readTouchSensor) { // Read capacitive sensor
    sensor.touchValue = capSensor.capacitiveSensor(2);
    lastCapRead = now; // Reset capacitive sensor timer
    if (sensor.touchValue > CAP_THRES) { // If touch detected
#ifdef DEBUG
      Serial.println("Reinicio de sensado.");
#endif
      lcd.backlight(); // Wake up LCD
      lastSensorSwitch = now;
      sensorsStatus = SENSORS_ENABLED; // Enable sensor readings
    }
  }

  // If inactive, turn off LCD and sensor reading
  if (disableSensors) {
#ifdef DEBUG
    Serial.println("Sensado desactivado.");
#endif
    lcd.clear();
    lcd.noBacklight();
    lastSensorSwitch = now;
    sensorsStatus = SENSORS_DISABLED;
  }
}
