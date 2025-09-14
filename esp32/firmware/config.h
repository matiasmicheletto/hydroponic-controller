#define DEBUG 1

#ifdef DEBUG
  #define BAUDRATE 115200
#endif
#define DHT_PIN 14      
#define DHT_TYPE DHT22

#define TRIG_PIN 9
#define ECHO_PIN 10

#define TDS_INPUT_PIN 33
#define TDS_A_REF 3.3
#define ADC_RANGE 4096
#define ANALOG_READ_RESOLUTION 12

#define WSS_PORT 82
#define WIFI_CONNECT_ATTEMPTS 50 // Max number of attempts to connect to WiFi

#define SENSOR_READ_PERIOD 5000UL // 5 seconds