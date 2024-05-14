#include <WiFiClient.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pwm.pio.h"
#include "sensor2motor.h"
#include "sensor.pio.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef PICO_DEFAULT_LED_PIN
#error pio/pwm example requires a board with a regular LED
#endif

#define MOTOR_PIN_0 12
#define MOTOR_PIN_1 13
#define MOTOR_PIN_2 14
#define MOTOR_PIN_3 15

#define SENSOR_PIN_0 26
#define SENSOR_PIN_1 22
#define SENSOR_PIN_2 21
#define SENSOR_PIN_3 20
#define SENSOR_PIN_4 19
#define SENSOR_PIN_5 18
#define SENSOR_PIN_6 17
#define SENSOR_PIN_7 16

const char* ssid = "PicoW";
const char* password = "12345678";

const IPAddress ip(192, 168, 40, 1);
const IPAddress subnet(255, 255, 255, 0);

WebSocketsServer webSocket = WebSocketsServer(8081);

/////////////////////////////////////////////////////////////////////
// Function prototypes
/////////////////////////////////////////////////////////////////////

void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_put_blocking(pio, sm, period);
  pio_sm_exec(pio, sm, pio_encode_pull(false, false));
  pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
  pio_sm_set_enabled(pio, sm, true);
}

void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
  pio_sm_put_blocking(pio, sm, level);
}

unsigned int readLatestDataFromFifo(PIO pio, uint sm) {
  unsigned int latestData = 0;
  while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
    latestData = pio_sm_get(pio, sm);
    Serial.println("wait");
  }
  Serial.println(latestData);
  return latestData;
}

PIO pio_0 = pio0;
PIO pio_1 = pio1;

uint pio_0_offset;
uint pio_1_offset;
int pio_0_sm_0 = 0;
int pio_0_sm_1 = 1;
int pio_0_sm_2 = 2;
int pio_0_sm_3 = 3;
int pio_1_sm_0 = 1;

static unsigned int leftMotorAccelForward   = 0;
static unsigned int leftMotorAccelBackward  = 0;
static unsigned int rightMotorAccelForward  = 0;
static unsigned int rightMotorAccelBackward = 0;

float speed = 0.5;
float Kp = 0.045;
float Ki = 0;
float Kd = 0;
int   sensorWeight[8] = {0, 0, 0, 0, 0 ,0, 0, 0};

char sensor, sensor_prev;

bool active = false;

int dma_chan_0;
int dma_chan_1;

void setup() {

  // GPIOピンを初期化
  gpio_init(SENSOR_PIN_0);
  gpio_init(SENSOR_PIN_1);
  gpio_init(SENSOR_PIN_2);
  gpio_init(SENSOR_PIN_3);
  gpio_init(SENSOR_PIN_4);
  gpio_init(SENSOR_PIN_5);
  gpio_init(SENSOR_PIN_6);
  gpio_init(SENSOR_PIN_7);
  gpio_init(MOTOR_PIN_0);
  gpio_init(MOTOR_PIN_1);
  gpio_init(MOTOR_PIN_2);
  gpio_init(MOTOR_PIN_3);

  gpio_set_dir(SENSOR_PIN_0, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_1, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_2, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_3, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_4, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_5, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_6, GPIO_IN);
  gpio_set_dir(SENSOR_PIN_7, GPIO_IN);

  gpio_set_dir(MOTOR_PIN_0, GPIO_OUT);
  gpio_set_dir(MOTOR_PIN_1, GPIO_OUT);
  gpio_set_dir(MOTOR_PIN_2, GPIO_OUT);
  gpio_set_dir(MOTOR_PIN_3, GPIO_OUT);

  // PWM
  pio_0_offset = pio_add_program(pio_0, &pwm_program);
  pwm_program_init(pio_0, pio_0_sm_0, pio_0_offset, MOTOR_PIN_0);
  pwm_program_init(pio_0, pio_0_sm_1, pio_0_offset, MOTOR_PIN_1);
  pwm_program_init(pio_0, pio_0_sm_2, pio_0_offset, MOTOR_PIN_2);
  pwm_program_init(pio_0, pio_0_sm_3, pio_0_offset, MOTOR_PIN_3);
  pio_pwm_set_period(pio_0, pio_0_sm_0, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_1, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_2, (1u << 16) - 1);
  pio_pwm_set_period(pio_0, pio_0_sm_3, (1u << 16) - 1);
  Serial.printf("Loaded program at %d\n", pio_0_offset);

  // Sensor
  // pio_1_offset    = pio_add_program(pio_1, &read_sensors_program);
  // pio_sm_config c = pio_get_default_sm_config();
  // sm_config_set_in_pins(&c, SENSOR_PIN_0);
  // pio_sm_init(pio_1, pio_1_sm_0, pio_1_offset, &c);
  // pio_sm_set_enabled(pio_1, pio_1_sm_0, true);
  // Serial.printf("Loaded program at %d\n", pio_1_offset);
  
  // DMA
  // dma_chan_0 = dma_claim_unused_channel(true);
  // dma_chan_1 = dma_claim_unused_channel(true);
  // dma_channel_config dma_c_0 = dma_channel_get_default_config(dma_chan_0);
  // dma_channel_config dma_c_1 = dma_channel_get_default_config(dma_chan_1);
  // channel_config_set_transfer_data_size(&dma_c_0, DMA_SIZE_32);
  // channel_config_set_transfer_data_size(&dma_c_1, DMA_SIZE_32);
  // channel_config_set_read_increment(&dma_c_0, false);
  // channel_config_set_read_increment(&dma_c_1, false);
  // channel_config_set_write_increment(&dma_c_0, false);
  // channel_config_set_write_increment(&dma_c_1, false);
  // channel_config_set_dreq(&dma_c_0, pio_get_dreq(pio_1, pio_1_sm_0, false));
  // channel_config_set_dreq(&dma_c_1, pio_get_dreq(pio_1, pio_1_sm_0, false));
  // channel_config_set_chain_to(&dma_c_0, dma_chan_1);
  // channel_config_set_chain_to(&dma_c_1, dma_chan_0);
  // dma_channel_configure(dma_chan_0, &dma_c_0, &sensorReg, &pio_1->rxf[pio_1_sm_0], 1, true);
  // dma_channel_configure(dma_chan_1, &dma_c_1, &sensorReg, &pio_1->rxf[pio_1_sm_0], 1, true);

  sleep_ms(1000);

  // Serial
  Serial.begin(115200);

  // websocket
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);    

  Serial.println("WebSocket server started.");
}
unsigned long lastTime = 0;

void loop() {
  if (active){
    
    unsigned long currentTime = millis();
    float deltaT = (currentTime - lastTime) / 1000.0;

    sensor = gpio_get(SENSOR_PIN_0) << 7 |  gpio_get(SENSOR_PIN_1) << 6 | gpio_get(SENSOR_PIN_2) << 5 | gpio_get(SENSOR_PIN_3) << 4 | gpio_get(SENSOR_PIN_4) << 3 | gpio_get(SENSOR_PIN_5) << 2 | gpio_get(SENSOR_PIN_6) << 1 | gpio_get(SENSOR_PIN_7) << 0;

    Serial.println(sensor, HEX);


    if (sensor == 0){
      sensor = sensor_prev;
    }

    uint32_t accel = control_motors(sensor, &sensorWeight, speed, Kp, Ki, Kd, deltaT);

    // Serial.println(accel, HEX);
    
    leftMotorAccelForward   = (accel >> 24) & 0xFF; 
    leftMotorAccelBackward  = (accel >> 16) & 0xFF;
    rightMotorAccelForward  = (accel >> 8)  & 0xFF;
    rightMotorAccelBackward = (accel >> 0)  & 0xFF;

    pio_pwm_set_level(pio_0, pio_0_sm_0, leftMotorAccelForward   * leftMotorAccelForward);
    pio_pwm_set_level(pio_0, pio_0_sm_1, leftMotorAccelBackward  * leftMotorAccelBackward);
    pio_pwm_set_level(pio_0, pio_0_sm_2, rightMotorAccelForward  * rightMotorAccelForward);
    pio_pwm_set_level(pio_0, pio_0_sm_3, rightMotorAccelBackward * rightMotorAccelBackward);

    sensor_prev = sensor;
    lastTime = currentTime;
  }
}

void loop1() {
  webSocket.loop();
}


/////////////////////////////////////////////////////////////////////
// decodeAccel
// @param accel_string: "accel 0x00 0x00 0x00 0x00"
/////////////////////////////////////////////////////////////////////

void decodeAccel(String accel_string) {
  String accel_hex_0 = accel_string.substring(6, 8);
  String accel_hex_1 = accel_string.substring(8, 10);
  String accel_hex_2 = accel_string.substring(10, 12);
  String accel_hex_3 = accel_string.substring(12, 14);
  leftMotorAccelForward   = strtoul(accel_hex_0.c_str(), NULL, 16);
  rightMotorAccelForward  = strtoul(accel_hex_1.c_str(), NULL, 16);
  leftMotorAccelBackward  = strtoul(accel_hex_2.c_str(), NULL, 16);
  rightMotorAccelBackward = strtoul(accel_hex_3.c_str(), NULL, 16);
}

void extractAndConvertFloatSpeed(String text) {
    int startIndex = text.indexOf("Speed:") + 6;  // "PID:kp:" の次の文字から開始
    int endIndex = text.indexOf(';', startIndex);  // ";" が出現する位置を検索

    if (startIndex > 0 && endIndex > startIndex) {
        String numberStr = text.substring(startIndex, endIndex);
        speed = numberStr.toFloat();
    } else {
        Serial.println("Error: Invalid format");
    }
}

void extractAndConvertFloatKp(String text) {
    int startIndex = text.indexOf("PID:kp:") + 7;  // "PID:kp:" の次の文字から開始
    int endIndex = text.indexOf(';', startIndex);  // ";" が出現する位置を検索

    if (startIndex > 0 && endIndex > startIndex) {
        String numberStr = text.substring(startIndex, endIndex);
        Kp = numberStr.toFloat();
    } else {
        Serial.println("Error: Invalid format");
    }
}

void extractAndConvertFloatKi(String text) {
    int startIndex = text.indexOf("PID:ki:") + 7;  // "PID:kp:" の次の文字から開始
    int endIndex = text.indexOf(';', startIndex);  // ";" が出現する位置を検索

    if (startIndex > 0 && endIndex > startIndex) {
        String numberStr = text.substring(startIndex, endIndex);
        Ki = numberStr.toFloat();
    } else {
        Serial.println("Error: Invalid format");
    }
}
void extractAndConvertFloatKd(String text) {
    int startIndex = text.indexOf("PID:kd:") + 7;  // "PID:kp:" の次の文字から開始
    int endIndex = text.indexOf(';', startIndex);  // ";" が出現する位置を検索

    if (startIndex > 0 && endIndex > startIndex) {
        String numberStr = text.substring(startIndex, endIndex);
        Kd = numberStr.toFloat();
    } else {
        Serial.println("Error: Invalid format");
    }
}

void extractAndConvertSensor(String text) {
    int startIndex   = text.indexOf("Sensor:sensor") + 13;  // "PID:kp:" の次の文字から開始
    String numberStr = text.substring(startIndex, startIndex + 1);
    int sensorIndex  = numberStr.toInt();
    startIndex       = startIndex + 2;
    int endIndex     = text.indexOf(';', startIndex);         // ";" が出現する位置を検索

    if (startIndex > 0 && endIndex > startIndex) {
        numberStr = text.substring(startIndex, endIndex);
        sensorWeight[sensorIndex] = numberStr.toInt();
    } else {
        Serial.println("Error: Invalid format");
    }
}

/////////////////////////////////////////////////////////////////////
// webSocketEvent
// @param num: WebSocket number
// @param type: WebSocket event type
// @param payload: WebSocket payload
// @param length: WebSocket payload length
/////////////////////////////////////////////////////////////////////

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      webSocket.sendTXT(num, "Welcome to the WebSocket server!");
      break;
    
    }
    case WStype_TEXT:
      String text = String((char *)payload).substring(0, length);
      Serial.printf("[%u] Text: %s\n", num, text.c_str());
      if(text.startsWith("start")) {
        active = true;
        init_pid();
      }
      if(text.startsWith("stop")) {
        active = false;
        leftMotorAccelForward   = 0;
        leftMotorAccelBackward  = 0;
        rightMotorAccelForward  = 0;
        rightMotorAccelBackward = 0;
        pio_pwm_set_level(pio_0, pio_0_sm_0, leftMotorAccelForward   * leftMotorAccelForward);
        pio_pwm_set_level(pio_0, pio_0_sm_1, leftMotorAccelBackward  * leftMotorAccelBackward);
        pio_pwm_set_level(pio_0, pio_0_sm_2, rightMotorAccelForward  * rightMotorAccelForward);
        pio_pwm_set_level(pio_0, pio_0_sm_3, rightMotorAccelBackward * rightMotorAccelBackward);
        init_pid();
      }
      if(text.startsWith("accel ")) {
        decodeAccel(text);
      }
      if(text.startsWith("PID:kp:")){
        extractAndConvertFloatKp(text);
      }
      if(text.startsWith("PID:ki:")){
        extractAndConvertFloatKi(text);
      }
      if(text.startsWith("PID:kd:")){
        extractAndConvertFloatKd(text);
      }
      if(text.startsWith("Speed:")){
        extractAndConvertFloatSpeed(text);
      }
      if(text.startsWith("Sensor")){
        extractAndConvertSensor(text);
      }
      break;
  }
}
