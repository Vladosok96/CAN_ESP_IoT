
#include "esp32_can.h"            // https://github.com/collin80/esp32_can AND https://github.com/collin80/can_common

#define CANPID_RPM          0x0C
#define CANPID_SPEED        0x0D
#define CANPID_TEMPERATURE  0x05
#define CAN_REQST_ID        0x7DF 
#define CAN_REPLY_ID        0x7E8

// GPRS credentials (leave empty, if not needed)
const char apn[]      = "internet.beeline.ru"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "beeline"; // GPRS User
const char gprsPass[] = "beeline"; // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 

// MQTT details
const char* broker = "dev.rightech.io";                    // Public IP address or domain name
const char* mqttUsername = "cmaxobject";  // MQTT username
const char* mqttPassword = "vlad2002";  // MQTT password

const char* topicOutput1 = "esp/output1";
const char* topicOutput2 = "esp/output2";
const char* topicTemperature = "enginetemperature";
const char* topicRPM = "rpm";
const char* topicSpeed = "speed";

// Keep this API Key value to be compatible with the PHP code provided in the project page. 
// If you change the apiKeyValue value, the PHP file /post-data.php also needs to have the same key 
String apiKeyValue = "tPmAT5Ab3j7F9";

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// BME280 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19

#define OUTPUT_1             2
#define OUTPUT_2             15

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);
PubSubClient mqtt(client);

uint32_t lastReconnectAttempt = 0;

#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

uint16_t rpm;
uint16_t speed;
uint16_t temperature;
int message_state = 0;
long lastMsg = 0;

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp/output1, you check if the message is either "true" or "false". 
  // Changes the output state according to the message
  if (String(topic) == "esp/output1") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      Serial.println("true");
      digitalWrite(OUTPUT_1, HIGH);
    }
    else if(messageTemp == "false"){
      Serial.println("false");
      digitalWrite(OUTPUT_1, LOW);
    }
  }
  else if (String(topic) == "esp/output2") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      Serial.println("true");
      digitalWrite(OUTPUT_2, HIGH);
    }
    else if(messageTemp == "false"){
      Serial.println("false");
      digitalWrite(OUTPUT_2, LOW);
    }
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  boolean status = mqtt.connect("mqtt-vmixoks-mrthet");

  // Or, if you want to authenticate MQTT:
  //boolean status = mqtt.connect("mqtt-vmixoks-p9qw32", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(topicOutput1);
  mqtt.subscribe(topicOutput2);

  return mqtt.connected();
}

void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start CAN0 (built-in can module)
  CAN0.begin();
  CAN0.watchFor(CAN_REPLY_ID);
  CAN0.setCallback(0, callback);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);
  
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }

  // Configure the wake up source as timer wake up  
  // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }
  
  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }  

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);  

}

void request_RPM(void) {  
  CAN_FRAME outgoing;
  outgoing.id = CAN_REQST_ID;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;  
  outgoing.data.uint8[1] = 0x01;  
  outgoing.data.uint8[2] = CANPID_RPM; 
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;  
  outgoing.data.uint8[5] = 0x00;  
  outgoing.data.uint8[6] = 0x00;  
  outgoing.data.uint8[7] = 0x00;  
  CAN0.sendFrame(outgoing);
}

void request_speed(void) {  
  CAN_FRAME outgoing;
  outgoing.id = CAN_REQST_ID;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;  
  outgoing.data.uint8[1] = 0x01;  
  outgoing.data.uint8[2] = CANPID_SPEED; 
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;  
  outgoing.data.uint8[5] = 0x00;  
  outgoing.data.uint8[6] = 0x00;  
  outgoing.data.uint8[7] = 0x00;  
  CAN0.sendFrame(outgoing);
}

void request_temperature(void) {  
  CAN_FRAME outgoing;
  outgoing.id = CAN_REQST_ID;
  outgoing.length = 8;
  outgoing.extended = 0;
  outgoing.rtr = 0;
  outgoing.data.uint8[0] = 0x02;  
  outgoing.data.uint8[1] = 0x01;  
  outgoing.data.uint8[2] = CANPID_TEMPERATURE; 
  outgoing.data.uint8[3] = 0x00;
  outgoing.data.uint8[4] = 0x00;  
  outgoing.data.uint8[5] = 0x00;  
  outgoing.data.uint8[6] = 0x00;  
  outgoing.data.uint8[7] = 0x00;  
  CAN0.sendFrame(outgoing);
}

void callback(CAN_FRAME *from_car) {
  if (from_car->data.uint8[2]==CANPID_RPM) {
    uint8_t rpmOBDH = from_car->data.uint8[3];
    uint8_t rpmOBDL = from_car->data.uint8[4];
    rpm = (uint16_t) ((256*rpmOBDH) + rpmOBDL)/(float)4;
  }
  else if (from_car->data.uint8[2]==CANPID_SPEED) {
    speed = from_car->data.uint8[3];
  }
  else if (from_car->data.uint8[2]==CANPID_TEMPERATURE) {
    temperature = from_car->data.uint8[3] - 40;
  }
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
   
  long now = millis();
  if (now - lastMsg > 15000 && message_state == 0) {
    request_temperature(); 
    
    // Convert the value to a char array
    char tmpString[8];
    dtostrf(temperature, 1, 2, tmpString);
    Serial.print("Temperature: ");
    Serial.println(tmpString);
    mqtt.publish(topicTemperature, tmpString);

    message_state = 1;
  }
  else if (now - lastMsg > 16000 && message_state == 1) {
    request_speed();

    // Convert the value to a char array
    char tmpString[8];
    dtostrf(temperature, 1, 2, tmpString);
    Serial.print("speed: ");
    Serial.println(tmpString);
    mqtt.publish(topicSpeed, tmpString);
    message_state = 2;
  }
  else if (now - lastMsg > 17000 && message_state == 2) {
    lastMsg = now;
    
    request_RPM();

    // Convert the value to a char array
    char tmpString[8];
    dtostrf(temperature, 1, 2, tmpString);
    Serial.print("rpm: ");
    Serial.println(tmpString);
    mqtt.publish(topicRPM, tmpString);
    message_state = 0;
  }

  mqtt.loop();
}
