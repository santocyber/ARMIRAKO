#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const uint8_t NUM_SERVOS = 5;
uint16_t SERVO_MIN[NUM_SERVOS]   = {200,200,200,200,145};
uint16_t SERVO_MAX[NUM_SERVOS]   = {600,600,650,600,630};
int8_t   servoOffset[NUM_SERVOS] = {  0,  0,  0,  0,  0};
int8_t   servoDir[NUM_SERVOS]    = { +1, +1, -1, +1, -1};
const uint8_t CH[NUM_SERVOS]     = {0,1,2,3,4};

int angAtual[NUM_SERVOS] = {0};
int angAlvo[NUM_SERVOS]  = {0};

WebSocketsClient webSocket;
const char* WIFI_SSID = "InternetSA";
const char* WIFI_PASS = "cadebabaca";
const char* WS_HOST   = "websocket.mirako.org";
const uint16_t WS_PORT= 443;
const char* WS_PATH   = "/";



// Guarda quando foi a última vez que enviamos o estado
unsigned long lastStateSend = 0;
const unsigned long STATE_INTERVAL = 30000; // 2 segundos




void sendState() {
  StaticJsonDocument<128> doc;
  doc["base"]     = angAlvo[0];     // envia os alvos atuais :contentReference[oaicite:5]{index=5}
  doc["shoulder"] = angAlvo[1];
  doc["elbow"]    = angAlvo[2];
  doc["wrist"]    = angAlvo[3];
  doc["gripper"]  = angAlvo[4];
  char buf[128];
  serializeJson(doc, buf);          // serializa o JSON :contentReference[oaicite:6]{index=6}
  webSocket.sendTXT(buf);           // envia via WebSocket :contentReference[oaicite:7]{index=7}
  Serial.printf("📤 Estado enviado e movimento iniciado: %s\n", buf);

  // dispara uma etapa de movimento para cada servo
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoStep(i);                   // passo único em direção ao novo alvo :contentReference[oaicite:8]{index=8}
  }
}



// Move cada servo um único passo em direção ao alvo
void servoStep(int idx) {
  if (angAtual[idx] == angAlvo[idx]) return;

  // passo de +1 ou -1
  angAtual[idx] += (angAlvo[idx] > angAtual[idx]) ? +1 : -1;

  // calcule pulso como antes (espelhamento, offset, map)
  int mirrored = (servoDir[idx] == -1) ? (180 - angAtual[idx])
                                       : angAtual[idx];
  int adjusted = constrain(mirrored + servoOffset[idx], 0, 180);
  uint16_t pulse = map(adjusted, 0, 180, SERVO_MIN[idx], SERVO_MAX[idx]);

  pwm.setPWM(CH[idx], 0, pulse);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type!=WStype_TEXT && type!=WStype_BIN) return;
  payload[length] = '\0';

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc,(char*)payload)) return;

  // apenas sobrescreve os alvos
  angAlvo[0] = doc["base"];
  angAlvo[1] = doc["shoulder"];
  angAlvo[2] = doc["elbow"];
  angAlvo[3] = doc["wrist"];
  angAlvo[4] = doc["gripper"];
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);


  WiFi.begin(WIFI_SSID,WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED){ delay(300); Serial.print('.'); }
  Serial.println("\n✅ WiFi conectado");
  webSocket.beginSSL(WS_HOST,WS_PORT,WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);


  
     // força estado inicial = zero
  for(int i = 0; i < NUM_SERVOS; i++) {
    angAlvo[i] = 0;
  }
  // envia imediatamente

  sendState();



}

void loop() {
  webSocket.loop();

  // dê um passo para cada servo
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoStep(i);
  }

  // envie estado a cada STATE_INTERVAL
  unsigned long now = millis();
  if (now - lastStateSend >= STATE_INTERVAL) {
    lastStateSend = now;
    sendState();
  }

  delay(20);  // pequeno delay para alívio de CPU e suavização
}
