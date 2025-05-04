#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>  // https://arduinojson.org/

// ————— Configuração de Rede —————
const char* ssid     = "InternetSA";
const char* password = "cadebabaca";

// ————— Configuração do WebSocket —————
const char* ws_host = "websocket.mirako.org";
const uint16_t ws_port = 6001;
const char* ws_path = "/";

// ————— Parâmetros de Calibração e Filtragem —————
struct Calib { int rawMin, rawMax; float smooth; };
Calib calibBase     = {  500, 3200, 0.1f };  // ajuste se precisar
Calib calibShoulder = {  500, 2900, 0.1f };
Calib calibElbow    = {  500, 2800, 0.1f };
Calib calibWrist    = {  500, 2800, 0.1f };

const int   TOLERANCE = 6;    // só envia quando muda > 5°
const int   DELAY_MS  = 150;   // intervalo entre leituras

// ————— Pinos —————
const int potBasePin     = 32;
const int potShoulderPin = 33;
const int potElbowPin    = 34;
const int potWristPin    = 35;
const int buttonPin      = 25;

// ————— Estado e variáveis de filtragem —————
WebSocketsClient webSocket;
int  lastBase     = -999;
int  lastShoulder = -999;
int  lastElbow    = -999;
int  lastWrist    = -999;
bool lastGrip     = false;
float filtBase     = 0, filtShoulder = 0, filtElbow = 0, filtWrist = 0;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.println("✅ WS conectado");
      break;
    case WStype_DISCONNECTED:
      Serial.println("❌ WS desconectado");
      break;
    case WStype_TEXT: {
      // 1) tenta calibração por JSON
      StaticJsonDocument<200> doc;
      if (deserializeJson(doc, payload) == DeserializationError::Ok) {
        bool fezCalib = false;
        if (doc.containsKey("calibShoulderMin")) {
          calibShoulder.rawMin = doc["calibShoulderMin"];
          fezCalib = true;
        }
        if (doc.containsKey("calibShoulderMax")) {
          calibShoulder.rawMax = doc["calibShoulderMax"];
          fezCalib = true;
        }
        if (doc.containsKey("calibElbowMin")) {
          calibElbow.rawMin = doc["calibElbowMin"];
          fezCalib = true;
        }
        if (doc.containsKey("calibElbowMax")) {
          calibElbow.rawMax = doc["calibElbowMax"];
          fezCalib = true;
        }
        if (doc.containsKey("calibWristMin")) {
          calibWrist.rawMin = doc["calibWristMin"];
          fezCalib = true;
        }
        if (doc.containsKey("calibWristMax")) {
          calibWrist.rawMax = doc["calibWristMax"];
          fezCalib = true;
        }
        if (fezCalib) {
          Serial.println("⚙️ Calibracao atualizada:");
          Serial.printf("   Shoulder: min=%d max=%d\n", calibShoulder.rawMin, calibShoulder.rawMax);
          Serial.printf("   Elbow:    min=%d max=%d\n", calibElbow.rawMin,    calibElbow.rawMax);
          Serial.printf("   Wrist:    min=%d max=%d\n", calibWrist.rawMin,    calibWrist.rawMax);
          return; // não trata como comando normal
        }
      }
      // 2) se não era calib, printa a mensagem normal
      Serial.printf("⏪ Servidor: %s\n", payload);
      break;
    }
    case WStype_ERROR:
      Serial.println("⚠️ Erro no WS");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);

  // Conecta Wi-Fi
  Serial.printf("Conectando em %s …", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(300);
  }
  Serial.printf("\nWi-Fi OK! IP: %s\n", WiFi.localIP().toString().c_str());

  // Inicia WebSocket
  webSocket.begin(ws_host, ws_port, ws_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  // 1) Lê brutos
  int rawB = analogRead(potBasePin);
  int rawS = analogRead(potShoulderPin);
  int rawE = analogRead(potElbowPin);
  int rawW = analogRead(potWristPin);
  bool grip = (digitalRead(buttonPin) == LOW);

  // 2) Atualiza WebSocket (pode receber calibração)
  webSocket.loop();

  // 3) Mapeia e constrange em 0–180°
  int angleB = constrain(map(rawB, calibBase.rawMin,     calibBase.rawMax,     0, 340), 0, 340);
  int angleS = 180 - constrain(map(rawS, calibShoulder.rawMin, calibShoulder.rawMax, 0, 180), 0, 180);
  int angleE = constrain(map(rawE, calibElbow.rawMin,    calibElbow.rawMax,    0, 180), 0, 180);
  int angleW = 180 - constrain(map(rawW, calibWrist.rawMin,    calibWrist.rawMax,    0, 180), 0, 180);

  // 4) Suaviza (média móvel)
  filtBase     += calibBase.smooth     * (angleB - filtBase);
  filtShoulder += calibShoulder.smooth * (angleS - filtShoulder);
  filtElbow    += calibElbow.smooth    * (angleE - filtElbow);
  filtWrist    += calibWrist.smooth    * (angleW - filtWrist);

  int mBase     = round(filtBase);
  int mShoulder = round(filtShoulder);
  int mElbow    = round(filtElbow);
  int mWrist    = round(filtWrist);

  // 5) Só envia se mudou além da tolerância
  bool changed = false;
  if (abs(mBase     - lastBase)     > TOLERANCE) { lastBase     = mBase;     changed = true; }
  if (abs(mShoulder - lastShoulder) > TOLERANCE) { lastShoulder = mShoulder; changed = true; }
  if (abs(mElbow    - lastElbow)    > TOLERANCE) { lastElbow    = mElbow;    changed = true; }
  if (abs(mWrist    - lastWrist)    > TOLERANCE) { lastWrist    = mWrist;    changed = true; }
  if (grip != lastGrip)                         { lastGrip     = grip;      changed = true; }

  if (changed) {
    // 6) Monta e envia JSON
    String json = "{";
    json += "\"base\":"     + String(lastBase)     + ",";
    json += "\"shoulder\":" + String(lastShoulder) + ",";
    json += "\"elbow\":"    + String(lastElbow)    + ",";
    json += "\"wrist\":"    + String(lastWrist)    + ",";
    json += "\"gripper\":"  + String(lastGrip ? 1 : 0);
    json += "}";
    webSocket.sendTXT(json);
    Serial.println(json);
  }

  delay(DELAY_MS);
}
