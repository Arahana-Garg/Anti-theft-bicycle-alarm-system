/* ESP32 Anti-theft Bike
   - RFID (MFRC522) toggles lock/unlock manually
   - Pushbutton sequence (GPIO 15,0,13,12) can lock/unlock when correct sequence entered
   - If correct sequence NOT entered within 16 presses -> buzzer sounds 10s and an alert is posted to Firebase
   - Sensor-triggered alarm: if sensors trip and correct RFID not presented within 10s -> buzzer 10s + alert
   - LED on GPIO2: HIGH = unlocked, LOW = locked
   - Posts /bicycle/live and /bicycle/alerts to Firebase
   - Uses WiFiClientSecure::setInsecure() for testing TLS (remove for production)
   - Edit WIFI_SSID, WIFI_PASS, FIREBASE_BASE, FIREBASE_AUTH before uploading
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <MFRC522.h>
#include "esp_sleep.h"

// ---------- CONFIG (edit) ----------
const char* WIFI_SSID   = "IITRPR";
const char* WIFI_PASS   = "V#6qF?pyM!bQ$%NX";
const char* FIREBASE_BASE  = "https://micycle-d3b07-default-rtdb.firebaseio.com/bicycle/live.json"; // replace
const char* FIREBASE_AUTH  = ""; // leave "" if not required

// SIM808 UART (GPS)
const int SIM_RX_PIN = 16;
const int SIM_TX_PIN = 17;
const unsigned long SIM_BAUD = 9600;

// MPU6050
MPU6050 mpu;
int16_t ax, ay, az;
const float pitchOffset = 7.68;

// SW-420 vibration sensor
const gpio_num_t vibrationDigitalPin = GPIO_NUM_33; // ext0 wake capable
const int vibrationAnalogPin = 34;                  // ADC1_CH6

// RFID (MFRC522)
const int SS_PIN = 5;
const int RST_PIN = 27;
// Authorized UID (update if your reader prints different byte order)
const byte AUTH_UID[] = {0x5B, 0xC9, 0x55, 0xD3};
const byte AUTH_UID_LEN = 4;
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Buzzer and LED
const int BUZZER_PIN = 25;
const int LED_PIN = 2; // per user snippet: LED indicator GPIO2 (HIGH = unlocked, LOW = locked)

// Pushbutton sequence pins and settings (from user snippet)
const int btnPins[4] = {15, 0, 13, 12};
const int sequenceLen = 4;
const int unlockSequence[sequenceLen] = {0, 2, 1, 3}; // example: 15 -> 13 -> 0 -> 12
int seqPos = 0;
int totalPresses = 0;
const int maxPresses = 16;
const unsigned long debounceDelay = 50;
unsigned long lastDebounceTime[4] = {0,0,0,0};
bool lastReading[4] = {HIGH, HIGH, HIGH, HIGH};
bool pressedState[4] = {false, false, false, false};

// thresholds and timings
const bool WAKE_ON_HIGH = true;
const unsigned long MONITOR_MS = 10000;            // monitoring window to decide sleep (10s)
const unsigned long SAMPLE_DELAY_MS = 200;
const float TILT_THRESHOLD_DEG = 25.0;
const float ACCMAG_HIGH = 1.5;
const float ACCMAG_LOW  = 0.5;
const int ANALOG_VIBRATION_THRESHOLD = 2000;
const unsigned long RFID_WAIT_MS = 10000;          // 10s to present tag for sensor-triggered alarm
const unsigned long BUZZER_DURATION_MS = 10000;    // 10s alarm
const unsigned long POST_COOLDOWN_MS = 3000;
const unsigned long COMMAND_POLL_MS = 5000;

HardwareSerial simSerial(2);
unsigned long lastPost = 0;
unsigned long lastCommandPoll = 0;
bool locked = true; // initial state locked

// last known GPS fix persistence (numeric strings)
String lastLat = "0.0";
String lastLon = "0.0";

// Forward declarations
bool getGPS(String &lat, String &lon, String &utc);
void flushSim();
String readSimLine(uint32_t timeout = 1200);
bool ensureWiFiConnected();
String buildFirebaseUrlLive();
String buildFirebaseUrlCommand();
String buildFirebaseUrlAlerts();
bool firebasePutLive(const String &json);
bool firebasePostAlert(const String &json);
String firebaseGetCommand();
bool waitForAuthorizedTagOnce(unsigned long timeoutMs, bool &authorized);
void setLockState(bool lockState, const String &lat, const String &lon);
void monitorAndDecideSleep();
void sendAlertToFirebaseMinimal();
void handleButtonPress(int btnIndex);
void resetSequence();
void flashLed(int times, int msDelay);

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Anti-theft Bike: sequence + RFID + Firebase alerts ===");

  // SIM UART
  simSerial.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(200);
  simSerial.println("AT+CGNSPWR=1");
  delay(500);
  flushSim();

  // MPU6050
  Wire.begin(21, 22);
  Wire.setClock(100000);
  mpu.initialize();
  delay(100);
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed - halting");
    while (true) { delay(1000); }
  }
  Serial.println("MPU6050 initialized");

  // SW-420
  if (WAKE_ON_HIGH) pinMode(vibrationDigitalPin, INPUT_PULLDOWN);
  else pinMode(vibrationDigitalPin, INPUT_PULLUP);
  analogReadResolution(12);

  // Buzzer & LED
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // locked -> LED LOW

  // Buttons: INPUT_PULLUP
  for (int i = 0; i < 4; i++) {
    pinMode(btnPins[i], INPUT_PULLUP);
    lastReading[i] = digitalRead(btnPins[i]);
    pressedState[i] = false;
    lastDebounceTime[i] = 0;
  }

  // RFID
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID initialized");

  // WiFi
  ensureWiFiConnected();

  // Post initial lock state (locked)
  String lat="", lon="", utc="";
  getGPS(lat, lon, utc);
  setLockState(true, lat, lon);

  // Start monitoring and decide whether to sleep
  monitorAndDecideSleep();

  // Before sleeping, ensure system is locked (requirement)
  String latNow, lonNow, utcNow;
  getGPS(latNow, lonNow, utcNow);
  setLockState(true, latNow, lonNow); // force lock before sleep

  // Configure ext0 wake and deep sleep
  esp_sleep_enable_ext0_wakeup(vibrationDigitalPin, WAKE_ON_HIGH ? 1 : 0);
  Serial.println("Entering deep sleep.");
  delay(100);
  esp_deep_sleep_start();
}

void loop() {
  // not used
}

// ---------- core monitoring + sleep decision ----------
void monitorAndDecideSleep() {
  while (true) {
    Serial.println("Starting monitor window...");
    unsigned long windowStart = millis();
    bool anyActivity = false;

    while (millis() - windowStart < MONITOR_MS) {
      // Poll Firebase command periodically (remote lock/unlock)
      if (millis() - lastCommandPoll >= COMMAND_POLL_MS) {
        lastCommandPoll = millis();
        if (ensureWiFiConnected()) {
          String cmd = firebaseGetCommand();
          if (cmd.length() > 0) {
            // normalize command
            cmd.trim();
            if (cmd.startsWith("\"") && cmd.endsWith("\"") && cmd.length() >= 2) {
              cmd = cmd.substring(1, cmd.length() - 1);
            }
            while (cmd.length() && isspace(cmd.charAt(0))) cmd = cmd.substring(1);
            while (cmd.length() && isspace(cmd.charAt(cmd.length()-1))) cmd.remove(cmd.length()-1);
            for (unsigned int i = 0; i < cmd.length(); ++i) cmd.setCharAt(i, tolower(cmd.charAt(i)));

            Serial.print("Normalized command: "); Serial.println(cmd);

            if (cmd.equals("lock")) {
              String lat, lon, utc; getGPS(lat, lon, utc);
              setLockState(true, lat, lon); // LED LOW
            } else if (cmd.equals("unlock")) {
              // For this version: unlocking via firebase should reflect immediately in LED
              String lat, lon, utc; getGPS(lat, lon, utc);
              setLockState(false, lat, lon); // LED HIGH
            } else {
              Serial.println("Unknown command value from Firebase.");
            }
            // Optionally clear command node here if desired
          }
        }
      }

      // MPU accel
      mpu.getAcceleration(&ax, &ay, &az);
      float axg = ax / 16384.0;
      float ayg = ay / 16384.0;
      float azg = az / 16384.0;
      float accMag = sqrt(axg*axg + ayg*ayg + azg*azg);

      float pitch = atan2(axg, sqrt(ayg*ayg + azg*azg)) * 180.0 / PI + pitchOffset;
      float tilt = pitch;

      int vibDigital = digitalRead(vibrationDigitalPin);
      int vibAnalog = analogRead(vibrationAnalogPin);

      Serial.print("tilt: "); Serial.print(tilt,2);
      Serial.print(" | acc: "); Serial.print(accMag,3);
      Serial.print(" | vibA: "); Serial.print(vibAnalog);
      Serial.print(" | vibD: "); Serial.println(vibDigital);

      // Manual non-blocking RFID toggle at any time
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        Serial.print("Manual tag UID:");
        for (byte i = 0; i < mfrc522.uid.size; i++) {
          Serial.print(" ");
          if (mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
          Serial.print(mfrc522.uid.uidByte[i], HEX);
        }
        Serial.println();
        bool match = (mfrc522.uid.size == AUTH_UID_LEN);
        if (match) {
          for (byte i = 0; i < AUTH_UID_LEN; i++) {
            if (mfrc522.uid.uidByte[i] != AUTH_UID[i]) { match = false; break; }
          }
        }
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        if (match) {
          Serial.println("Authorized UID matched. Toggling lock state.");
          String latNow, lonNow, utcNow; getGPS(latNow, lonNow, utcNow);
          setLockState(!locked, latNow, lonNow);
        } else {
          Serial.println("UID not authorized.");
        }
      }

      // Read buttons with debounce and handle presses (sequence logic)
      for (int i = 0; i < 4; i++) {
        bool reading = digitalRead(btnPins[i]);
        if (reading != lastReading[i]) {
          lastDebounceTime[i] = millis();
          lastReading[i] = reading;
        }
        if ((millis() - lastDebounceTime[i]) > debounceDelay) {
          if (reading == LOW && !pressedState[i]) {
            pressedState[i] = true;
            handleButtonPress(i);
          } else if (reading == HIGH) {
            pressedState[i] = false;
          }
        }
      }

      // Detect activity
      bool activity = false;
      if (abs(tilt) > TILT_THRESHOLD_DEG) activity = true;
      if (accMag > ACCMAG_HIGH || accMag < ACCMAG_LOW) activity = true;
      if (vibAnalog > ANALOG_VIBRATION_THRESHOLD) activity = true;
      if (vibDigital == (WAKE_ON_HIGH ? HIGH : LOW)) activity = true;
      // button presses count as activity (totalPresses > 0)
      if (totalPresses > 0) activity = true;

      if (activity) {
        anyActivity = true;
        Serial.println("Activity detected during monitor window.");

        // Refresh GPS (updates lastLat/lastLon if fix)
        String latNow="", lonNow="", utcNow="";
        getGPS(latNow, lonNow, utcNow);

        // Post an immediate snapshot including locked state
        String payload = "{";
        payload += "\"acceleration\":" + String(accMag, 3) + ",";
        payload += "\"lat\":" + lastLat + ",";
        payload += "\"lon\":" + lastLon + ",";
        payload += "\"tilt\":" + String(tilt, 2) + ",";
        payload += "\"vibration\":" + String(vibAnalog) + ",";
        payload += "\"locked\":" + String(locked ? "true" : "false");
        payload += "}";
        if (ensureWiFiConnected() && millis() - lastPost >= POST_COOLDOWN_MS) {
          firebasePutLive(payload);
          lastPost = millis();
        }

        // Sensor-triggered alarm: wait for RFID within RFID_WAIT_MS
        if (abs(tilt) > TILT_THRESHOLD_DEG || accMag > ACCMAG_HIGH || accMag < ACCMAG_LOW ||
            vibAnalog > ANALOG_VIBRATION_THRESHOLD || vibDigital == (WAKE_ON_HIGH ? HIGH : LOW)) {
          Serial.println("Sensor-triggered activity: waiting for RFID tag for 10s...");
          bool authorized = false;
          bool tagSeen = waitForAuthorizedTagOnce(RFID_WAIT_MS, authorized);
          if (tagSeen && authorized) {
            Serial.println("Authorized tag presented - alarm canceled.");
            String lat2, lon2, utc2; getGPS(lat2, lon2, utc2);
            setLockState(false, lat2, lon2); // unlock
          } else {
            Serial.println("No authorized tag within 10s - sounding buzzer and sending alert.");
            sendAlertToFirebaseMinimal();
            // sound buzzer for BUZZER_DURATION_MS (10s)
            digitalWrite(BUZZER_PIN, HIGH);
            unsigned long buzzStart = millis();
            while (millis() - buzzStart < BUZZER_DURATION_MS) {
              // allow authorized RFID to stop alarm early
              if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
                bool match = (mfrc522.uid.size == AUTH_UID_LEN);
                if (match) {
                  for (byte i = 0; i < AUTH_UID_LEN; i++) {
                    if (mfrc522.uid.uidByte[i] != AUTH_UID[i]) { match = false; break; }
                  }
                }
                mfrc522.PICC_HaltA();
                mfrc522.PCD_StopCrypto1();
                if (match) {
                  Serial.println("Authorized tag presented during buzzer. Stopping alarm and toggling lock.");
                  digitalWrite(BUZZER_PIN, LOW);
                  String lat3, lon3, utc3; getGPS(lat3, lon3, utc3);
                  setLockState(!locked, lat3, lon3);
                  break;
                }
              }
              delay(50);
            }
            digitalWrite(BUZZER_PIN, LOW);
          }
        }
      } // end if activity

      delay(SAMPLE_DELAY_MS);
    } // end window loop

    if (!anyActivity) {
      Serial.println("No activity detected for entire monitor window. Proceeding to deep sleep.");
      break; // exit while(true) and go to sleep
    } else {
      Serial.println("Activity occurred during window; restarting monitor window (will only sleep when a full window is quiet).");
      // loop again to require a full quiet window before sleeping
    }
  } // end outer while
}

// ---------- button sequence handling ----------
void handleButtonPress(int btnIndex) {
  totalPresses++;
  Serial.print("Pressed button ");
  Serial.print(btnIndex + 1);
  Serial.print("  (press ");
  Serial.print(totalPresses);
  Serial.print("/");
  Serial.print(maxPresses);
  Serial.println(")");

  if (btnIndex == unlockSequence[seqPos]) {
    seqPos++;
    Serial.print("Correct so far: ");
    Serial.println(seqPos);
    if (seqPos >= sequenceLen) {
      // Sequence complete -> toggle lock/unlock
      Serial.println("Sequence correct: toggling lock state.");
      String latNow, lonNow, utcNow; getGPS(latNow, lonNow, utcNow);
      setLockState(!locked, latNow, lonNow);
      resetSequence();
    }
  } else {
    Serial.println("Wrong button - sequence reset");
    seqPos = 0;
  }

  if (totalPresses >= maxPresses) {
    // if not unlocked/locked by now, sound buzzer for 10s and send alert (minimal)
    Serial.println("ALERT: Correct sequence not entered within max presses! Sounding buzzer and posting alert.");
    // visual alert (fast flashes)
    flashLed(8, 100);
    // send minimal alert (no reason field)
    sendAlertToFirebaseMinimal();
    // sound buzzer for BUZZER_DURATION_MS
    digitalWrite(BUZZER_PIN, HIGH);
    unsigned long buzzStart = millis();
    while (millis() - buzzStart < BUZZER_DURATION_MS) {
      // allow authorized RFID to stop alarm early
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        bool match = (mfrc522.uid.size == AUTH_UID_LEN);
        if (match) {
          for (byte i = 0; i < AUTH_UID_LEN; i++) {
            if (mfrc522.uid.uidByte[i] != AUTH_UID[i]) { match = false; break; }
          }
        }
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        if (match) {
          Serial.println("Authorized tag presented during buzzer. Stopping alarm and toggling lock.");
          digitalWrite(BUZZER_PIN, LOW);
          String lat3, lon3, utc3; getGPS(lat3, lon3, utc3);
          setLockState(!locked, lat3, lon3);
          break;
        }
      }
      delay(50);
    }
    digitalWrite(BUZZER_PIN, LOW);
    resetSequence();
  }
}

void resetSequence() {
  seqPos = 0;
  totalPresses = 0;
  Serial.println("Sequence and press counter reset");
}

void flashLed(int times, int msDelay) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(msDelay);
    digitalWrite(LED_PIN, LOW);
    delay(msDelay);
  }
}

// ---------- set lock state helper ----------
void setLockState(bool lockState, const String &lat, const String &lon) {
  locked = lockState;
  // LED HIGH = unlocked, LOW = locked
  digitalWrite(LED_PIN, locked ? LOW : HIGH);

  // Build payload to update live node immediately
  String payload = "{";
  payload += "\"locked\":" + String(locked ? "true" : "false") + ",";
  payload += "\"lat\":" + (lat.length() ? lat : lastLat) + ",";
  payload += "\"lon\":" + (lon.length() ? lon : lastLon);
  payload += "}";

  Serial.print("Updating lock state to Firebase: "); Serial.println(payload);
  if (ensureWiFiConnected()) {
    if (firebasePutLive(payload)) Serial.println("Lock state posted.");
    else Serial.println("Failed to post lock state.");
  }
}

// ---------- send minimal alert helper ----------
void sendAlertToFirebaseMinimal() {
  // Minimal alert payload: just an alert flag and current locked state and coords
  String payload = "{";
  payload += "\"alert\":true,";
  payload += "\"locked\":" + String(locked ? "true" : "false") + ",";
  payload += "\"lat\":" + lastLat + ",";
  payload += "\"lon\":" + lastLon;
  payload += "}";
  Serial.print("Posting minimal alert to Firebase: "); Serial.println(payload);
  if (ensureWiFiConnected()) {
    if (firebasePostAlert(payload)) Serial.println("Alert posted.");
    else Serial.println("Failed to post alert.");
  } else {
    Serial.println("Cannot post alert: WiFi not connected.");
  }
}

// ---------- RFID helper (wait up to timeout; returns true if any tag seen; authorized set accordingly) ----------
bool waitForAuthorizedTagOnce(unsigned long timeoutMs, bool &authorized) {
  unsigned long start = millis();
  authorized = false;
  bool seen = false;
  while (millis() - start < timeoutMs) {
    if (!mfrc522.PICC_IsNewCardPresent()) { delay(50); continue; }
    if (!mfrc522.PICC_ReadCardSerial()) { delay(50); continue; }

    seen = true;
    Serial.print("Tag UID:");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      Serial.print(" ");
      if (mfrc522.uid.uidByte[i] < 0x10) Serial.print("0");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();

    if (mfrc522.uid.size == AUTH_UID_LEN) {
      bool match = true;
      for (byte i = 0; i < AUTH_UID_LEN; i++) {
        if (mfrc522.uid.uidByte[i] != AUTH_UID[i]) { match = false; break; }
      }
      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
      if (match) {
        Serial.println("Authorized UID matched.");
        authorized = true;
        return true;
      } else {
        Serial.println("UID not authorized.");
        return true; // tag seen but not authorized
      }
    } else {
      Serial.println("UID length mismatch.");
      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
      return true;
    }
  }
  return false; // no tag seen
}

// ---------- SIM + GPS helpers ----------
void flushSim() { while (simSerial.available()) simSerial.read(); }

String readSimLine(uint32_t timeout) {
  String s;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (simSerial.available()) {
      char c = (char)simSerial.read();
      s += c;
      if (c == '\n') return s;
    }
    delay(5);
  }
  return s;
}

bool getGPS(String &outLat, String &outLon, String &outUtc) {
  flushSim();
  simSerial.println("AT+CGNSINF");
  unsigned long start = millis();
  String buffer;
  while (millis() - start < 2000) {
    buffer += readSimLine(400);
    int p = buffer.indexOf("+CGNSINF:");
    if (p >= 0) {
      int nl = buffer.indexOf('\n', p);
      String line = (nl > p) ? buffer.substring(p, nl) : buffer.substring(p);
      int colon = line.indexOf(':');
      String payload = (colon >= 0) ? line.substring(colon + 1) : line;
      payload.trim();
      String tokens[12];
      int t = 0;
      int idx = 0;
      while (idx < (int)payload.length() && t < 12) {
        int comma = payload.indexOf(',', idx);
        if (comma < 0) comma = payload.length();
        tokens[t++] = payload.substring(idx, comma);
        idx = comma + 1;
      }
      if (t >= 5) {
        String fix = tokens[1];
        String utc = tokens[2];
        String lat = tokens[3];
        String lon = tokens[4];
        lat.trim(); lon.trim(); utc.trim();
        if (fix == "1" && lat.length() > 0 && lon.length() > 0) {
          outLat = lat;
          outLon = lon;
          outUtc = utc;
          // persist last known good fix as numeric strings
          lastLat = lat;
          lastLon = lon;
          return true;
        } else {
          outLat = "";
          outLon = "";
          outUtc = "";
          return false;
        }
      }
    }
    delay(50);
  }
  return false;
}

// ---------- WiFi and Firebase helpers ----------
bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: "); Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("WiFi connection failed.");
  return false;
}

String buildFirebaseUrlLive() {
  String base = String(FIREBASE_BASE);
  base.trim();
  if (base.endsWith(".json")) {
    if (base.endsWith("/")) base.remove(base.length() - 1);
    return base;
  }
  if (base.endsWith("/")) base.remove(base.length() - 1);
  return base + "/bicycle/live.json";
}

String buildFirebaseUrlCommand() {
  String base = String(FIREBASE_BASE);
  base.trim();
  if (base.endsWith(".json")) {
    int lastSlash = base.lastIndexOf('/');
    if (lastSlash > 0) {
      String root = base.substring(0, lastSlash);
      return root + "/bicycle/command.json";
    }
    return base;
  }
  if (base.endsWith("/")) base.remove(base.length() - 1);
  return base + "/bicycle/command.json";
}

String buildFirebaseUrlAlerts() {
  String base = String(FIREBASE_BASE);
  base.trim();
  if (base.endsWith(".json")) {
    int lastSlash = base.lastIndexOf('/');
    if (lastSlash > 0) {
      String root = base.substring(0, lastSlash);
      return root + "/bicycle/alerts.json";
    }
    return base;
  }
  if (base.endsWith("/")) base.remove(base.length() - 1);
  return base + "/bicycle/alerts.json";
}

bool firebasePutLive(const String &json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("HTTP: WiFi not connected");
    return false;
  }

  String url = buildFirebaseUrlLive();
  if (FIREBASE_AUTH && strlen(FIREBASE_AUTH) > 0) {
    if (url.indexOf('?') >= 0) url += "&auth=" + String(FIREBASE_AUTH);
    else url += "?auth=" + String(FIREBASE_AUTH);
  }

  Serial.print("Final PUT URL: "); Serial.println(url);
  Serial.print("Payload: "); Serial.println(json);

  WiFiClientSecure *client = new WiFiClientSecure();
  client->setInsecure(); // testing only; remove for production

  HTTPClient http;
  http.begin(*client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(json);
  String resp = http.getString();
  Serial.printf("HTTP %d: %s\n", code, resp.c_str());
  http.end();
  delete client;
  return (code >= 200 && code < 300);
}

bool firebasePostAlert(const String &json) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("HTTP: WiFi not connected");
    return false;
  }

  String url = buildFirebaseUrlAlerts();
  if (FIREBASE_AUTH && strlen(FIREBASE_AUTH) > 0) {
    if (url.indexOf('?') >= 0) url += "&auth=" + String(FIREBASE_AUTH);
    else url += "?auth=" + String(FIREBASE_AUTH);
  }

  Serial.print("Final POST URL: "); Serial.println(url);
  Serial.print("Alert payload: "); Serial.println(json);

  WiFiClientSecure *client = new WiFiClientSecure();
  client->setInsecure(); // testing only; remove for production

  HTTPClient http;
  http.begin(*client, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  String resp = http.getString();
  Serial.printf("HTTP POST %d: %s\n", code, resp.c_str());
  http.end();
  delete client;
  return (code >= 200 && code < 300);
}

String firebaseGetCommand() {
  if (WiFi.status() != WL_CONNECTED) return String();

  String url = buildFirebaseUrlCommand();
  if (FIREBASE_AUTH && strlen(FIREBASE_AUTH) > 0) {
    if (url.indexOf('?') >= 0) url += "&auth=" + String(FIREBASE_AUTH);
    else url += "?auth=" + String(FIREBASE_AUTH);
  }

  Serial.print("GET command URL: "); Serial.println(url);

  WiFiClientSecure *client = new WiFiClientSecure();
  client->setInsecure(); // testing only

  HTTPClient http;
  http.begin(*client, url);
  int code = http.GET();
  String resp = "";
  if (code > 0) {
    resp = http.getString();
    Serial.printf("HTTP GET %d: %s\n", code, resp.c_str());
  } else {
    Serial.printf("HTTP GET failed, code: %d\n", code);
  }
  http.end();
  delete client;

  resp.trim();
  if (resp.length() == 0 || resp == "null") return String();
  if (resp.startsWith("\"") && resp.endsWith("\"") && resp.length() >= 2) resp = resp.substring(1, resp.length() - 1);

  int idx = resp.indexOf("\"command\"");
  if (idx >= 0) {
    int colon = resp.indexOf(':', idx);
    if (colon >= 0) {
      int q1 = resp.indexOf('"', colon);
      if (q1 >= 0) {
        int q2 = resp.indexOf('"', q1 + 1);
        if (q2 > q1) {
          String cmd = resp.substring(q1 + 1, q2);
          return cmd;
        }
      }
    }
  }
  return resp;
}