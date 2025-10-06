#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WebServer.h>
#include <EEPROM.h>

// EEPROM設定
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xABCD  // 設定が有効かチェックする魔法の数値

// ピン定義
const int RED_LED_PIN = 33;
const int GREEN_LED_PIN = 32;
const int INIT_PIN = 0;  // GPIO0: 初期化ピン

// 設定構造体
struct Config {
    uint16_t magic;  // 設定の有効性を確認
    char wifi_ssid[32];
    char wifi_pass[64];
    uint8_t ip_addr[4];
    uint8_t gateway[4];
    uint8_t subnet[4];
    char mqtt_broker[64];
    uint16_t mqtt_port;
    char mqtt_client_id[32];
    char mqtt_topic[64];
    uint16_t detection_distance;  // 検知距離(mm)
    uint16_t deposit_duration;    // 預かり判定時間(秒)
    uint16_t pickup_duration;     // 取り出し判定時間(秒)
};

// デフォルト設定値
const Config DEFAULT_CONFIG = {
    EEPROM_MAGIC,
    "TP-Link_2G",
    "5e0f624603e22",
    {192, 168, 0, 210},      // IP
    {192, 168, 0, 1},    // Gateway
    {255, 255, 255, 0},      // Subnet
    "192.168.0.251",           // MQTT Broker
    1883,                    // MQTT Port
    "ESP32_Locker_1",              // MQTT Client ID
    "locker/sensor",                      // MQTT Topic
    400,                     // 検知距離 400mm
    10,                      // 預かり判定 10秒
    10                       // 取り出し判定 10秒
};

// グローバル設定
Config config;

const int ERROR_DISTANCE = 8191;     // VL53L0Xのエラー値

// VL53L0Xオブジェクト
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// WiFiとMQTTクライアント
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer server(80);

// 状態管理
bool isOccupied = false;
bool lastOccupiedState = false;  // 前回の預かり状態
unsigned long detectionStartTime = 0;
unsigned long pickupStartTime = 0;  // 取り出し検知開始時刻
unsigned long lastSendTime = 0;
int currentDistance = 0;

// 移動平均フィルター用
const int FILTER_SIZE = 5;  // 5回の平均を取る
int distanceBuffer[FILTER_SIZE] = {9999, 9999, 9999, 9999, 9999};
int bufferIndex = 0;

// 関数プロトタイプ
void connectWiFi();
void connectMQTT();
void sendStatus();
void updateLEDs();
void loadConfig();
void saveConfig();
void resetConfig();
void handleRoot();
void handleSave();
void handleNotFound();

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=================================");
    Serial.println("ESP32 Locker Sensor System");
    Serial.println("with VL53L0X ToF Sensor");
    Serial.println("=================================");

    // LED初期化
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);

    // 初期化ピン設定
    pinMode(INIT_PIN, INPUT_PULLUP);

    // EEPROM初期化
    EEPROM.begin(EEPROM_SIZE);

    // GPIO0チェック: Low=初期化（GND接続時）、High=通常起動
    if (digitalRead(INIT_PIN) == LOW) {
        Serial.println("*** 初期化モード: 設定をデフォルト値にリセット ***");
        resetConfig();

        // 初期化完了LED表示（赤緑同時点滅3回）
        for(int i = 0; i < 3; i++) {
            digitalWrite(RED_LED_PIN, HIGH);
            digitalWrite(GREEN_LED_PIN, HIGH);
            delay(500);
            digitalWrite(RED_LED_PIN, LOW);
            digitalWrite(GREEN_LED_PIN, LOW);
            delay(500);
        }
    } else {
        Serial.println("通常起動モード");
        // 起動時LED点滅
        for(int i = 0; i < 3; i++) {
            digitalWrite(RED_LED_PIN, HIGH);
            digitalWrite(GREEN_LED_PIN, HIGH);
            delay(1000);
            digitalWrite(RED_LED_PIN, LOW);
            digitalWrite(GREEN_LED_PIN, LOW);
            delay(1000);
        }
    }

    // 設定読み込み
    loadConfig();

    // I2C初期化
    Wire.begin();

    //VL53L0X初期化
    Serial.println("VL53L0X初期化中...");
    if (!lox.begin()) {
        Serial.println("VL53L0Xセンサーが見つかりません！");
        Serial.println("配線を確認してください：");
        Serial.println("  VIN -> 3.3V");
        Serial.println("  GND -> GND");
        Serial.println("  SDA -> GPIO21");
        Serial.println("  SCL -> GPIO22");

        // エラー時は赤LED点滅
        while(1) {
            digitalWrite(RED_LED_PIN, HIGH);
            delay(500);
            digitalWrite(RED_LED_PIN, LOW);
            delay(500);
        }
    }
    Serial.println("VL53L0X初期化完了！");

    // 測定タイミングバジェット設定（測定時間を長くして精度向上）
    Serial.println("測定モード設定中...");
    lox.setMeasurementTimingBudgetMicroSeconds(200000);  // 200ms（デフォルトは33ms）
    Serial.println("測定タイミング: 200ms (長距離・高精度モード)");

    // WiFi接続
    connectWiFi();

    // Webサーバー設定
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Webサーバー起動完了！");
    Serial.print("設定画面URL: http://");
    Serial.println(WiFi.localIP());

    // MQTT設定
    mqttClient.setServer(config.mqtt_broker, config.mqtt_port);
    connectMQTT();

    Serial.println("システム起動完了！");
    Serial.println("---------------------------------");
}

void loop() {
    // Webサーバー処理
    server.handleClient();

    // MQTT接続維持
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();

    // 距離測定
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    // RangeStatusが0,1,2の場合は測定値を使用（4=範囲外のみ除外）
    // 0: 正常, 1: シグマエラー（精度低いが使用可）, 2: 信号エラー（弱い信号だが使用可）
    // 4: 範囲外, 5: ハードウェアエラー
    int rawDistance = 9999;  // 生の測定値

    if (measure.RangeStatus != 4 && measure.RangeStatus != 5) {  // 範囲外とHWエラー以外は使用
        rawDistance = measure.RangeMilliMeter;

        // エラー値のチェック
        if (rawDistance == ERROR_DISTANCE) {
            rawDistance = 9999;  // 範囲外として扱う
        }
    } else {
        // 測定エラー（範囲外またはハードウェアエラー）
        rawDistance = 9999;
        static unsigned long lastErrorTime = 0;
        if (millis() - lastErrorTime > 1000) {
            Serial.print("測定エラー - RangeStatus: ");
            Serial.print(measure.RangeStatus);
            if (measure.RangeStatus == 4) {
                Serial.println(" (範囲外: 対象物が遠すぎるか、反射率が低い)");
            } else if (measure.RangeStatus == 5) {
                Serial.println(" (ハードウェアエラー)");
            } else {
                Serial.println();
            }
            lastErrorTime = millis();
        }
    }

    // 移動平均フィルター処理
    distanceBuffer[bufferIndex] = rawDistance;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    // 平均値を計算
    long sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += distanceBuffer[i];
    }
    currentDistance = sum / FILTER_SIZE;

    // デバッグ出力（1秒ごと）
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        Serial.print("距離: ");
        Serial.print(currentDistance);
        Serial.print(" mm");
        if (currentDistance <= config.detection_distance) {
            Serial.print(" [検知中]");
            if (detectionStartTime > 0) {
                Serial.print(" (");
                Serial.print((millis() - detectionStartTime) / 1000);
                Serial.print("秒経過)");
            }
        }
        Serial.println();
        lastDebugTime = millis();
    }

    // 物体検知処理
    if (currentDistance <= config.detection_distance) {
        // 物体が検知距離内にある

        if (isOccupied && pickupStartTime > 0) {
            // 預かり状態で取り出し検知中に物体が戻った
            Serial.println("取り出しキャンセル（荷物が戻りました）");
            pickupStartTime = 0;  // 取り出し検知をキャンセル
        } else if (!isOccupied) {
            // 預かり前の検知処理
            if (detectionStartTime == 0) {
                detectionStartTime = millis();
                Serial.println("物体検知開始！");
            }

            // 設定秒数以上継続したら預かり状態に
            if (millis() - detectionStartTime >= config.deposit_duration * 1000) {
                isOccupied = true;
                lastOccupiedState = true;
                Serial.println("=================================");
                Serial.println("荷物預かり開始！");
                Serial.println("occupied: true に変更");
                Serial.println("=================================");
                sendStatus();  // 預かり状態をMQTT送信（状態変化時）
                lastSendTime = millis();  // 送信時刻を更新
            }
        }
    } else {
        // 物体が検知距離から離れた

        if (isOccupied) {
            // 預かり中の荷物が取り出された
            if (pickupStartTime == 0) {
                // 取り出し検知開始
                pickupStartTime = millis();
                Serial.println("荷物取り出し検知開始...");
            }

            // 取り出し判定時間が経過したら取り出し完了
            if (millis() - pickupStartTime >= config.pickup_duration * 1000) {
                Serial.println("=================================");
                Serial.println("荷物取り出し完了！");
                Serial.println("occupied: false に変更");
                Serial.println("=================================");
                isOccupied = false;
                lastOccupiedState = false;
                sendStatus();  // 取り出し状態をMQTT送信（状態変化時）
                lastSendTime = millis();  // 送信時刻を更新
                detectionStartTime = 0;
                pickupStartTime = 0;
            }
        } else if (detectionStartTime > 0) {
            // 預かり前の検知がキャンセルされた
            Serial.print("検知キャンセル（");
            Serial.print(config.deposit_duration);
            Serial.println("秒未満）");
            detectionStartTime = 0;
        }
    }

    // LED更新
    updateLEDs();

    // 定期送信（10秒ごと、状態変化時は即座に送信済み）
    if (millis() - lastSendTime >= 10000) {
        sendStatus();
        lastSendTime = millis();
    }

    delay(100);  // 測定間隔
}

void connectWiFi() {
    Serial.print("WiFi接続中: ");
    Serial.println(config.wifi_ssid);

    // 固定IP設定
    IPAddress local_IP(config.ip_addr[0], config.ip_addr[1], config.ip_addr[2], config.ip_addr[3]);
    IPAddress gateway(config.gateway[0], config.gateway[1], config.gateway[2], config.gateway[3]);
    IPAddress subnet(config.subnet[0], config.subnet[1], config.subnet[2], config.subnet[3]);

    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("固定IP設定失敗！");
    }

    WiFi.begin(config.wifi_ssid, config.wifi_pass);

    // 接続待機（LED点滅）
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 50) {
        digitalWrite(RED_LED_PIN, HIGH);
        delay(250);
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
        delay(250);
        digitalWrite(GREEN_LED_PIN, LOW);
        Serial.print(".");
        retry++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi接続成功！");
        Serial.print("IPアドレス: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi接続失敗！");
    }
}

void connectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("MQTTサーバー接続中: ");
        Serial.println(config.mqtt_broker);

        if (mqttClient.connect(config.mqtt_client_id)) {
            Serial.println("MQTT接続成功！");

            // 初回ステータス送信
            sendStatus();
        } else {
            Serial.print("MQTT接続失敗 rc=");
            Serial.println(mqttClient.state());
            Serial.println("5秒後に再試行...");

            // エラー時LED点滅
            for(int i = 0; i < 5; i++) {
                digitalWrite(RED_LED_PIN, HIGH);
                delay(500);
                digitalWrite(RED_LED_PIN, LOW);
                delay(500);
            }
        }
    }
}

void sendStatus() {
    // MQTT接続チェック
    if (!mqttClient.connected()) {
        Serial.println("MQTT未接続のため送信スキップ");
        return;
    }

    // JSON作成
    StaticJsonDocument<256> doc;
    doc["lockerId"] = 1;
    doc["occupied"] = isOccupied;
    doc["distance"] = currentDistance;
    doc["timestamp"] = millis();

    // 検知中の情報
    if (detectionStartTime > 0 && !isOccupied) {
        doc["detecting"] = true;
        doc["detectTime"] = (millis() - detectionStartTime) / 1000;
    } else {
        doc["detecting"] = false;
    }

    // JSON文字列化
    char buffer[256];
    serializeJson(doc, buffer);

    // MQTT送信（トピックが空の場合は送信しない）
    if (strlen(config.mqtt_topic) > 0) {
        if (mqttClient.publish(config.mqtt_topic, buffer)) {
            Serial.print("MQTT送信成功: ");
            Serial.println(buffer);
        } else {
            Serial.print("MQTT送信失敗！ rc=");
            Serial.println(mqttClient.state());
        }
    } else {
        Serial.println("MQTTトピックが未設定のため送信スキップ");
    }
}

void updateLEDs() {
    if (!WiFi.isConnected() || !mqttClient.connected()) {
        // 接続エラー時：赤緑交互点滅
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        if (millis() - lastBlink > 500) {
            if (blinkState) {
                digitalWrite(RED_LED_PIN, HIGH);
                digitalWrite(GREEN_LED_PIN, LOW);
            } else {
                digitalWrite(RED_LED_PIN, LOW);
                digitalWrite(GREEN_LED_PIN, HIGH);
            }
            blinkState = !blinkState;
            lastBlink = millis();
        }
    } else if (isOccupied && pickupStartTime > 0) {
        // 預かり状態で取り出し検知中：赤LED 1秒間隔で点滅
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        if (millis() - lastBlink > 1000) {
            digitalWrite(RED_LED_PIN, blinkState);
            blinkState = !blinkState;
            lastBlink = millis();
        }
        digitalWrite(GREEN_LED_PIN, LOW);
    } else if (isOccupied) {
        // 預かり状態（取り出し検知前）：赤LED点灯
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
    } else if (detectionStartTime > 0) {
        // 検知中：緑LED 1秒間隔で点滅
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        if (millis() - lastBlink > 1000) {
            digitalWrite(GREEN_LED_PIN, blinkState);
            blinkState = !blinkState;
            lastBlink = millis();
        }
        digitalWrite(RED_LED_PIN, LOW);
    } else {
        // 空き状態：緑LED点灯
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
    }
}

// EEPROM関数
void loadConfig() {
    Serial.println("EEPROMから設定読み込み中...");
    EEPROM.get(0, config);

    // 設定の有効性チェック
    if (config.magic != EEPROM_MAGIC) {
        Serial.println("有効な設定が見つかりません。デフォルト値を使用します。");
        resetConfig();
    } else {
        Serial.println("設定読み込み完了！");
        Serial.print("  WiFi SSID: ");
        Serial.println(config.wifi_ssid);
        Serial.print("  IP: ");
        Serial.print(config.ip_addr[0]); Serial.print(".");
        Serial.print(config.ip_addr[1]); Serial.print(".");
        Serial.print(config.ip_addr[2]); Serial.print(".");
        Serial.println(config.ip_addr[3]);
        Serial.print("  MQTT Broker: ");
        Serial.println(config.mqtt_broker);
        Serial.print("  検知距離: ");
        Serial.print(config.detection_distance);
        Serial.println("mm");
    }
}

void saveConfig() {
    Serial.println("設定をEEPROMに保存中...");
    config.magic = EEPROM_MAGIC;
    EEPROM.put(0, config);
    EEPROM.commit();
    Serial.println("保存完了！");
}

void resetConfig() {
    Serial.println("設定を初期値にリセット中...");
    config = DEFAULT_CONFIG;
    saveConfig();
}

// Webサーバーハンドラー
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>ESP32 ロッカーセンサー 設定</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; background: #f0f0f0; }";
    html += "h1 { color: #333; text-align: center; }";
    html += "h2 { color: #666; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; margin-top: 30px; }";
    html += ".form-group { margin: 15px 0; }";
    html += "label { display: block; margin-bottom: 5px; color: #555; font-weight: bold; }";
    html += "input[type='text'], input[type='password'], input[type='number'] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }";
    html += "button { background: #4CAF50; color: white; padding: 12px 30px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; width: 100%; margin-top: 20px; }";
    html += "button:hover { background: #45a049; }";
    html += ".container { background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>ESP32 ロッカーセンサー設定</h1>";
    html += "<form method='POST' action='/save'>";

    html += "<h2>ネットワーク設定</h2>";
    html += "<div class='form-group'><label>WiFi SSID:</label>";
    html += "<input type='text' name='wifi_ssid' value='" + String(config.wifi_ssid) + "' required></div>";
    html += "<div class='form-group'><label>WiFi パスワード:</label>";
    html += "<input type='password' name='wifi_pass' value='" + String(config.wifi_pass) + "'></div>";
    html += "<div class='form-group'><label>IPアドレス:</label>";
    html += "<input type='text' name='ip_addr' value='";
    html += String(config.ip_addr[0]) + "." + String(config.ip_addr[1]) + "." + String(config.ip_addr[2]) + "." + String(config.ip_addr[3]);
    html += "' required></div>";
    html += "<div class='form-group'><label>ゲートウェイ:</label>";
    html += "<input type='text' name='gateway' value='";
    html += String(config.gateway[0]) + "." + String(config.gateway[1]) + "." + String(config.gateway[2]) + "." + String(config.gateway[3]);
    html += "' required></div>";
    html += "<div class='form-group'><label>サブネットマスク:</label>";
    html += "<input type='text' name='subnet' value='";
    html += String(config.subnet[0]) + "." + String(config.subnet[1]) + "." + String(config.subnet[2]) + "." + String(config.subnet[3]);
    html += "' required></div>";

    html += "<h2>MQTT設定</h2>";
    html += "<div class='form-group'><label>MQTTブローカー:</label>";
    html += "<input type='text' name='mqtt_broker' value='" + String(config.mqtt_broker) + "' required></div>";
    html += "<div class='form-group'><label>MQTTポート:</label>";
    html += "<input type='number' name='mqtt_port' value='" + String(config.mqtt_port) + "' required></div>";
    html += "<div class='form-group'><label>MQTTクライアントID:</label>";
    html += "<input type='text' name='mqtt_client_id' value='" + String(config.mqtt_client_id) + "' required></div>";
    html += "<div class='form-group'><label>MQTTトピック:</label>";
    html += "<input type='text' name='mqtt_topic' value='" + String(config.mqtt_topic) + "'></div>";

    html += "<h2>センサー設定</h2>";
    html += "<div class='form-group'><label>検知距離 (mm):</label>";
    html += "<input type='number' name='detection_distance' value='" + String(config.detection_distance) + "' required></div>";
    html += "<div class='form-group'><label>預かり判定時間 (秒):</label>";
    html += "<input type='number' name='deposit_duration' value='" + String(config.deposit_duration) + "' required></div>";
    html += "<div class='form-group'><label>取り出し判定時間 (秒):</label>";
    html += "<input type='number' name='pickup_duration' value='" + String(config.pickup_duration) + "' required></div>";

    html += "<button type='submit'>設定を更新</button>";
    html += "</form></div></body></html>";

    server.send(200, "text/html", html);
}

void handleSave() {
    // IPアドレスのパース
    String ip_str = server.arg("ip_addr");
    sscanf(ip_str.c_str(), "%hhu.%hhu.%hhu.%hhu",
           &config.ip_addr[0], &config.ip_addr[1], &config.ip_addr[2], &config.ip_addr[3]);

    String gw_str = server.arg("gateway");
    sscanf(gw_str.c_str(), "%hhu.%hhu.%hhu.%hhu",
           &config.gateway[0], &config.gateway[1], &config.gateway[2], &config.gateway[3]);

    String sn_str = server.arg("subnet");
    sscanf(sn_str.c_str(), "%hhu.%hhu.%hhu.%hhu",
           &config.subnet[0], &config.subnet[1], &config.subnet[2], &config.subnet[3]);

    // 文字列設定
    strncpy(config.wifi_ssid, server.arg("wifi_ssid").c_str(), sizeof(config.wifi_ssid) - 1);
    strncpy(config.wifi_pass, server.arg("wifi_pass").c_str(), sizeof(config.wifi_pass) - 1);
    strncpy(config.mqtt_broker, server.arg("mqtt_broker").c_str(), sizeof(config.mqtt_broker) - 1);
    strncpy(config.mqtt_client_id, server.arg("mqtt_client_id").c_str(), sizeof(config.mqtt_client_id) - 1);
    strncpy(config.mqtt_topic, server.arg("mqtt_topic").c_str(), sizeof(config.mqtt_topic) - 1);

    // 数値設定
    config.mqtt_port = server.arg("mqtt_port").toInt();
    config.detection_distance = server.arg("detection_distance").toInt();
    config.deposit_duration = server.arg("deposit_duration").toInt();
    config.pickup_duration = server.arg("pickup_duration").toInt();

    // EEPROMに保存
    saveConfig();

    // 完了ページ
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>設定完了</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; background: #f0f0f0; text-align: center; }";
    html += ".container { background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
    html += "h1 { color: #4CAF50; }";
    html += "p { color: #666; font-size: 18px; }";
    html += "a { display: inline-block; margin-top: 20px; padding: 10px 20px; background: #4CAF50; color: white; text-decoration: none; border-radius: 4px; }";
    html += "a:hover { background: #45a049; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<h1>設定が保存されました！</h1>";
    html += "<p>設定を反映するには、ESP32を再起動してください。</p>";
    html += "<a href='/'>設定画面に戻る</a>";
    html += "</div></body></html>";

    server.send(200, "text/html", html);
}

void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}