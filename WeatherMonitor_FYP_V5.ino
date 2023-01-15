#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sntp.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>
#include <Math.h>
#include <Preferences.h>

DHT dht(33, DHT22);
Adafruit_BME280 bme;
BH1750 bh = BH1750(0x23);
struct RainSensor {
  const uint8_t digital = 34;
  const uint8_t analog = 35;
} rainSensor;

//Sampled from https://meteologix.com at Alor Star between January 1st to 3rd
int seaLevelPressure[][24] = { { 14, 14, 14, 13, 12, 12, 12, 12, 13, 13, 14, 14, 13, 12, 11, 10, 9, 8, 9, 9, 11, 12, 13, 13 },
                               { 14, 14, 13, 12, 12, 12, 12, 12, 13, 13, 14, 15, 14, 13, 12, 11, 10, 9, 10, 10, 11, 13, 13, 14 },
                               { 13, 13, 13, 12, 12, 12, 12, 12, 13, 13, 14, 13, 13, 12, 11, 10, 9, 8, 9, 10, 11, 12, 13, 14 } };

struct SensorsData {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
  int light;
  int rain;
} sensorData;

File dataFile;
char fileType[] = ".csv";
File logFile;
String logFileName = "/log.txt";

struct PressReleaseMillis {
  unsigned long released = 0;
  unsigned long pressed = 0;
};

struct Encoder {
  const uint8_t clk = 25;
  const uint8_t dt = 26;
  const uint8_t sw = 27;
  bool currentRotaryState;
  bool previousRotaryState;
  bool currentState;
  bool previouState = HIGH;
  bool isHolding = false;
  PressReleaseMillis millis;
} encoder;

struct Button {
  const uint8_t pin = 0;
  bool currentState;
  bool previouState = HIGH;
  bool isHolding = false;
  PressReleaseMillis millis;
} button;

const int inputHold_Settings = 3000;
const int inputHold_ForceRestart = 5000;

const int debounce = 50;

#define LED_ERROR 17

//I2C starts from 0x27 by default
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

#define SD_SPI 5
bool isSDAvailable = false;

//Wifi Config
const char* ssid = "TitanicSyncing";
const char* password = "cheongyf";
const int network_timeout = 5000;

//Favoriot
#define API_ACCESS_TOKEN "BGXNsEtJsB0roistgClyxeVC1SyHioZe"
#define DEV_ID "HibiscusSense@CHEONGYOOKFAI"

//Time Servers
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
int time_zone = 8;
struct tm timeinfo;

int sensorChecks = 2;
int wifiChecks = 1;
int sdChecks = 1;

bool systemInitialized = false;

//LCD State Machine
enum MenuStates {
  MAIN,
  SENSOR_A,
  SENSOR_B,
  SLEEP,
  SETTINGS_INTERVAL,
  SETTINGS_INTERVAL_NEWINTERVAL,
  SETTINGS_INTERVAL_STARTCHOICE,
  SETTINGS_INTERVAL_NEWSTART,
  SETTINGS_CARD,
  SETTINGS_CARD_INSERT,
  SETTINGS_CARD_EJECT,
  SETTINGS_CARD_EJECT_CONFIRM,
  CHANGES_DONE
};

struct Menu {
  MenuStates currentState = MAIN;
  MenuStates previouState;
} menu;

bool updateMenu = false;

//Timers
hw_timer_t* timer_Seconds = NULL;
hw_timer_t* timer_DataUpdate = NULL;
hw_timer_t* timer_Timeout = NULL;

//Timer duration
const int timeoutDuration_Main = 10000;
const int timeoutDuration_Sensors = 3000;
const int timeoutDuration_Settings = 3000;

//Update data to serial & cloud every X minute
int dataUpdate_Interval;
unsigned int minutesPassed;
const unsigned int minuteDataUpdate_Interval = 60 * 1000;  //1 minute to millis

int newHour, newMinute, newSecond;
String hourScheduled, minuteScheduled, secondScheduled;
String nextScheduledUpdate;
bool hourSet = false;
bool minuteSet = false;
bool secondSet = false;

enum UpdateTime {
  CURRENT,
  SCHEDULE
};

UpdateTime updateTimeSel = CURRENT;
bool startUpdateNow = false;
bool waitingForNewUpdateTime = false;
bool scheduledUpdate = false;

Preferences preferences;

#pragma region LCD_Custom_Chars
byte CheckMark[8] = {
  0b00000,
  0b00000,
  0b00001,
  0b00010,
  0b10100,
  0b01000,
  0b00000,
  0b00000
};

byte CrossMark[8] = {
  0b00000,
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000
};

byte SDCard[8] = {
  0b00000,
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b00000
};

byte Network[8] = {
  0b00000,
  0b10000,
  0b10000,
  0b10100,
  0b10100,
  0b10101,
  0b10101,
  0b00000
};
#pragma endregion LCD_Custom_Chars

#pragma region Initialize_Functions
void LCDInit() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Initializing...");

  lcd.createChar(0, CheckMark);
  lcd.createChar(1, CrossMark);
  lcd.createChar(2, SDCard);
  lcd.createChar(3, Network);
}

void SensorInit() {
  Serial.println("");

  //DHT22 Temperature, Humidity
  dht.begin();

  //BME280 Altitude, Pressure
  if (!bme.begin()) {
    Serial.println("Failed to find Hibiscus Sense BME280 chip");
  } else {
    Serial.println("BME280 initialized");
    if (!systemInitialized) sensorChecks--;
  }

  //BH1750 Light
  if (!bh.begin()) {
    Serial.println("Failed to find BH1750");
  } else {
    Serial.println("BH1750 initialized");
    sensorChecks--;
  }

  //Rain SensorInit Rain
  pinMode(rainSensor.digital, INPUT);
  pinMode(rainSensor.analog, INPUT);

  if (!systemInitialized) {
    //Display check result on LCD
    lcd.setCursor(2, 1);
    lcd.print("Sensors");

    if (sensorChecks == 0) {
      lcd.setCursor(0, 1);
      lcd.write(0);
    } else {
      lcd.setCursor(0, 1);
      lcd.write(1);
    }
  }
}

void WifiInit() {
  Serial.print("Attempting to connect to WiFi");

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < network_timeout) {
    Serial.print(".");
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nConnection failed, please reset the device");
  } else {
    Serial.println("\nConnection success!");
    if (!systemInitialized) wifiChecks--;
  }

  if (!systemInitialized) {
    //Display check result on LCD
    lcd.setCursor(2, 2);
    lcd.print("Wi-Fi Connection");

    if (wifiChecks == 0) {
      lcd.setCursor(0, 2);
      lcd.write(0);
    } else {
      lcd.setCursor(0, 2);
      lcd.write(1);
    }
  }
}

void OTASetup() {
  if (wifiChecks == 0) {
    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void SetupInternetTime() {
  configTime(3600 * time_zone, 0, ntpServer1, ntpServer2);

  if (!getLocalTime(&timeinfo)) {
    Serial.println("No time available (yet)");
    return;
  }
}

void SDInit() {
  if (!SD.begin(SD_SPI)) {
    Serial.println("Card Mount Failed");
  } else {
    Serial.println("Card Mount Initialized");
    if (SD.cardType() != CARD_NONE) {
      isSDAvailable = true;
      if (systemInitialized) LogToSD("SD card inserted");
    }
    if (!systemInitialized) sdChecks--;
  }

  if (!systemInitialized) {
    //Display check result on LCD
    lcd.setCursor(2, 3);
    lcd.print("SD Module");

    if (sdChecks == 0) {
      lcd.setCursor(0, 3);
      lcd.write(0);
    } else {
      lcd.setCursor(0, 3);
      lcd.write(1);
    }
  }
}
#pragma endregion

int GetSeaLevelPressure() {
  char hour[3];
  strftime(hour, sizeof(hour), "%H", &timeinfo);
  int currentHour = String(hour).toInt();

  double currentSeaLevelPressure = 0;
  for (int i = 0; i < (sizeof(seaLevelPressure) / sizeof(seaLevelPressure[0])); i++) {
    currentSeaLevelPressure += (1000 + seaLevelPressure[i][currentHour]);
  }

  currentSeaLevelPressure /= (sizeof(seaLevelPressure) / sizeof(seaLevelPressure[0]));
  return round(currentSeaLevelPressure);
}

void GetSensorValue() {
  getLocalTime(&timeinfo);

  sensorData.temperature = dht.readTemperature();
  sensorData.humidity = dht.readHumidity();
  sensorData.pressure = bme.readPressure() / 100.00;
  sensorData.altitude = bme.readAltitude(GetSeaLevelPressure());
  sensorData.light = bh.readLightLevel();
  sensorData.rain = analogRead(rainSensor.analog);

  Serial.println("");
  Serial.println(&timeinfo, "%A, %B %d %Y, %H:%M:%S");
  Serial.println("Temperature\t= " + String(sensorData.temperature) + " °C");
  Serial.println("Humidity\t= " + String(sensorData.humidity) + " %");
  Serial.println("Pressure\t= " + String(sensorData.pressure) + " hPa");
  Serial.println("Altitude\t= " + String(sensorData.altitude) + " m");
  Serial.println("Light\t\t= " + String(sensorData.light) + " lx");
  Serial.println("Rain\t\t= " + String(sensorData.rain));

  LogToSD("Sensor readings received");
}

void DataToSD() {
  Serial.println("\nSaving to SD Card...");
  char date[11];
  strftime(date, sizeof(date), "%d-%m-%Y", &timeinfo);
  String fileName = "/" + String(date) + String(fileType);

  if (!SD.exists(fileName)) {
    dataFile = SD.open(fileName, FILE_WRITE);
    dataFile.println("Time,Temperature,Humidity,Pressure,Altitude,Light,Rain");
    dataFile.close();
  }

  dataFile = SD.open(fileName, FILE_APPEND);

  if (dataFile) {
    dataFile.print(&timeinfo, "%T");
    dataFile.print("," + String(sensorData.temperature));
    dataFile.print("," + String(sensorData.humidity));
    dataFile.print("," + String(sensorData.pressure));
    dataFile.print("," + String(sensorData.altitude));
    dataFile.print("," + String(sensorData.light));
    dataFile.println("," + String(sensorData.rain));
    dataFile.close();
    Serial.println("Data saved");
    LogToSD("Data saved to SD card");
  } else {
    Serial.println("Error unable to save data");
    LogToSD("Failed to save data to SD card");
  }
}

void LogToSD(String message) {
  logFile = SD.open(logFileName, FILE_APPEND);

  if (logFile) {
    logFile.print(&timeinfo, "%d-%m-%Y, %T > ");
    logFile.println(message);
    logFile.close();
  }
}

double Round2d(double num) {
  return round(num * 100) / 100.0;
}

void DataToFavoriot() {
  Serial.println("\nSending to Favoriot...");
  StaticJsonDocument<200> doc;

  JsonObject root = doc.to<JsonObject>();
  root["device_developer_id"] = DEV_ID;

  JsonObject data = root.createNestedObject("data");
  data["Temperature Celsius"] = Round2d(sensorData.temperature);
  data["Humidity"] = Round2d(sensorData.humidity);
  data["Pressure"] = Round2d(sensorData.pressure);
  data["Altitude"] = Round2d(sensorData.altitude);
  data["Light Intensity"] = sensorData.light;
  data["Rain"] = sensorData.rain;

  String json;
  serializeJsonPretty(root, json);
  Serial.println(json);

  HTTPClient http;

  http.begin("http://apiv2.favoriot.com/v2/streams");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Apikey", API_ACCESS_TOKEN);

  int httpCode = http.POST(json);
  if (httpCode > 0) {
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

String SensorDataWithSpacing(String text, int row) {
  String newText;
  int spaceLength = 8 - text.length();
  int spaces = 9;

  for (int i = 0; i < spaceLength; i++) {
    spaces++;
  }

  lcd.setCursor(spaces, row);
  return (text);
}

void MenuPrint() {
  lcd.clear();

  switch (menu.currentState) {
    case MAIN:
    case SLEEP:
      getLocalTime(&timeinfo);

      //For reference https://cplusplus.com/reference/ctime/strftime/
      lcd.setCursor(6, 1);
      lcd.print(&timeinfo, "%T");
      lcd.setCursor(5, 2);
      lcd.print(&timeinfo, "%d-%m-%Y");

      lcd.setCursor(18, 3);
      (isSDAvailable) ? lcd.write(' ') : lcd.write('!');
      lcd.setCursor(19, 3);
      (isSDAvailable) ? lcd.write(' ') : lcd.write(2);

      lcd.setCursor(0, 0);
      (WiFi.status() == WL_CONNECTED) ? lcd.write(' ') : lcd.write('!');
      lcd.setCursor(1, 0);
      (WiFi.status() == WL_CONNECTED) ? lcd.write(' ') : lcd.write(3);
      break;

    case SENSOR_A:
      lcd.setCursor(0, 0);
      lcd.print("Temp    :");
      lcd.print(SensorDataWithSpacing(String(sensorData.temperature), 0) + " " + (char)223 + "C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity:");
      lcd.print(SensorDataWithSpacing(String(sensorData.humidity), 1) + "  %");
      lcd.setCursor(0, 2);
      lcd.print("Pressure:");
      lcd.print(SensorDataWithSpacing(String(sensorData.pressure), 2) + "hPa");
      lcd.setCursor(0, 3);
      lcd.print("Altitude:");
      lcd.print(SensorDataWithSpacing(String(sensorData.altitude), 3) + "  m");
      break;

    case SENSOR_B:
      lcd.setCursor(0, 0);
      lcd.print("Light   :");
      lcd.print(SensorDataWithSpacing(String(sensorData.light), 0) + " lx");
      lcd.setCursor(0, 1);
      lcd.print("Rain    :");
      lcd.print(SensorDataWithSpacing(String(sensorData.rain), 1));
      break;

    case SETTINGS_INTERVAL:
    case SETTINGS_CARD:
      for (int i = 1; i < 20 - 1; i++) {
        lcd.setCursor(i, 0);
        lcd.print("*");
      }
      lcd.setCursor(5, 0);
      lcd.print(" SETTINGS ");
      switch (menu.currentState) {
        case SETTINGS_INTERVAL:
          lcd.setCursor(2, 2);
          lcd.print("UPDATE  INTERVAL");
          break;

        case SETTINGS_CARD:
          lcd.setCursor(6, 2);
          lcd.print("SD  CARD");
          break;
      }
      break;

    case SETTINGS_INTERVAL_NEWINTERVAL:
      lcd.setCursor(2, 0);
      lcd.print("UPDATE  INTERVAL");
      lcd.setCursor(2, 2);
      lcd.print("Every " + String((dataUpdate_Interval < 10) ? " " : "") + String(dataUpdate_Interval) + " Minutes");
      break;

    case SETTINGS_INTERVAL_STARTCHOICE:
      lcd.setCursor(4, 0);
      lcd.print("START UPDATE");
      lcd.setCursor(4, 2);
      lcd.print("Now      New");
      (updateTimeSel == CURRENT) ? lcd.setCursor(2, 2) : lcd.setCursor(11, 2);
      lcd.print(">");
      break;

    case SETTINGS_INTERVAL_NEWSTART:
      lcd.setCursor(3, 0);
      lcd.print("NEW START TIME");
      lcd.setCursor(6, 2);
      nextScheduledUpdate = ((newHour < 10) ? "0" : "") + String(newHour) + ":" + ((newMinute < 10) ? "0" : "") + String(newMinute) + ":" + ((newSecond < 10) ? "0" : "") + String(newSecond);
      lcd.print(nextScheduledUpdate);
      break;

    case SETTINGS_CARD_INSERT:
      lcd.setCursor(6, 0);
      lcd.print("SD  CARD");
      lcd.setCursor(4, 2);
      lcd.print("Insert  Card");
      break;

    case SETTINGS_CARD_EJECT:
      lcd.setCursor(6, 0);
      lcd.print("SD  CARD");
      lcd.setCursor(5, 2);
      lcd.print("Eject Card");
      break;

    case SETTINGS_CARD_EJECT_CONFIRM:
      lcd.setCursor(5, 0);
      lcd.print("EJECT CARD");
      lcd.setCursor(5, 2);
      lcd.print("Confirm  ?");
      break;

    case CHANGES_DONE:
      switch (menu.previouState) {
        case SETTINGS_INTERVAL_STARTCHOICE:
        case SETTINGS_INTERVAL_NEWSTART:
          lcd.setCursor(2, 1);
          lcd.print("PREFERENCE SAVED");
          break;

        case SETTINGS_CARD_INSERT:
          if (isSDAvailable) {
            lcd.setCursor(8, 1);
            lcd.print("CARD");
            lcd.setCursor(6, 2);
            lcd.print("INSERTED");
          } else {
            lcd.setCursor(8, 1);
            lcd.print("CARD");
            lcd.setCursor(4, 2);
            lcd.print("NOT INSERTED");
          }
          break;

        case SETTINGS_CARD_EJECT_CONFIRM:
          lcd.setCursor(8, 1);
          lcd.print("CARD");
          lcd.setCursor(3, 2);
          lcd.print("SAFE TO REMOVE");
          break;
      }
      break;
  }
}

void MenuTransition(MenuStates nextState) {
  menu.previouState = menu.currentState;
  menu.currentState = nextState;

  timerAlarmDisable(timer_Timeout);

  switch (menu.currentState) {
    case MAIN:
      timerAlarmWrite(timer_Timeout, 1000 * timeoutDuration_Main, true);
      timerWrite(timer_Timeout, 0);
      timerAlarmEnable(timer_Timeout);
      break;

    case SENSOR_A:
    case SENSOR_B:
      timerAlarmWrite(timer_Timeout, 1000 * timeoutDuration_Sensors, true);
      timerWrite(timer_Timeout, 0);
      timerAlarmEnable(timer_Timeout);
      break;

    case CHANGES_DONE:
      timerAlarmWrite(timer_Timeout, 1000 * timeoutDuration_Settings, true);
      timerWrite(timer_Timeout, 0);
      timerAlarmEnable(timer_Timeout);
      break;
  }

  updateMenu = true;
}

void ReceiveEncoderSWInputs() {
  encoder.currentState = digitalRead(encoder.sw);

  if (encoder.currentState == LOW && encoder.previouState == HIGH) {
    if (millis() - encoder.millis.released >= debounce) {
      encoder.millis.pressed = millis();
      encoder.isHolding = false;
    }
  }

  if (encoder.currentState == HIGH && encoder.previouState == LOW) {
    if (millis() - encoder.millis.pressed >= debounce && !encoder.isHolding) {
      encoder.millis.released = millis();
      switch (menu.currentState) {
        case MAIN:
          GetSensorValue();
          MenuTransition(SENSOR_A);
          break;

        case SLEEP:
          MenuTransition(MAIN);
          break;

        case SETTINGS_INTERVAL:
          MenuTransition(SETTINGS_INTERVAL_NEWINTERVAL);
          break;

        case SETTINGS_INTERVAL_NEWINTERVAL:
          preferences.putUInt("updateInterval", dataUpdate_Interval);
          updateTimeSel = CURRENT;
          LogToSD("Update interval changed, " + String(dataUpdate_Interval) + " minute(s)");
          MenuTransition(SETTINGS_INTERVAL_STARTCHOICE);
          break;

        case SETTINGS_INTERVAL_STARTCHOICE:
          switch (updateTimeSel) {
            case CURRENT:
              timerAlarmDisable(timer_DataUpdate);
              startUpdateNow = true;
              LogToSD("Data update started");
              MenuTransition(CHANGES_DONE);
              break;

            case SCHEDULE:
              timerAlarmDisable(timer_DataUpdate);
              newHour = newMinute = newSecond = 0;
              hourSet = minuteSet = secondSet = false;
              MenuTransition(SETTINGS_INTERVAL_NEWSTART);
              break;
          }
          break;

        case SETTINGS_INTERVAL_NEWSTART:
          if (!hourSet) hourSet = !hourSet;
          else if (!minuteSet) minuteSet = !minuteSet;
          else if (!secondSet) secondSet = !secondSet;

          if (hourSet && minuteSet && secondSet) {
            waitingForNewUpdateTime = true;
            LogToSD("Waiting for next update time, " + String(nextScheduledUpdate));
            MenuTransition(CHANGES_DONE);
          } else MenuTransition(SETTINGS_INTERVAL_NEWSTART);
          break;

        case SETTINGS_CARD:
          if (isSDAvailable) MenuTransition(SETTINGS_CARD_EJECT);
          else MenuTransition(SETTINGS_CARD_INSERT);
          break;

        case SETTINGS_CARD_INSERT:
          SDInit();
          MenuTransition(CHANGES_DONE);
          break;

        case SETTINGS_CARD_EJECT:
          MenuTransition(SETTINGS_CARD_EJECT_CONFIRM);
          break;

        case SETTINGS_CARD_EJECT_CONFIRM:
          dataFile.close();
          LogToSD("SD card ejected");
          SD.end();
          isSDAvailable = false;
          MenuTransition(CHANGES_DONE);
          break;
      }
    }
  }

  if (menu.currentState == MAIN) {
    if (encoder.currentState == LOW) {
      if (millis() - encoder.millis.pressed >= inputHold_Settings) {
        encoder.millis.pressed = millis();
        encoder.isHolding = true;
        MenuTransition(SETTINGS_INTERVAL);
      }
    }
  }

  encoder.previouState = encoder.currentState;
}

void ReceivePBInputs() {
  button.currentState = digitalRead(button.pin);

  if (button.currentState == LOW && button.previouState == HIGH) {
    if (millis() - button.millis.released >= debounce) {
      button.millis.pressed = millis();
      button.isHolding = false;
    }
  }

  if (button.currentState == HIGH && button.previouState == LOW) {
    if (millis() - button.millis.pressed >= debounce && !button.isHolding) {
      button.millis.released = millis();
      switch (menu.currentState) {
        case MAIN:
        case SENSOR_A:
        case SENSOR_B:
        case SLEEP:
          break;

        default:
          MenuTransition(MAIN);
          break;
      }
    }
  }

  if (button.currentState == LOW) {
    if (millis() - button.millis.pressed >= inputHold_ForceRestart) {
      button.millis.pressed = millis();
      button.isHolding = true;
      dataFile.close();
      logFile.close();
      ESP.restart();
    }
  }

  button.previouState = button.currentState;
}

#pragma region Interrupts
void IRAM_ATTR EncoderUpdate() {
  encoder.currentRotaryState = digitalRead(encoder.clk);

  if (encoder.currentRotaryState != encoder.previousRotaryState && encoder.currentRotaryState == 1) {
    switch (menu.currentState) {
      case SETTINGS_INTERVAL:
        MenuTransition(SETTINGS_CARD);
        break;

      case SETTINGS_CARD:
        MenuTransition(SETTINGS_INTERVAL);
        break;

      case SETTINGS_INTERVAL_NEWINTERVAL:
        if (digitalRead(encoder.dt) != encoder.currentState) {
          dataUpdate_Interval++;
          if (dataUpdate_Interval > 60) dataUpdate_Interval = 1;
        } else {
          dataUpdate_Interval--;
          if (dataUpdate_Interval < 1) dataUpdate_Interval = 60;
        }
        MenuTransition(SETTINGS_INTERVAL_NEWINTERVAL);
        break;

      case SETTINGS_INTERVAL_STARTCHOICE:
        if (updateTimeSel == CURRENT) {
          updateTimeSel = SCHEDULE;
        } else if (updateTimeSel == SCHEDULE) {
          updateTimeSel = CURRENT;
        }
        MenuTransition(SETTINGS_INTERVAL_STARTCHOICE);
        break;

      case SETTINGS_INTERVAL_NEWSTART:
        if (digitalRead(encoder.dt) != encoder.currentState) {
          if (!hourSet) {
            newHour++;
            if (newHour > 23) newHour = 0;
          } else if (!minuteSet) {
            newMinute++;
            if (newMinute > 59) newMinute = 0;
          } else if (!secondSet) {
            newSecond++;
            if (newSecond > 59) newSecond = 0;
          }
        } else {
          if (!hourSet) {
            newHour--;
            if (newHour < 0) newHour = 23;
          } else if (!minuteSet) {
            newMinute--;
            if (newMinute < 0) newMinute = 59;
          } else if (!secondSet) {
            newSecond--;
            if (newSecond < 0) newSecond = 59;
          }
        }
        MenuTransition(SETTINGS_INTERVAL_NEWSTART);
        break;
    }
  }

  encoder.previousRotaryState = encoder.currentRotaryState;
}

void IRAM_ATTR SecondsUpdate() {
  switch (menu.currentState) {
    case MAIN:
    case SLEEP:
      updateMenu = true;
      break;
  }
}

void IRAM_ATTR DataUpdate() {
  minutesPassed++;

  if (minutesPassed == dataUpdate_Interval) {
    minutesPassed = 0;
    scheduledUpdate = true;
  }
}

void IRAM_ATTR InactivityTimeout() {
  switch (menu.currentState) {
    case MAIN:
      MenuTransition(SLEEP);
      break;

    case SENSOR_A:
      MenuTransition(SENSOR_B);
      break;

    case SENSOR_B:
      MenuTransition(MAIN);
      break;

    case CHANGES_DONE:
      MenuTransition(MAIN);
      break;
  }
}
#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();

  LCDInit();
  SensorInit();
  WifiInit();
  OTASetup();
  SetupInternetTime();
  SDInit();

  pinMode(encoder.clk, INPUT);
  pinMode(encoder.dt, INPUT);
  pinMode(encoder.sw, INPUT_PULLUP);
  pinMode(button.pin, INPUT_PULLUP);

  pinMode(LED_ERROR, OUTPUT);

  delay(1000);

  if (sensorChecks != 0 || wifiChecks != 0 || sdChecks != 0) {
    digitalWrite(LED_ERROR, HIGH);
    while (digitalRead(encoder.sw))
      ;
    digitalWrite(LED_ERROR, LOW);
    ESP.restart();
  }

  attachInterrupt(encoder.clk, EncoderUpdate, CHANGE);
  attachInterrupt(encoder.dt, EncoderUpdate, CHANGE);

  timer_Seconds = timerBegin(0, 80, true);
  timerAttachInterrupt(timer_Seconds, &SecondsUpdate, true);
  timerAlarmWrite(timer_Seconds, 1000 * 1000, true);
  timerAlarmEnable(timer_Seconds);

  timer_DataUpdate = timerBegin(1, 80, true);
  timerAttachInterrupt(timer_DataUpdate, &DataUpdate, true);
  timerAlarmWrite(timer_DataUpdate, 1000 * minuteDataUpdate_Interval, true);
  timerAlarmEnable(timer_DataUpdate);

  timer_Timeout = timerBegin(2, 80, true);
  timerAttachInterrupt(timer_Timeout, &InactivityTimeout, true);

  preferences.begin("settings", false);
  dataUpdate_Interval = preferences.getUInt("updateInterval", 1);

  systemInitialized = true;

  Serial.println("Device ready!");
  LogToSD("System initialized");
  MenuTransition(MAIN);
}

void loop() {
  ArduinoOTA.handle();

  ReceiveEncoderSWInputs();
  ReceivePBInputs();

  if (startUpdateNow) {
    startUpdateNow = !startUpdateNow;
    timerWrite(timer_DataUpdate, 0);
    timerAlarmEnable(timer_DataUpdate);
    GetSensorValue();
    DataToFavoriot();
    if (isSDAvailable) {
      DataToSD();
    }
  }

  char time[9];
  strftime(time, sizeof(time), "%T", &timeinfo);

  if (waitingForNewUpdateTime) {
    if (strcmp(time, nextScheduledUpdate.c_str()) == 0) {
      waitingForNewUpdateTime = !waitingForNewUpdateTime;
      timerWrite(timer_DataUpdate, 0);
      timerAlarmEnable(timer_DataUpdate);
      GetSensorValue();
      DataToFavoriot();
      if (isSDAvailable) {
        DataToSD();
      }
    }
  }

  if (scheduledUpdate) {
    scheduledUpdate = !scheduledUpdate;
    GetSensorValue();
    DataToFavoriot();
    if (isSDAvailable) {
      DataToSD();
    }
  }

  if (updateMenu) {
    MenuPrint();
    updateMenu = !updateMenu;
  }

  (menu.currentState == SLEEP) ? lcd.noBacklight() : lcd.backlight();
}
