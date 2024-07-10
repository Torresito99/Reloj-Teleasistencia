#include <driver/i2s.h>
#include <math.h>
#include "config.h"
#include <TTGO.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "RTClib.h"

TTGOClass *ttgo;
TFT_eSPI *tft;
BMA *sensor;
RTC_PCF8563 rtc;

const char* ssid = "";
const char* password = "";
const char* homeAssistantUrl = "http://homeassistant.local:8123/api/webhook/caida_detectada";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "time.google.com", 7200, 60000);

bool isRunning = false;
bool fallDetected = false;
bool notificationSent = false;
unsigned long fallTime = 0;
const int sampleRate = 16000;
const int frequency = 1000;
unsigned long lastMotionTime = 0;
bool timeUpdated = false;
char currentTime[6] = "";
int currentBatteryLevel = -1;
bool screenOn = true;
unsigned long lastAccelCheckTime = 0;
const unsigned long accelCheckInterval = 100;
const unsigned long motionInactiveInterval = 10000;

lv_obj_t *btn1;

TaskHandle_t toneTaskHandle = NULL; // Manejador de la tarea del tono

static void event_handler(lv_obj_t *obj, lv_event_t event) {
    if (event == LV_EVENT_CLICKED) {
        resetFallDetection();
        if (btn1) {
            lv_obj_del(btn1);
            btn1 = NULL;
        }
        clearScreen();
        refreshScreen();
        Serial.println("Alarma detenida por el botón");
    }
}

void setup() {
    Serial.begin(115200);
    ttgo = TTGOClass::getWatch();
    ttgo->begin();
    ttgo->openBL();
    ttgo->setBrightness(30);
    tft = ttgo->tft;
    sensor = ttgo->bma;

    if (!rtc.begin()) {
        Serial.println("No se puede encontrar RTC");
        while (1);
    }
    if (rtc.lostPower()) {
        Serial.println("RTC no está inicializado, estableciendo la hora a las 00:00:00 del 1 de enero de 2023");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    tft->setTextColor(TFT_WHITE);
    tft->setTextFont(4);
    tft->fillScreen(TFT_BLACK);

    connectToWiFi();
    timeClient.begin();

    if (!sensor->begin()) {
        Serial.println("Error al inicializar el sensor BMA.");
        while (1);
    } else {
        Serial.println("Sensor BMA inicializado correctamente.");
    }

    Acfg cfg;
    cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    cfg.range = BMA4_ACCEL_RANGE_4G;
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    cfg.perf_mode = BMA4_CONTINUOUS_MODE;
    sensor->accelConfig(cfg);
    sensor->enableAccel();
    Serial.println("Sensor BMA configurado correctamente.");

    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 33,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    displayAll();

    Serial.println("Setup completo");
}

void loop() {
    lv_task_handler();
    delay(5);

    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    DateTime now = DateTime(epochTime);

    if (abs((int64_t)(now.unixtime() - rtc.now().unixtime())) > 10) {
        rtc.adjust(now);
        Serial.println("Hora ajustada desde el servidor NTP");
    }

    unsigned long currentTimeMillis = millis();
    if (currentTimeMillis - lastAccelCheckTime >= accelCheckInterval) {
        lastAccelCheckTime = currentTimeMillis;
        checkMotion();
        checkFall();
    }

    if (!fallDetected) {
        if (screenOn) {
            if (now.hour() == 0 && now.minute() == 0 && !timeUpdated) {
                displayDate();
                displayDayOfWeek();
                timeUpdated = true;
            }

            if (now.hour() == 0 && now.minute() == 1 && now.second() == 0) {
                timeUpdated = false;
            }

            char newTime[6];
            sprintf(newTime, "%02d:%02d", now.hour(), now.minute());
            if (strcmp(newTime, currentTime) != 0) {
                strcpy(currentTime, newTime);
                displayTime();
            }

            int newBatteryLevel = ttgo->power->getBattPercentage();
            if (newBatteryLevel != currentBatteryLevel && newBatteryLevel >= 0 && newBatteryLevel <= 100) {
                currentBatteryLevel = newBatteryLevel;
                displayBatteryLevel();
            }
        }
    } else {
        if (!btn1) {
            showAlarmButton();
        }

        if (!notificationSent && (millis() - fallTime > 10000)) { // Verificar si han pasado 10 segundos desde la detección de la caída
            sendFallNotification();
            notificationSent = true;
        }
    }
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando a WiFi...");
    }
    Serial.println("Conectado a WiFi");
}

void refreshScreen() {
    if (!screenOn) {
        ttgo->openBL();
        screenOn = true;
        displayAll();
        Serial.println("Pantalla encendida y refrescada.");
    } else {
        ttgo->closeBL();
        screenOn = false;
        Serial.println("Pantalla apagada.");
    }
}

void generateToneTask(void *parameter) {
    int freq = *(int*)parameter;
    int samples = sampleRate / freq;
    int16_t sampleData[samples];

    for (int i = 0; i < samples; i++) {
        sampleData[i] = 100000 * sin(2 * PI * i / samples);
    }

    size_t bytes_written;
    while (true) {
        i2s_write(I2S_NUM_0, sampleData, sizeof(sampleData), &bytes_written, portMAX_DELAY);
        vTaskDelay(1); // Agregar un pequeño retraso para evitar sobrecargar el I2S
    }
}

void startTone(int freq) {
    if (toneTaskHandle == NULL) {
        xTaskCreate(generateToneTask, "Tone Task", 2048, &freq, 1, &toneTaskHandle);
        Serial.println("Tone task started.");
    }
}

void stopTone() {
    if (toneTaskHandle != NULL) {
        vTaskDelete(toneTaskHandle);
        toneTaskHandle = NULL;
        Serial.println("Tone task stopped.");
    }
}

void checkMotion() {
    static int sameValueCount = 0;
    static bool outOfRangeDetected = false;

    Accel acc;
    if (sensor->getAccel(acc)) {
        float totalAcceleration = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) / 1000.0;
        Serial.print("Total Acceleration: ");
        Serial.println(totalAcceleration);

        if (totalAcceleration >= 0.45 && totalAcceleration <= 0.55) {
            sameValueCount++;
        } else {
            sameValueCount = 0;
            outOfRangeDetected = true;
            if (!screenOn) {
                ttgo->openBL();
                screenOn = true;
                displayAll();
                Serial.println("Pantalla encendida debido a movimiento detectado.");
            }
        }

        if (sameValueCount >= motionInactiveInterval / accelCheckInterval) {
            if (screenOn) {
                ttgo->closeBL();
                screenOn = false;
                Serial.println("Pantalla apagada debido a inactividad.");
            }
            sameValueCount = 0;
        }

        if (outOfRangeDetected && screenOn) {
            outOfRangeDetected = false;
        }
    } else {
        Serial.println("No se pudo obtener la aceleración del sensor.");
    }
}

void checkFall() {
    static bool inFreeFall = false;
    static unsigned long fallStartTime = 0;

    Accel acc;
    if (sensor->getAccel(acc)) {
        float totalAcceleration = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z) / 1000.0;

        if (totalAcceleration > 2) {
            if (!inFreeFall) {
                inFreeFall = true;
                fallStartTime = millis();
            } else if (millis() - fallStartTime > 100) {
                fallDetected = true;
                fallTime = millis();
                notificationSent = false; // Reiniciar la variable de notificación

                clearScreen();
                showAlarmButton();
                startTone(frequency); // Inicia el tono

                Serial.println("¡Caída detectada!");
            }
        } else {
            inFreeFall = false;
        }
    } else {
        Serial.println("No se pudo obtener la aceleración del sensor.");
    }
}

void showAlarmButton() {
    static bool lvglInitialized = false;
    if (!lvglInitialized) {
        ttgo->lvgl_begin();
        lvglInitialized = true;
    }

    clearScreen();

    if (btn1) {
        lv_obj_del(btn1);
    }

    btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn1, event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);
    lv_obj_t *label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Detener Alarma");
}

void sendFallNotification() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(homeAssistantUrl);
        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.POST("{}");
        if (httpResponseCode > 0) {
            Serial.printf("Código de respuesta HTTP: %d\n", httpResponseCode);
        } else {
            Serial.printf("Código de error: %s\n", http.errorToString(httpResponseCode).c_str());
        }
        http.end();
    }
}

void displayDate() {
    DateTime now = rtc.now();
    char buffer[11];
    sprintf(buffer, "%02d/%02d/%04d", now.day(), now.month(), now.year());
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextDatum(TL_DATUM);
    tft->setTextFont(4);
    tft->drawString(buffer, 5, 10, 4);
    Serial.println(buffer);
}

void displayDayOfWeek() {
    DateTime now = rtc.now();
    const char* daysOfWeek[] = {"Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado"};
    char buffer[10];
    sprintf(buffer, "%s", daysOfWeek[now.dayOfTheWeek()]);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(4);
    tft->drawString(buffer, tft->width() / 2, tft->height() - 40, 4);
    Serial.println(buffer);
}

void displayTime() {
    DateTime now = rtc.now();
    char buffer[6];
    sprintf(buffer, "%02d:%02d", now.hour(), now.minute());
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setTextDatum(MC_DATUM);
    tft->setTextFont(7);
    tft->drawString(buffer, tft->width() / 2, tft->height() / 2 - 20, 7);
    Serial.println(buffer);
}

void displayBatteryLevel() {
    int batteryLevel = ttgo->power->getBattPercentage();

    uint32_t fillColor;
    uint32_t borderColor = TFT_WHITE;

    if (ttgo->power->isChargeing()) {
        fillColor = TFT_ORANGE;
    } else {
        fillColor = TFT_GREEN;
    }

    int x = tft->width() - 45;
    int y = 10;
    int w = 40;
    int h = 20;
    int r = 5;

    tft->drawRoundRect(x, y, w, h, r, borderColor);

    int fillWidth = (w - 4) * batteryLevel / 100;
    tft->fillRoundRect(x + 2, y + 2, fillWidth, h - 4, r, fillColor);

    tft->setTextColor(TFT_BLACK, TFT_BLACK);
    tft->setTextDatum(TR_DATUM);
    tft->drawString("100%", x - 3, y + 2, 2);

    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->drawString(String(batteryLevel) + "%", x - 3, y + 2, 2);
    Serial.printf("Nivel de batería: %d%%\n", batteryLevel);
}

void displayAll() {
    clearScreen();
    displayDate();
    displayDayOfWeek();
    displayTime();
    displayBatteryLevel();
}

void clearScreen() {
    tft->fillScreen(TFT_BLACK);
    delay(100);
}

void resetFallDetection() {
    fallDetected = false;
    isRunning = false;
    notificationSent = true; // Asegura que la notificación no se envíe después de pulsar el botón
    stopTone(); // Detiene el tono
    Serial.println("Reset fall detection state.");
}
