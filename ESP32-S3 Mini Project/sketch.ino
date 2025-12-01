#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// =================== PIN DEFINITIONS (AMAN UNTUK ESP32-S3) ====================
#define LED_PIN        2
#define BUZZER_PIN     10  
#define BUTTON_OPEN    35 
#define BUTTON_CLOSE   14
#define POT_INPUT      5   
#define OLED_SDA       38
#define OLED_SCL       39
#define SERVO_PIN      18

// =================== HARDWARE OBJECTS ====================
Adafruit_SSD1306 screen(128, 64, &Wire, -1);
Servo myservo;

// =================== RTOS DATA STRUCTURES ====================

// 1. Pesan Perintah (Core 0 -> Core 1)
struct CommandMsg {
  int targetAngle;
  int velocity; // Delay (ms) -> Diatur oleh Potensio
};

// 2. Status Sistem (Core 1 -> Core 0)
struct SystemState {
  int currentRealAngle;
  bool isMoving;
};

// Variabel Global (Mulai di 90 derajat / TERTUTUP)
SystemState globalState = {90, false};

// =================== RTOS HANDLES ====================
QueueHandle_t commandQueue;
SemaphoreHandle_t stateMutex;

// =================== TASK CORE 1: ACTUATOR (MOTOR) ====================
void taskActuator(void *pv) {
  myservo.attach(SERVO_PIN);
  CommandMsg cmd;
  
  // Inisialisasi posisi awal 90 derajat (Tertutup)
  int currentPos = 90; 
  myservo.write(currentPos);

  while (1) {
    // 1. Tunggu perintah dari Queue
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      
      Serial.printf("[CORE 1] Target: %d | Speed Delay: %d ms\n", cmd.targetAngle, cmd.velocity);

      int step = (cmd.targetAngle > currentPos) ? 1 : -1;

      // [START] Set Status MOVING = TRUE
      if (xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        globalState.isMoving = true;
        xSemaphoreGive(stateMutex);
      }

      // Loop Gerakan
      while (currentPos != cmd.targetAngle) {
        currentPos += step;
        myservo.write(currentPos);

        // Update Posisi Real-time
        if (xSemaphoreTake(stateMutex, (TickType_t)5) == pdTRUE) {
          globalState.currentRealAngle = currentPos;
          xSemaphoreGive(stateMutex);
        }

        // DELAY DINAMIS (Kecepatan)
        vTaskDelay(cmd.velocity / portTICK_PERIOD_MS);
      }

      // [STOP] Set Status MOVING = FALSE (Selesai)
      if (xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
        globalState.isMoving = false;
        globalState.currentRealAngle = currentPos;
        xSemaphoreGive(stateMutex);
      }
      
      Serial.println("[CORE 1] Movement Done.");
    }
  }
}

// =================== TASK CORE 0: UI & INPUT ====================
void taskControllerUI(void *pv) {
  pinMode(BUTTON_OPEN, INPUT_PULLUP);
  pinMode(BUTTON_CLOSE, INPUT_PULLUP);
  pinMode(POT_INPUT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Init OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!screen.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Failed");
    vTaskDelete(NULL);
  }

  CommandMsg newCmd;
  SystemState localStateCopy;
  
  int lastSentAngle = -1;
  int currentSpeedDelay = 15; // Default Speed

  while (1) {
    bool triggerSend = false;
    int targetAngle = -1;

    // --- 1. LOGIKA POTENSIOMETER (Mengatur KECEPATAN) ---
    int potRaw = analogRead(POT_INPUT);
    // Pot 0 -> Lambat (60ms), Pot Max -> Cepat (2ms)
    currentSpeedDelay = map(potRaw, 0, 4095, 60, 2); 

    // --- 2. LOGIKA TOMBOL (90 Tertutup, 45 Terbuka) ---
    if (digitalRead(BUTTON_OPEN) == LOW) {
      targetAngle = 45; // UBAH DISINI: Terbuka = 45
      triggerSend = true;
    } 
    else if (digitalRead(BUTTON_CLOSE) == LOW) {
      targetAngle = 90; // UBAH DISINI: Tertutup = 90
      triggerSend = true;
    }

    // --- 3. KIRIM QUEUE ---
    if (triggerSend) {
      newCmd.targetAngle = targetAngle;
      newCmd.velocity = currentSpeedDelay; 
      
      if (xQueueSend(commandQueue, &newCmd, 0) == pdTRUE) {
        lastSentAngle = targetAngle;
        vTaskDelay(200 / portTICK_PERIOD_MS); // Debounce tombol
      }
    }

    // --- 4. BACA STATUS MUTEX (Update Data Lokal) ---
    bool stateValid = false;
    if (xSemaphoreTake(stateMutex, (TickType_t)10) == pdTRUE) {
      localStateCopy = globalState; 
      xSemaphoreGive(stateMutex);   
      stateValid = true;
    }

    // --- 5. INDIKATOR LED & BUZZER (Berbasis Status Moving) ---
    if (stateValid) {
      if (localStateCopy.isMoving) {
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
      }
    }

    // --- 6. UPDATE OLED ---
    if (stateValid) {
      screen.clearDisplay();
      screen.setTextColor(SSD1306_WHITE);
      screen.setTextSize(1);

      screen.setCursor(0, 0);
      screen.println("MODE: SPEED CONTROL");

      int speedPercent = map(currentSpeedDelay, 60, 2, 0, 100);
      screen.setCursor(0, 15);
      screen.printf("Speed Set: %d %%", speedPercent);

      screen.setCursor(0, 30);
      screen.printf("Position : %d deg", localStateCopy.currentRealAngle);

      screen.setCursor(0, 45);
      if (localStateCopy.isMoving) {
        screen.println("Status   : MOVING >>");
      } else {
        // UBAH DISINI: Cek Status Terbalik (90=Tutup, 45=Buka)
        if (localStateCopy.currentRealAngle >= 88) screen.println("Status   : CLOSED");
        else if (localStateCopy.currentRealAngle <= 47) screen.println("Status   : OPEN");
        else screen.println("Status   : IDLE");
      }

      // Visual Bar (Mapping Terbalik: 90->0 pixel, 45->126 pixel)
      screen.drawRect(0, 56, 128, 8, SSD1306_WHITE);
      int w = map(localStateCopy.currentRealAngle, 90, 45, 0, 126);
      
      // Pastikan w tidak negatif atau melebihi batas (Constraint)
      if (w < 0) w = 0;
      if (w > 126) w = 126;
      
      screen.fillRect(2, 58, w, 4, SSD1306_WHITE);

      screen.display();
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// =================== SETUP ====================
void setup() {
  Serial.begin(115200);
  
  // RTOS Init
  commandQueue = xQueueCreate(10, sizeof(CommandMsg));
  stateMutex = xSemaphoreCreateMutex();

  if (commandQueue == NULL || stateMutex == NULL) {
    while(1);
  }

  // Tasks
  xTaskCreatePinnedToCore(taskControllerUI, "UI", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskActuator, "MOTOR", 4096, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}