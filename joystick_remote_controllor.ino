#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

/*********************** TFT Display setup ***********************************************************************************************/
#define TFT_CS 5
#define TFT_DC 2
#define TFT_RST 4

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
unsigned long currentMillis = 0;
unsigned long currentSendTimeMicros = 0;
unsigned long currentReceiveTimeMicros = 0;
unsigned long lastSendTime = 0;  // for transmitter
unsigned long lastReceiveTime = 0;
unsigned long lastTxSendTime = 0;  // transmitter frequency

int Txstate = 0;
int TxRxState = 0;
int RxState = 0;

bool showTriangle = false;

unsigned long lastBlinkTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastReadMillis = 0;

const unsigned long intervalMillis = 50;

int signalLevel = 0;
int batteryPercentage = 100;
bool is_ED_shown = false;

/**************************** NRF module setup ***************************************************************************************************/
#define CE_PIN 16
#define CSN_PIN 17
RF24 radio(CE_PIN, CSN_PIN);  // radio object creation

// channel addresses setup
const byte CH_3[6] = "10001";

uint8_t commandNO = 0;
uint8_t FCdebugCode = 0;
uint8_t FCerrorCode = 0;

static bool debug_check_array[35] = { 0 };
static bool error_check_array[35] = { 0 };

int packetLossCounter = 0;
int packetSentCounter = 0;

bool is_takeoff = 0;
bool is_landed = 1;

float altitude = 0.0f;


/************************** Pots, Joysticks, switches setup **********************************************************************************/
// Switch pins
const int switchPins[] = { 21, 22, 25, 26 };

// Potentiometer pins
const int potPins[] = { 14, 39 };

// Joystick pins
const int joy1XPin = 32;
const int joy1YPin = 33;

const int joy2XPin = 34;
const int joy2YPin = 35;

// Filtered values
float filteredPots[2] = { 0, 0 };
float filteredJoy[4] = { 0, 0, 0, 0 };

//Variables for storing mapped controller values
float mappedJoy1X = 0;
float mappedJoy1Y = 0;
float mappedJoy2X = 0;
float mappedJoy2Y = 0;
float mappedPot1 = 0;
float mappedPot2 = 0;

bool button_1 = 0;  // Arm/Disarm
bool button_2 = 0;  // Manual/Autonomous
bool button_3 = 0;  // push button
bool button_4 = 0;  // push button

//Telemetery values
float height = 0;
float speed = 0;

float pitch = 0;
float roll = 0;
float yaw = 0;

float maxHeight = 0;
float flightTime = 0;

uint8_t RSSIarray[15] = { 0 };

// LPF alpha
const float alpha = 0.1;

/* Display's functions >=======================================================================================================================================*/

//Drawing signal bars
void drawSignalBars(int level, int x = 5, int y = 7) {
  int barWidth = 3;
  int spacing = 3;
  int maxBars = 5;
  int maxHeight = 15;

  for (int i = 0; i < maxBars; i++) {
    int height = (i + 1) * (maxHeight / maxBars);
    int xpos = x + i * (barWidth + spacing);
    int ypos = y + maxHeight - height;
    uint16_t color = (i < level) ? ILI9341_GREEN : ILI9341_DARKGREY;
    tft.fillRect(xpos, ypos, barWidth, height, color);
  }
}

//Drawing Battery symbol
void drawBatterySymbol(int level, int x = 280, int y = 7) {
  int width = 30;
  int height = 15;
  int tipWidth = 4;
  int tipHeight = 6;
  int margin = 2;

  tft.drawRect(x, y, width, height, ILI9341_WHITE);
  tft.fillRect(x + width, y + (height - tipHeight) / 2, tipWidth, tipHeight, ILI9341_WHITE);

  int fillWidth = map(level, 0, 100, 0, width - 2 * margin);
  uint16_t fillColor = (level > 50) ? ILI9341_GREEN : (level > 20) ? ILI9341_YELLOW
                                                                   : ILI9341_RED;
  tft.fillRect(x + margin, y + margin, fillWidth, height - 2 * margin, fillColor);
}

//Shows warnings
void drawWarningSymbol(bool show, int x = 150, int y = 1) {
  if (show) {
    // Draw yellow triangle with exclamation mark
    tft.fillTriangle(x, y, x - 10, y + 20, x + 10, y + 20, ILI9341_YELLOW);
    tft.setTextColor(ILI9341_BLACK);
    //Exclemanatin mark
    tft.fillRect(x - 1, y + 5, 3, 10, ILI9341_BLACK);
    tft.fillRect(x - 1, 18, 3, 2, ILI9341_BLACK);
  } else {
    // Clear the area
    tft.fillRect(x - 11, y, 22, 22, ILI9341_BLACK);
  }
}

//Blink warning Triangle
void showWarning() {
  if (currentMillis - lastBlinkTime >= 500) {
    showTriangle = !showTriangle;
    drawWarningSymbol(showTriangle);
    lastBlinkTime = currentMillis;
  }
}


void display_blinking_text(char* text, int duration) {

  static unsigned long start_time = 0;
  static bool is_active = false;
  static bool fading = false;

  unsigned long current_time = millis();

  // Show text initially
  if (!is_active) {
    tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);

    tft.setCursor(75, 90);
    tft.setTextSize(2);  // FIXED SIZE
    tft.setTextColor(ILI9341_GREEN);
    tft.print(text);

    start_time = current_time;
    is_active = true;
    fading = false;
  }

  // Start fading after duration
  if (is_active && !fading && (current_time - start_time >= duration)) {
    fading = true;
    start_time = current_time;
  }

  // Fade logic
  if (fading) {

    const int fade_steps = 30;
    const int step_time = 10;

    int step = (current_time - start_time) / step_time;

    if (step < fade_steps) {

      uint8_t intensity = 200 - (step * (100 / fade_steps));
      uint16_t color = tft.color565(intensity, 100, intensity - 100);

      // Clear previous frame
      tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);

      // Draw with SAME size + SAME position
      tft.setCursor(75, 90);
      tft.setTextSize(2);  // locked size
      tft.setTextColor(color);
      tft.print(text);

    } else {
      // Fully clear
      tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);
      is_active = false;
      fading = false;
    }
  }
}

void display_plain_text(char* text) {

  tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);

  tft.setCursor(5, 60);
  tft.setTextSize(2);  // FIXED SIZE
  tft.setTextColor(ILI9341_MAGENTA);
  tft.print(text);
}


void display_fading_text(char* text, int duration) {

  static unsigned long start_time = 0;
  static bool is_active = false;
  static bool fading = false;

  unsigned long current_time = millis();

  // Show text initially
  if (!is_active) {
    tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);

    tft.setCursor(5, 60);
    tft.setTextSize(2);  // FIXED SIZE
    tft.setTextColor(ILI9341_MAGENTA);
    tft.print(text);

    start_time = current_time;
    is_active = true;
    fading = false;
  }

  // Start fading after duration
  if (is_active && !fading && (current_time - start_time >= duration)) {
    fading = true;
    start_time = current_time;
  }

  // Fade logic
  if (fading) {

    const int fade_steps = 20;
    const int step_time = 20;

    int step = (current_time - start_time) / step_time;

    if (step < fade_steps) {

      uint8_t intensity = 255 - (step * (255 / fade_steps));
      uint16_t color = tft.color565(intensity, 0, intensity);

      // Clear previous frame
      tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);

      // Draw with SAME size + SAME position
      tft.setCursor(5, 60);
      tft.setTextSize(2);  //  locked size
      tft.setTextColor(color);
      tft.print(text);

    } else {
      // Fully clear
      tft.fillRect(0, 58, 320, 18, ILI9341_BLACK);
      is_active = false;
      fading = false;
    }
  }
}

// Show flight data
void drawTelemetryData(float altitude, float speed, float pitch, float roll, float yaw, float maxHeight, float flightTime, int x = 5, int y = 140) {

  // tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK); // White text on black background
  // tft.setTextSize(2);
  // tft.setCursor(x, y-25);
  // tft.print("Flight Data");

  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);  // White text on black background
  tft.setTextSize(1);
  int lineHeight = 12;

  tft.setCursor(x, y);
  tft.print("Altitude: ");
  tft.print(altitude);
  tft.println(" m");

  tft.setCursor(x, y + lineHeight * 1);
  tft.print("Speed:    ");
  tft.print(speed);
  tft.println(" m/s");

  tft.setCursor(x, y + lineHeight * 2);
  tft.print("Pitch:    ");
  tft.print(pitch);
  tft.println(" deg");

  tft.setCursor(x, y + lineHeight * 3);
  tft.print("Roll:     ");
  tft.print(roll);
  tft.println(" deg");

  tft.setCursor(x, y + lineHeight * 4);
  tft.print("Yaw:      ");
  tft.print(yaw);
  tft.println(" deg");

  tft.setCursor(x, y + lineHeight * 5);
  tft.print("Height limit: ");
  tft.print(maxHeight);
  tft.println(" m");

  tft.setCursor(x, y + lineHeight * 6);
  tft.print("Flight Time:  ");
  tft.print(flightTime);
  tft.println(" min");
}

// Show controller data
void drawControllerOutput(
  int joy1_x, int joy1_y,
  int joy2_x, int joy2_y,
  int pot1, int pot2,
  int x = 240,
  int y = 140) {
  int textSize = 1;
  int lineSpacing = 10;

  for (int i = 0; i < 6; i++) {
    tft.fillRect(x + 30, y + i * lineSpacing, 120, lineSpacing, ILI9341_BLACK);
  }

  tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
  tft.setTextSize(textSize);

  tft.setCursor(x, y + 0 * lineSpacing);
  tft.print("J1 X: ");
  tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
  tft.print(joy1_x);

  tft.setCursor(x, y + 1 * lineSpacing);
  tft.print("J1 Y: ");
  tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
  tft.print(joy1_y);

  tft.setCursor(x, y + 2 * lineSpacing);
  tft.print("J2 X: ");
  tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
  tft.print(joy2_x);

  tft.setCursor(x, y + 3 * lineSpacing);
  tft.print("J2 Y: ");
  tft.print(joy2_y);

  tft.setCursor(x, y + 4 * lineSpacing);
  tft.print("Pot1: ");
  tft.print(pot1);

  tft.setCursor(x, y + 5 * lineSpacing);
  tft.print("Pot2: ");
  tft.print(pot2);
}

void process_ED_code() {

  static uint8_t error_code_array[] = { 100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
                                        110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
                                        120, 121, 122, 123, 124, 125, 126, 127, 128, 129 };


  static uint8_t debug_code_array[] = { 100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
                                        110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
                                        120, 121, 122, 123, 124, 125, 126, 127, 128, 129 };

  for (int i = 0; i <= 30; i++) {
    if (debug_code_array[i] == FCdebugCode) debug_check_array[i] = 1;
    if (error_code_array[i] == FCerrorCode) error_check_array[i] = 1;
  }
}

void show_ED_messages() {

  static char* char_debug_array[] = {
    "",                       //0
    "Hello Sir!",             //1
    "FC is Online!",          //2
    "ESC calibrated ",        //3
    "RF module Initialised",  //4
    "Disconnected!",          //5
    "MPU6500 Initialised",    //6
    "MPU6500 not found!",     //7
    "BMM150 Initialised",     //8
    "BMM150 not found!",      //9
    "BMP280 Initialised",     //10
    "BMP280 not found!",      //11
    "LiDAR Initialised",      //12
    "LiDAR not found!",       //13
    "GPS Initialised",        //14
    "GPS not found!",         //15
    "All clear",              //16
    "Ready for takeoff",      //17
    "Taking Off...",          //18
    "Landing...",             //19
    "Have a good flight!"     //20
  };

  static char* char_error_array[] = {
    " ",              //0
    "Signal lost!",   //1
    "Battery Low!",   //2
    "GPS lost!",      //3
    "MPU6500 lost!",  //4
    "BMM150 lost!",   //5
    "LiDAR lost!",    //6
    "BMP280 lost!"    //7
  };

  static unsigned long lastLoopTime = 0;
  static unsigned long errorPrintTime = 0;
  static bool debugHold = false;

  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  /* ---------- 1000 Hz loop ---------- */
  if (nowMicros - lastLoopTime < 2000) return;
  lastLoopTime = nowMicros;

  /* ---------- Error hold (1 second wait) ---------- */
  if (debugHold) {
    if (nowMillis - errorPrintTime >= 1000) {
      debugHold = false;  // release after 1 second
    }
    return;  // skip printing while waiting
  }

  /* ---------- Scan arrays ---------- */

  for (int i = 0; i < 25; i++) {

    if (error_check_array[i] == 1) {
      Serial.println(char_error_array[i]);

      error_check_array[i] = 0;
    }

    if (debug_check_array[i] == 1) {
      Serial.println(char_debug_array[i]);
      display_fading_text(char_debug_array[i], 500);
      debug_check_array[i] = 0;

      errorPrintTime = nowMillis;
      debugHold = true;  // block for 1 second
      break;             // only one error at a time
    }
  }
}

/*Reading controller values>===================================================================================================================================*/

// Map helper
int mapToRange(float value, int in_min, int in_max, int out_min, int out_max) {
  int mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // Constrain just in case
  if (mapped < out_min) mapped = out_min;
  if (mapped > out_max) mapped = out_max;
  return mapped;
}

void handle_push_buttons() {

  static unsigned long pressTime = 0;

  if (button_3 == 1 && button_4 == 1 && is_takeoff == 0) {

    if (pressTime == 0) {
      pressTime = millis();
    }

    if (millis() - pressTime >= 2000) {
      commandNO = 101;  // Takeoff
      is_takeoff = 1;
      pressTime = 0;  // reset to avoid retrigger
    }

  } else if (button_3 == 1 && button_4 == 1 && is_takeoff == 1) {

    if (pressTime == 0) {
      pressTime = millis();
    }

    if (millis() - pressTime >= 2000) {
      commandNO = 102;  // Landing
      is_takeoff = 0;
      pressTime = 0;  // reset to avoid retrigger
    }

  } else {
    pressTime = 0;  // buttons released
  }
}

void readInputs() {

  button_1 = digitalRead(switchPins[0]);
  button_2 = digitalRead(switchPins[1]);
  button_3 = digitalRead(switchPins[2]);
  button_4 = digitalRead(switchPins[3]);
  // Pots

  int rawPot1 = analogRead(potPins[0]);
  int rawPot2 = analogRead(potPins[1]);

  filteredPots[0] = alpha * rawPot1 + (1 - alpha) * filteredPots[0];
  filteredPots[1] = alpha * rawPot2 + (1 - alpha) * filteredPots[1];

  mappedPot1 = mapToRange(filteredPots[0], 0, 4095, 0, 99);
  mappedPot2 = mapToRange(filteredPots[1], 0, 4095, 0, 99);


  // Joysticks
  int joyRaw[4] = {
    analogRead(joy1XPin),
    analogRead(joy1YPin),
    analogRead(joy2XPin),
    analogRead(joy2YPin)
  };

  for (int i = 0; i < 4; i++) {
    filteredJoy[i] = alpha * joyRaw[i] + (1 - alpha) * filteredJoy[i];
  }

  mappedJoy1X = 999 - mapToRange(filteredJoy[1], 0, 4095, 0, 999);
  mappedJoy1Y = mapToRange(filteredJoy[0], 0, 4095, 0, 999);
  mappedJoy2X = mapToRange(filteredJoy[2], 0, 4095, 0, 999);
  mappedJoy2Y = 999 - mapToRange(filteredJoy[3], 0, 4095, 0, 999);
}


/*Radio transmission >====================================================================================================================*/

// Transmitter structs
#pragma pack(push, 1)
struct RC_data {
  uint16_t joy1X;
  uint16_t joy1Y;
  uint16_t joy2X;
  uint16_t joy2Y;
  uint8_t toggle1;
  uint8_t toggle2;
  uint8_t cmd;
};

#pragma pack(pop)
struct ackPayload {
  uint8_t battery;
  uint8_t error_code;
  uint8_t debug_code;
  float alt;
};

void sendControllerData() {
  if (button_1 == 1) {
    RC_data data = { mappedJoy1X, mappedJoy1Y, mappedJoy2X,
                     mappedJoy2Y, button_1, button_2, commandNO };
    if (radio.write(&data, sizeof(data))) {
      if (radio.isAckPayloadAvailable()) {

        if (packetSentCounter < 301) {
          packetSentCounter++;
          if (packetSentCounter >= 200) {
            packetLossCounter = 0;
          }
        }
        ackPayload ackData;

        radio.read(&ackData, sizeof(ackData));
        batteryPercentage = ackData.battery;
        FCerrorCode = ackData.error_code;
        FCdebugCode = ackData.debug_code;
        altitude = ackData.alt;

        if (batteryPercentage < 20) {
          //  Serial.println("Battery Low! ");
        }

        // Serial.print(" Battery percentage : ");
        // Serial.print(batteryPercentage);
        // Serial.print(" | Altitude : ");
        // Serial.print(altitude);
        // Serial.print(" | Debugging Code : ");
        // Serial.println(FCdebugCode);
        // Serial.print(" | Error Code : ");
        // Serial.println(FCerrorCode);

        // for(int i = 0 ; i<11 ; i++ ){
        //  messageCode[i] = ackData.errorCode[i] ;
        //  Serial.print(messageCode[i]);
        // }


      } else if (packetLossCounter < 205) {
        packetLossCounter++;
        if (packetLossCounter >= 100)
          packetSentCounter = 0;
        // Serial.println("ACK received, no payload");
      }
    }
  }
}

// Reciver Struct
struct rollPitchYaw {
  uint8_t rollDeg;
  uint8_t pitchDeg;
  uint8_t yawDeg;
};

struct batteryHeightSpeed {
  uint8_t batteryPercentage;
  uint8_t droneAltitude;
  uint8_t droneSpeed;
};

void reciveTelemetery1() {
  // radio.stopListening();
  // radio.setChannel(111);
  radio.openReadingPipe(1, CH_3);
  radio.startListening();
  if (radio.available()) {
    rollPitchYaw data;
    radio.read(&data, sizeof(data));
    Serial.print("Roll: ");
    Serial.print(data.rollDeg);
    Serial.print(" Pitch: ");
    Serial.print(data.pitchDeg);
    Serial.print(" Yaw: ");
    Serial.println(data.yawDeg);
  }
}

void reciveTelemetery2() {
  // radio.stopListening();
  // radio.setChannel(115);
  radio.openReadingPipe(1, CH_3);
  // radio.startListening();
  if (radio.available()) {
    batteryHeightSpeed data;
    radio.read(&data, sizeof(data));
    Serial.print("Battery: ");
    Serial.print(data.batteryPercentage);
    Serial.print(" Altitude: ");
    Serial.print(data.droneAltitude);
    Serial.print(" Speed: ");
    Serial.println(data.droneSpeed);
  }
}

void reciveRSSIarray() {

  // radio.stopListening();
  // radio.setChannel(120);
  radio.openReadingPipe(1, CH_3);  // same pipe, but different channel
  // radio.startListening();
  if (radio.available()) {
    radio.read(&RSSIarray, sizeof(RSSIarray));
    Serial.print("RSSI: ");
    for (int i = 0; i < sizeof(RSSIarray); i++) {
      Serial.print(RSSIarray[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

bool receiveData() {
  currentReceiveTimeMicros = micros();

  if (RxState == 0 && currentReceiveTimeMicros - lastReceiveTime >= 500) {
    reciveTelemetery1();
    RxState = 1;
  } else if (RxState == 1 && currentReceiveTimeMicros - lastReceiveTime >= 1000) {
    reciveTelemetery1();
    RxState = 2;
  } else if (RxState == 2 && currentReceiveTimeMicros - lastReceiveTime >= 1500) {
    reciveRSSIarray();
    RxState = 0;
    lastReceiveTime = currentReceiveTimeMicros;
    return true;
  }
  return false;
}

/*Creating RTOS Tasks >=============================================================================================================================*/

// Transmitting values
void TxTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    sendControllerData();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Reading values
void readInputTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    // non-blocking in itself
    readInputs();
    process_ED_code();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void showUI() {

  signalLevel = random(0, 5);

  // Clear and redraw signal bars
  tft.fillRect(0, 0, 60, 20, ILI9341_BLACK);
  drawSignalBars(signalLevel);
  // showWarning();
  // drawWarningSymbol();

  // Show flight modes
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(5, 35);
  if (button_1 == 0) {  // Toggle 1
    //tft.fillRect(5, 35, 70, 10, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.print("Disarmed");

  } else {
    //tft.fillRect(5, 35, 70, 10, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.print("Armed   ");
  }

  tft.setCursor(260, 35);
  if (button_2 == 0) {  // Toggle 2
    tft.setCursor(260, 35);
    // tft.fillRect(260, 35, 70, 10, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.print("   Manual ");
  } else {
    // tft.fillRect(260, 35, 70, 10, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.print("Autonomous");
  }

  // Clear battery area and redraw
  tft.fillRect(240, 0, 100, 20, ILI9341_BLACK);
  //tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setCursor(255, 10);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print(batteryPercentage);
  tft.print("%");
  drawBatterySymbol(batteryPercentage);

  char* abc = "Battery Low!";
  display_blinking_text(abc, 500);
  // char* afc = "We are ready for takeoff!";
  // display_plain_text(afc, 1000);

  drawTelemetryData(height, speed, pitch, roll, yaw, maxHeight, flightTime);
  drawControllerOutput(mappedJoy1X, mappedJoy1Y, mappedJoy2X, mappedJoy2Y, mappedPot1, mappedPot2);
}

// Show UI Task
void showUITask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    // non-blocking in itself
    showUI();
    show_ED_messages();

    vTaskDelay(pdMS_TO_TICKS(33));
  }
}


/*==================================================================================================================================================*/

void setup() {
  // Display
  Serial.begin(115200);
  tft.begin(40000000);
  tft.setRotation(1);
  debug_check_array[0] = 1;

  // tft.drawLine(0, 75, 320, 75, ILI9341_DARKGREY);
  // tft.drawLine(0, 58, 320, 58, ILI9341_DARKGREY);

  // Starting controller
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 60);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setTextSize(5);
  tft.print(" WELCOME");
  tft.setCursor(0, 115);
  tft.setTextSize(2);
  tft.print("             TO");
  tft.setCursor(0, 150);
  tft.setTextSize(5);
  tft.print("   SKYWAVE");
  delay(2000);
  tft.fillScreen(ILI9341_BLACK);
  tft.drawLine(0, 28, 320, 28, ILI9341_WHITE);

  // Configure switches as inputs with pull-up
  for (int i = 0; i < 4; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }

  // Initialize filtered values with first readings
  for (int i = 0; i < 2; i++) {
    filteredPots[i] = analogRead(potPins[i]);
  }

  filteredJoy[0] = analogRead(joy1XPin);
  filteredJoy[1] = analogRead(joy1YPin);
  filteredJoy[2] = analogRead(joy2XPin);
  filteredJoy[3] = analogRead(joy2YPin);

  // NRF module setup
  if (!radio.begin()) {
    Serial.println("NRF24 initialization failed!");
  } else {
    Serial.println("NRF24 initialization successful");
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setChannel(101);
  radio.openWritingPipe(CH_3);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening();

  // Radio transmission with RTOS
  xTaskCreatePinnedToCore(
    TxTask,    // task function
    "TxTask",  // name
    4096,      // stack size in bytes
    NULL,      // parameter
    2,         // priority (2 is arbitrary; adjust if needed)
    NULL,      // returned handle
    0          // pin to core 1
  );

  xTaskCreatePinnedToCore(
    readInputTask,  // task function
    "TxTask",       // name
    2048,           // stack size in bytes
    NULL,           // parameter
    3,              // priority (2 is arbitrary; adjust if needed)
    NULL,           // returned handle
    1               // pin to core 1
  );

  xTaskCreatePinnedToCore(
    showUITask,    // task function
    "showUITask",  // name
    4096,          // stack size in bytes
    NULL,          // parameter
    2,             // priority
    NULL,          // returned handle
    1              // pin to core 1
  );
}

void loop() {

  static unsigned long lastT = 0;
  unsigned long currentT = millis();

  if (currentT - lastT >= 1000) {
    lastT += 1000;               // drift-free
    is_ED_shown = !is_ED_shown;  // toggle state
  }

  //   unsigned long txSendTime = millis();
  //   if (txSendTime - lastTxSendTime >= 3) {
  //     lastTxSendTime = txSendTime;

  //     if (TxRxState == 0) {
  //       if (sendData()) {
  //         Serial.println("Data sent successfully");
  //         // TxRxState = 1;
  //       }
  //     }
  //     // if (TxRxState == 1) {
  //     //    if (receiveData()) {
  //     //      Serial.println("Data received");
  //     //    TxRxState = 0;
  //     //    }
  //     // }
  //   }
}
