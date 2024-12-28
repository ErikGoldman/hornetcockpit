#include <Wire.h>
#include <USB.h>
#include <Joystick_ESP32S2.h>
#include <PCA95x5.h>

#define IS_LEFT_DDI 1
#define IS_TEST 0

const int ROTARY_BUTTON_PRESS_TIME_MS = 150;

// Create io extenders
PCA9555 ioex[4];
uint16_t IOEX_ADDRESSES[] = { 0x20, 0x21, 0x22, 0x23 };

// Constants
const int INTERRUPT_PIN = 15;       // Pin connected to the interrupt source
volatile bool interruptFlag = false; // Flag set in the ISR
uint16_t DDIButtonStates[] = { 0xFF, 0xFF, 0xFF, 0xFF };

const int POT_PINS[] = { 1, 2 };
const int THREE_WAY_PINS[] = { 9, 11 };
int ThreeWayState = -1;

const int MASTER_ARM_PIN = 10;
const int MASTER_ARM_BUTTON = 4 * 8;
int MasterArmState = -1;
const int AA_AG_PINS[] = {6, 8};
const int AA_AG_BUTTONS[] = {MASTER_ARM_BUTTON + 2, MASTER_ARM_BUTTON + 3};
int AA_AG_States[] = { -1, -1 };

const int THREE_WAY_BASE = 4 * 8 + 2 + 2; // DDI buttons + master arm + AA + AG buttons
const int NUM_TOTAL_BUTTONS = THREE_WAY_BASE + 3;

unsigned long lastPotMeasurementTime = 0;
const unsigned long POT_REFRESH_TIME_MILLIS = 50;

struct SmoothingBuffer {
  float *buffer;
  int idx;
  int numElems;

  SmoothingBuffer(int numElemsIn) {
    idx = 0;
    numElems = numElemsIn;
    buffer = new float[numElemsIn];
  }

  float addAndRead(float sample) {
    buffer[idx] = sample;
    idx = (idx + 1) % numElems;

    float avg = 0;
    for (int i = 0; i < numElems; i++) {
      avg += (buffer[i] / numElems);
    }
    return avg;
  }
};

SmoothingBuffer PotSmoothers[2] = { SmoothingBuffer(6), SmoothingBuffer(6) };

Joystick_ Joystick(0x01, 
  JOYSTICK_TYPE_JOYSTICK, NUM_TOTAL_BUTTONS, 0,
  true, true, false, false, false, false, // include x and y axes
  true, true, false, false, false);

const int AXIS_RANGE = 128;

void onInterrupt()
{
  interruptFlag = true;
}

void setup()
{
  #if IS_TEST != 1
  char productName[128];
  snprintf(productName, sizeof(productName), "F18 DDI %s", IS_LEFT_DDI ? "LEFT" : "RIGHT");
  if (!USB.PID(0x2341) || !USB.VID(IS_LEFT_DDI ? 0x0002 : 0x0003) || !USB.productName(productName) || !USB.manufacturerName("Goldylox Buttons Inc.")) {
    while(1) {
      Serial.println("Error initializing USB");
      delay(100);
    }
  }
  #endif

  Joystick.setXAxisRange(-AXIS_RANGE, AXIS_RANGE);
  Joystick.setYAxisRange(-AXIS_RANGE, AXIS_RANGE);

	USB.begin();
  Joystick.begin();

  Serial.begin(9600);
  Serial.println(__FILE__);

  Wire.begin();

  // configure the PCA9555s
  for (int i = 0; i < 4; i++) {
    ioex[i].attach(Wire, IOEX_ADDRESSES[i]);
    ioex[i].polarity(PCA95x5::Polarity::ORIGINAL_ALL);
    ioex[i].direction(PCA95x5::Direction::IN_ALL);
  }

  // three way
  for (int i = 0; i < sizeof(THREE_WAY_PINS) / sizeof(THREE_WAY_PINS[0]); i++) {
    pinMode(THREE_WAY_PINS[i], INPUT_PULLUP);
  }

  // aa/ag buttons
  for (int i = 0; i < sizeof(AA_AG_PINS) / sizeof(AA_AG_PINS[0]); i++) {
    pinMode(AA_AG_PINS[i], INPUT_PULLUP);
  }

  // master arm
  pinMode(MASTER_ARM_PIN, INPUT_PULLUP);
  
  // setup interrupt on processor
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), onInterrupt, FALLING);
}

void handleDDIButtons() {
  bool hitSomething = false;

  for (int ddiIdx = 0; ddiIdx < 4; ddiIdx++) {
    const uint16_t buttonStates = ioex[ddiIdx].read();
    const uint16_t diffMask = DDIButtonStates[ddiIdx] ^ buttonStates;

    if (diffMask != 0) {
      for (int i = 0; i < 8; i++) {
        if ((1 << i) & diffMask) {
          bool newValue = ((1 << i) & buttonStates) == 0;
          int buttonNum = ddiIdx * 8 + i;

          Serial.print(newValue ? "DDI Pressed " : "DDI Released ");
          Serial.println(buttonNum);
          Joystick.setButton(buttonNum, newValue);
          bool hitSomething = true;
        }
      }
      DDIButtonStates[ddiIdx] = buttonStates;
    }
  }

  if (hitSomething) {
    delay(20); // bad debouncing
  }
}

void handlePots() {
  unsigned long currTime = millis();
  if (currTime - lastPotMeasurementTime < POT_REFRESH_TIME_MILLIS) {
    return;
  }
  lastPotMeasurementTime = currTime;

  for (int i = 0; i < 2; i++) {
    int rawValue = analogRead(POT_PINS[i]);
    float normalizedValue = rawValue / 8192.0; // Normalize to 0...1 range

    float smoothedValue = PotSmoothers[i].addAndRead(normalizedValue);

    int actualValue = (smoothedValue * 2 * AXIS_RANGE) - AXIS_RANGE;

    if (i == 0) {
      Joystick.setXAxis(actualValue);
    } else {
      Joystick.setYAxis(actualValue);
    }
  }

  // TODO: REMOVE
  //Serial.println("--------------------------------");
  //delay(500);
}

void handleThreeWay() {
  int newState = 2;
  bool hitSomething = false;

  for (int i = 0; i < 2; i++) {
    int value = digitalRead(THREE_WAY_PINS[i]);
    if (value == 0) {
      newState = i;
    }
  }

  // the release can happen before the press. so it might look like release 0, (nothing...), press 2
  // we want to turn that into direct input, so pause after we see the release before we check again
  if (newState != ThreeWayState) {
    delay(30);
    
    for (int i = 0; i < 2; i++) {
      int value = digitalRead(THREE_WAY_PINS[i]);
      if (value == 0) {
        newState = i;
      }      
    }
    
    if (newState != ThreeWayState) {
      Serial.print("3W Released ");
      Serial.println(ThreeWayState);
      Joystick.setButton(THREE_WAY_BASE + ThreeWayState, false);

      Serial.print("3W Pressed ");
      Serial.println(newState);
      Joystick.setButton(THREE_WAY_BASE + newState, true);

      ThreeWayState = newState;
    }
  }
}

void handleMasterArm() {
  int masterArmRead = digitalRead(MASTER_ARM_PIN);
  if (masterArmRead == 0 && MasterArmState != 0) {
      Serial.println("Master arm ON");
      Joystick.setButton(MASTER_ARM_BUTTON + 1, false);
      Joystick.setButton(MASTER_ARM_BUTTON, true);
      MasterArmState = masterArmRead;
      delay(20);
  } else if (masterArmRead != 0 && MasterArmState == 0) {
    Serial.println("Master arm OFF");
    Joystick.setButton(MASTER_ARM_BUTTON, false);
    Joystick.setButton(MASTER_ARM_BUTTON + 1, true);
    MasterArmState = masterArmRead;
    delay(20);
  }
}

void handleAAAGButtons() {
  for (int i = 0; i < 2; i++) {
    int currState = digitalRead(AA_AG_PINS[i]);
    if (currState != AA_AG_States[i]) {
      AA_AG_States[i] = currState;

      Serial.print("AA/AG");
      Serial.print(currState == 0 ? " Pressed " : " Released ");
      Serial.println(i);
      Joystick.setButton(AA_AG_BUTTONS[i], currState == 0);
    }
  }
}

void loop()
{
  handleDDIButtons();
  handlePots();
  handleThreeWay();

  #if IS_LEFT_DDI == 1
    handleMasterArm();
    handleAAAGButtons();
  #endif
}