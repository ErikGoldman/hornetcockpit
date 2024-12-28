#include <Wire.h>

#define WITH_USB 1

#ifdef WITH_USB
#include <USB.h>
#include <Joystick_ESP32S2.h>
#endif

#include <Adafruit_TCA8418.h>
#include <PCA95x5.h>

const int ROTARY_BUTTON_PRESS_TIME_MS = 150;

struct RotaryWithButton {
  int buttonPin;
  int buttonState;
  int btnBase;
  Joystick_ *joystick;

  bool currentDirectionCw = true;
  unsigned long lastBounceMillis = 0;

  int p1, p2;

  RotaryWithButton(int pin1, int pin2, int bPin, int btnBaseIn, Joystick_ *j) {
    buttonPin = bPin;
    p1 = pin1;
    p2 = pin2;
    joystick = j;

    btnBase = btnBaseIn;
    buttonState = 0;
  }

  void init() {    
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(p1, INPUT_PULLUP);
    pinMode(p2, INPUT_PULLUP);
  }

  void loop() {
    int btn = digitalRead(buttonPin);
    if (btn != buttonState) {
      if (btn == 0) {
        Serial.println("Encoder pressed");
      	joystick->setButton(btnBase, true);
      } else {
        Serial.println("Encoder released");
        joystick->setButton(btnBase, false);
      }
      buttonState = btn;
      delay(20); // bad debouncing
    }

    int p1v = digitalRead(p1);
    if (p1v == 0) {
      int p2v = digitalRead(p2);
      bool isCw = (p2v == p1v);

      // sometimes we get a random opposite direction sprinkled in as noise
      if (millis() - lastBounceMillis > 200 || isCw == currentDirectionCw) {
        if (p2v == p1v) {
          Serial.println("CW");
          joystick->setButton(btnBase + 1, true);
          delay(ROTARY_BUTTON_PRESS_TIME_MS); // simulate click
          joystick->setButton(btnBase + 1, false);
        } else {
          Serial.println("CCW");
          joystick->setButton(btnBase + 2, true);
          delay(ROTARY_BUTTON_PRESS_TIME_MS); // simulate click
          joystick->setButton(btnBase + 2, false);
        }
        delay(20);

        lastBounceMillis = millis();
        currentDirectionCw = isCw;
      }
    }
  }
};

// Create io extenders
Adafruit_TCA8418 keypad = Adafruit_TCA8418();
PCA9555 ioex;

// Constants
const int INTERRUPT_PIN = 15;       // Pin connected to the interrupt source
const gpio_num_t INTERRUPT_GPIO_PIN = GPIO_NUM_15;
const int NUM_ROWS = 5;
const int NUM_COLS = 4;
const int TCA_INPUTS[] = {3, 13, 16, 17, 18};

// matrix buttons 0 - 19 are the 5x4 keypad
int MATRIX_BTN_BASE = 0;
// 20-36 are the TCA_INPUTs (function select)
int TCA_BTN_BASE = 20;
// 37, 38, 39 for R1
const int R1_BTN_BASE = 37;
// 40, 41, 42 for R2
const int R2_BTN_BASE = 40;
// 43 - 59 for keypad individual inputs
const int MATRIX_INDIVIDUAL_BTN_BASE = 43;
const int NUM_TOTAL_BUTTONS = MATRIX_INDIVIDUAL_BTN_BASE + 24;


const uint8_t TCA_ADDRESS = 0x34; // TCA8418RTWR I2C address

volatile bool interruptFlag = false; // Flag set in the ISR
uint16_t FunctionButtonStates = 0xFF;

#ifdef WITH_USB
Joystick_ Joystick(0x01, 
  JOYSTICK_TYPE_JOYSTICK, NUM_TOTAL_BUTTONS, 0,
  false, false, false, false, false, false,
  true, true, false, false, false);
#endif

RotaryWithButton r1(36, 38, 40, R1_BTN_BASE, &Joystick);
RotaryWithButton r2(17, 21, 34, R2_BTN_BASE, &Joystick);


void TCA8418_irq_fired()
{
  interruptFlag = true;
  Serial.println("INT");
}

void setup()
{
  #if WITH_USB == 1
    if (!USB.PID(0x2341) || !USB.VID(0x0001) || !USB.productName("F18 UFC") || !USB.manufacturerName("Goldylox Buttons Inc.")) {
      while(1) {
        Serial.println("Error initializing USB");
        delay(100);
      }
    }
    
    USB.begin();
    Joystick.begin();
  #endif

  Serial.begin(9600);
  Serial.println(__FILE__);

  Wire.begin();
  while (!keypad.begin(TCA_ADDRESS, &Wire)) {
    Serial.println("keypad not found, check wiring & pullups!");
  }

  // configure the keypad (matrix + gpios)
  keypad.matrix(NUM_ROWS, NUM_COLS);
  for (int i = 0; i < sizeof(TCA_INPUTS) / sizeof(TCA_INPUTS[0]); i++)
  {
    const int pin = TCA_INPUTS[i] - 1;

    keypad.pinMode(pin, INPUT_PULLUP);
    keypad.pinIRQMode(pin, CHANGE);
  }

  // configure the PCA9555
  ioex.attach(Wire);
  ioex.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  ioex.direction(PCA95x5::Direction::IN_ALL);

  // setup interrupt on processor
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), TCA8418_irq_fired, FALLING);

  r1.init();
  r2.init();

  // flush the internal buffer
  keypad.flush();
}

void handleKeypad() {
if (keypad.available() > 0)
  {
    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();

    bool bWasPressed = false;
    if (k & 0x80) {
      Serial.print("PRESS: ");
      bWasPressed = true;
    } else {
      Serial.print("RELEASE: ");
    }
    k &= 0x7F;

     if (k > 96)  //  GPIO
      {
        //  process  gpio
        k -= 97;
        Serial.print("B");
        Serial.println(k);
        #if WITH_USB == 1
          Joystick.setButton(k + MATRIX_INDIVIDUAL_BTN_BASE, bWasPressed);
        #endif
      }
      else
      {
        //  process  matrix
        k--;
        const int row = k / 10;
        const int col = k % 10;
        const int buttonNum = col * NUM_ROWS + row;
        Serial.print("M");
        Serial.println(buttonNum);
        #if WITH_USB == 1
          Joystick.setButton(buttonNum + MATRIX_BTN_BASE, bWasPressed);
        #endif
      }
   
  }
}

void handleFunctionButtons() {
  const uint16_t buttonStates = ioex.read();
  const uint16_t diffMask = FunctionButtonStates ^ buttonStates;

  if (diffMask != 0) {
    for (int i = 0; i < 16; i++) {
      if ((1 << i) & diffMask) {
        bool newValue = ((1 << i) & buttonStates) == 0;
        Serial.print(newValue ? "FB Pressed " : "FB Released ");
        Serial.println(i);

        #if WITH_USB == 1
          Joystick.setButton(i + TCA_BTN_BASE, newValue);
        #endif
      }
    }
    FunctionButtonStates = buttonStates;
    delay(20); // bad debouncing
  }
}

void handleRotary() {
  r1.loop();
  r2.loop();
}

void loop()
{
  handleKeypad();
  handleFunctionButtons();
  handleRotary();
}