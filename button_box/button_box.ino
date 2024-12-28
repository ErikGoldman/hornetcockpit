#include <Wire.h>

#define WITH_USB 1

#ifdef WITH_USB
#include <USB.h>
#include <Joystick_ESP32S2.h>
#endif

// Constants
const int INTERRUPT_PIN = 39;       // Pin connected to the interrupt source
const gpio_num_t INTERRUPT_GPIO_PIN = GPIO_NUM_39;

const uint8_t pcf8575Address = 0x20; // PCF8575 I2C address (all set low)

volatile bool interruptFlag = false; // Flag set in the ISR

uint16_t lastState = 0xFF;
unsigned long debounceTimes[32];
const unsigned long DEBOUNCE_TIME_MS = 10;

#ifdef WITH_USB
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 32, 0,
  false, false, false, false, false, false,
  true, true, false, false, false);
#endif

const uint16_t BUTTON_PINS[] = {0, 1, 2};
const uint16_t SPST_PINS[] = {3, 4, 5, 6, 7};
const uint16_t SPST_VIRTUAL_PINS[] = {16, 17, 18, 19, 20};
bool SPST_VIRTUAL_PIN_STATES[] = {false, false, false, false, false};

const uint16_t THREE_POS1_PINS[] = {8, 9};
const uint16_t THREE_POS1_OFF_BUTTON = 31;
bool THREE_POS1_OFF_BUTTON_STATE = false;

const uint16_t DIAL1_PINS[] = {10, 11, 12, 13};
const uint16_t DIAL1_OFF_BUTTON = 30;
bool Dial1OffButtonState = false;
unsigned long dialDebounceTime = 0;

const uint16_t PB_ARDUINO_PINS[] = {2, 3, 5, 4};
int PbIsPull = -1;
int PbIsCCW = -1;
int PbLastHit = -1;
const uint16_t PB_BTN_CCW = 26;
const uint16_t PB_BTN_CW = 27;
const uint16_t PB_BTN_PULL = 28;
const uint16_t PB_BTN_STOW = 29;

bool isWaitingOnDebounce = false;

// ISR for the interrupt pin
void handleInterrupt() {
  interruptFlag = true;
}

void setup() {
  // Initialize serial monitor for debugging
  Serial.begin(19200);
  delay(50);
  Serial.println("Booting");

  if (sizeof(SPST_PINS) / sizeof(SPST_PINS[0]) != sizeof(SPST_VIRTUAL_PINS) / sizeof(SPST_VIRTUAL_PINS[0])) {
    Serial.println("ERROR: Mismatch between real and virtual SPST pin arrays");
  } else if (sizeof(SPST_PINS) / sizeof(SPST_PINS[0]) != sizeof(SPST_VIRTUAL_PIN_STATES) / sizeof(SPST_VIRTUAL_PIN_STATES[0])) {
    Serial.println("ERROR: Mismatch between real and virtual SPST state array");
  }

#ifdef WITH_USB
  USB.PID(0x2341);
	USB.VID(0x2341);
  USB.productName("Parking button box");
  USB.manufacturerName("Goldylox Buttons Inc.");
	
	USB.begin();
  Joystick.begin();
#endif

  memset(debounceTimes, 0, sizeof(debounceTimes));
  
  // Set up the interrupt pin for the I2C
  pinMode(INTERRUPT_PIN, INPUT_PULLUP); // Using internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING); // Interrupt on falling edge
  esp_sleep_enable_ext0_wakeup(INTERRUPT_GPIO_PIN, LOW);  // Wake up on LOW signal on !INT

  // set up PB input pins on the board itself
  for (int i = 0; i < sizeof(PB_ARDUINO_PINS) / sizeof(PB_ARDUINO_PINS[0]); i++) {
    pinMode(PB_ARDUINO_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PB_ARDUINO_PINS[i]), handleInterrupt, CHANGE);
  }
  
  // Initialize I2C communication
  Wire.begin();
  runButtonLogic(false);
  Serial.print("Setup complete. Initial state is ");
  Serial.println(lastState, BIN);
  Serial.flush();
}

void loop() {
  // zzzzz
  //Serial.println("Zzzzz");
  //Serial.flush();
  //esp_light_sleep_start();

  // Check if the interrupt has fired
  if (interruptFlag) {
    interruptFlag = false; // Reset flag

    runButtonLogic(true);
    while (isWaitingOnDebounce)  {
      delay(5);
      runButtonLogic(true);      
    }
  }
}

void runButtonLogic(bool withDebounce) {
    // Read input states from the PCF8575
    uint16_t inputState = readPCF8575();
    uint16_t diff = lastState ^ inputState;
    uint16_t debounceMask = makeDebounceMask();
    uint16_t hit = withDebounce ? diff & debounceMask : diff;

    isWaitingOnDebounce = ~debounceMask > 0;

    Serial.println(diff, BIN);
    Serial.println(hit, BIN);

    for (int i = 0; i < 16; i++) {
      if (hit & 1 << i) {
        debounceTimes[i] = millis();

        bool bWasReleased = inputState & 1 << i;

        if (bWasReleased) {
          Serial.print("Released ");  
        } else {
          Serial.print("Pressed ");  
        }
        Serial.print("input ");
        Serial.println(i);

        #ifdef WITH_USB  
       	Joystick.setButton(i, !bWasReleased);
        #endif
      }
    }

    HandlePBState();

    uint16_t nextState = lastState ^ hit;

    // correct for unwired inputs on 3pos, and SPST
    correctForUnwired(nextState, THREE_POS1_PINS, sizeof(THREE_POS1_PINS) / sizeof(THREE_POS1_PINS[0]), THREE_POS1_OFF_BUTTON, &THREE_POS1_OFF_BUTTON_STATE);    
    for (int i = 0; i < sizeof(SPST_PINS) / sizeof(SPST_PINS[0]); i++) {
      correctForUnwired(nextState, &SPST_PINS[i], 1, SPST_VIRTUAL_PINS[i], &SPST_VIRTUAL_PIN_STATES[i]);
    }

    // correct for unwired on dial -- requires debounce to settle since first we release the old value then we press the new one
    maybeDebounceDial(nextState, diff);

    if (dialDebounceTime <= millis()) {
      bool allOff = true;
      for (int i = 0; i < sizeof(DIAL1_PINS) / sizeof(DIAL1_PINS[0]); i++) {
        if ((nextState & 1 << DIAL1_PINS[i]) == 0) {
          allOff = false;
          if (Dial1OffButtonState) {
            #ifdef WITH_USB
              Joystick.setButton(DIAL1_OFF_BUTTON, false);
            #endif
            Serial.print("Released unwired ");
            Serial.println(DIAL1_OFF_BUTTON);    
            Dial1OffButtonState = false;
            break;
          }
        }
      }

      if (allOff && !Dial1OffButtonState) {
        #ifdef WITH_USB
          Joystick.setButton(DIAL1_OFF_BUTTON, true);
        #endif
        Serial.print("Pressed unwired ");
        Serial.println(DIAL1_OFF_BUTTON);    
        Dial1OffButtonState = true;
      }
    } else {
      isWaitingOnDebounce = true;
    }
    
    lastState = nextState;
}

void maybeDebounceDial(uint16_t nextState, uint16_t hit) {
  for (int i = 0; i < sizeof(DIAL1_PINS) / sizeof(DIAL1_PINS[0]); i++) {
    if (hit & 1 << DIAL1_PINS[i]) {
      // ok, something got hit in the dial. just delay for now (w/e) and then we'll check again
      Serial.println("Debouncing dial");
      dialDebounceTime = millis() + 25;
      return;
    }
  } 
}

// checks the pins[] array to see if any pin has been set. potentially sets or unsets unwiredButtonState and presses unwiredButton in response
void correctForUnwired(uint16_t newState, const uint16_t *pins, int numPins, int unwiredButton, bool *unwiredButtonState) {
    for (int i = 0; i < numPins; i++) {      
      if ((newState & 1 << pins[i]) == 0) {
        if (*unwiredButtonState) {
          #ifdef WITH_USB
            Joystick.setButton(unwiredButton, false);
          #endif
          Serial.print("Released unwired ");
          Serial.println(unwiredButton);          
          *unwiredButtonState = false;
        }
        return;
      }
    }

    if (*unwiredButtonState == false) {
      #ifdef WITH_USB
          Joystick.setButton(unwiredButton, true);
      #endif
        Serial.print("Pressed unwired ");
        Serial.println(unwiredButton);      
      *unwiredButtonState = true;
    }
}

void HandlePBState() {
  for (int i = 0; i < sizeof(PB_ARDUINO_PINS) / sizeof(PB_ARDUINO_PINS[0]); i++) {
    const int buttonState = !digitalRead(PB_ARDUINO_PINS[i]);  

    if (buttonState && PbLastHit != i) {
      Serial.print("PB hit ");
      Serial.println(i);

      if (i == 1) {
        // CCW stowed
        // if we came from CCW pulled, do a little dance
        if (PbIsCCW == 1 && PbIsPull == 1) {
          Serial.println("PB CCW Pull -> Stow");
          Joystick.setButton(PB_BTN_CW, false);
          Joystick.setButton(PB_BTN_PULL, false);
          Joystick.setButton(PB_BTN_CCW, false);
          delay(50);
          Serial.println("CCW hit");
          Joystick.setButton(PB_BTN_CCW, true);
          delay(150);
          Joystick.setButton(PB_BTN_CCW, false);
          delay(700);
          Serial.println("CCW hit 2");
          Joystick.setButton(PB_BTN_CCW, true);
          delay(150);
          Joystick.setButton(PB_BTN_CCW, false);
          PbIsCCW = 1;
          PbIsPull = 0;
        } else {
          if (PbIsCCW != 1) {
            Serial.println("PB CCW");
            PbIsCCW = 1;
            Joystick.setButton(PB_BTN_CW, false);  
            Joystick.setButton(PB_BTN_CCW, true);  
          }
          if (PbIsPull != 0) {
            Serial.println("PB Stow");
            PbIsPull = 0;
            Joystick.setButton(PB_BTN_PULL, false);
            Joystick.setButton(PB_BTN_STOW, true);  
          }
        }
        
      } else if (i == 3) {
        // CW stowed
        if (PbIsCCW != 0) {
          Serial.println("PB CW");
          PbIsCCW = 0;
          Joystick.setButton(PB_BTN_CCW, false);  
          Joystick.setButton(PB_BTN_CW, true);  
        }
        if (PbIsPull != 0) {
          Serial.println("PB Stow");
          PbIsPull = 0;
          Joystick.setButton(PB_BTN_PULL, false);
          Joystick.setButton(PB_BTN_STOW, true);  
        }
      } else if (i == 0) {
        // pull while CW
        if (PbIsCCW != 0) {
          Serial.println("PB CW");
          PbIsCCW = 0;
          Joystick.setButton(PB_BTN_CCW, false);  
          Joystick.setButton(PB_BTN_CW, true);  
        }
        if (PbIsPull != 1) {
          Serial.println("PB Pull");
          PbIsPull = 1;
          Joystick.setButton(PB_BTN_STOW, false);  
          Joystick.setButton(PB_BTN_PULL, true);
        }
      } else if (i == 2) {
        // pull while CCW
        if (PbIsCCW != 1) {
          Serial.println("PB CCW");
          PbIsCCW = 0;
          Joystick.setButton(PB_BTN_CW, false);  
          Joystick.setButton(PB_BTN_CCW, true);  
        }
        if (PbIsPull != 1) {
          Serial.println("PB Pull");
          PbIsPull = 1;
          Joystick.setButton(PB_BTN_STOW, false);  
          Joystick.setButton(PB_BTN_PULL, true);
        }
      }

      PbLastHit = i;
      isWaitingOnDebounce = true;
      return;
    }
  }
}

uint16_t makeDebounceMask() {
  unsigned long currTime = millis();

  uint16_t out = 0;
  for (int i = 0; i < 16; i++) {
    if (currTime - debounceTimes[i] > DEBOUNCE_TIME_MS) {
      out |= 1 << i;
    }
  }
  return out;
}

// Function to read all inputs from the PCF8575
uint16_t readPCF8575() {
  uint16_t state = 0xFFFF; // Default state for PCF8575 (all high)
  
  // Send a request to read 2 bytes from the PCF8575
  Wire.requestFrom(pcf8575Address, (uint8_t)2);
  delay(50); // Add small delay
  
  if (Wire.available() == 2) {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    state = (highByte << 8) | lowByte; // Combine high and low bytes
  } else {
    Serial.println("Error: Could not read from PCF8575");
  }

  return state;
}