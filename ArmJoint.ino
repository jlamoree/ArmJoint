/* 
 * BTS7960B H-bridge board
 * 1 Forward PWM (Only one at a time, obviously)
 * 2 Reverse PWM
 * 3 Enable forward (Both enable pins need to be high to enable the driver)
 * 4 Enable reverse
 * 5 Forward overcurrent alarm
 * 6 Reverse overcurrent alarm
 * 7 VCC 5v
 * 8 Ground
 */

/*
 * AS5147 board
 * 5v Power
 * 3v3 Power
 * No connection
 * SPI chip select (active low)
 * SPI clock
 * MISO SPI master data input, slave output
 * MOSI SPI master data output, slave input
 * Ground
 */

/*
 * Libraries
 *   New LiquidCrystal library available from https://github.com/fmalpartida/New-LiquidCrystal
 *   Arduino Timer (arduino-timer) from https://github.com/contrem/arduino-timer
 *   
 * Credits
 *   The setup of the 16x2 LCD panel comes from https://dronebotworkshop.com/lcd-displays-arduino/
 *   The code to interface with the AS5147 comes from https://github.com/MarginallyClever/AS5147Test
 */

#define PIN_SENSOR_CSEL_0 2
#define PIN_SENSOR_CLK_0 3
#define PIN_SENSOR_MOSI_0 4
#define PIN_SENSOR_MISO_0 5
#define BOTTOM_14_MASK (0x3FFF)
#define SENSOR_TOTAL_BITS 16
#define SENSOR_DATA_BITS 15
#define SENSOR_ANGLE_BITS 14

#define MAX_SPEED_POT_PIN A0
#define FORWARD_PWM_PIN 9
#define FORWARD_ENABLE_PIN 7
#define REVERSE_PWM_PIN 10
#define REVERSE_ENABLE_PIN 8

#define FORWARD_BUTTON_PIN 11
#define REVERSE_BUTTON_PIN 12

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <arduino-timer.h>

const int en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;
const int i2c_addr = 0x27;
LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

volatile uint8_t maxMotorSpeed = 0;
volatile int8_t motorDirection = 0;
volatile bool jogButtonDown = false;

auto timer = timer_create_default();

volatile float armAngle = 0.0;

void setup() {
  Wire.begin();
  delay(100);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Startup");

  pinMode(PIN_SENSOR_CSEL_0, OUTPUT);
  pinMode(PIN_SENSOR_CLK_0, OUTPUT);
  pinMode(PIN_SENSOR_MOSI_0, OUTPUT);
  pinMode(PIN_SENSOR_MISO_0, INPUT);

  digitalWrite(PIN_SENSOR_CSEL_0, HIGH);
  digitalWrite(PIN_SENSOR_CLK_0, LOW);
  digitalWrite(PIN_SENSOR_MOSI_0, HIGH);
  
  pinMode(FORWARD_PWM_PIN, OUTPUT);
  analogWrite(FORWARD_PWM_PIN, 0);
  pinMode(FORWARD_ENABLE_PIN, OUTPUT);
  digitalWrite(FORWARD_ENABLE_PIN, HIGH);

  pinMode(REVERSE_PWM_PIN, OUTPUT);
  analogWrite(REVERSE_PWM_PIN, 0);
  pinMode(REVERSE_ENABLE_PIN, OUTPUT);
  digitalWrite(REVERSE_ENABLE_PIN, HIGH);

  pinMode(FORWARD_BUTTON_PIN, INPUT);
  pinMode(REVERSE_BUTTON_PIN, INPUT);
  pinMode(MAX_SPEED_POT_PIN, INPUT);

  timer.every(1000, updateMaxMotorSpeed);
  timer.every(250, updateDisplay);
  timer.every(50, updateJogButtons);
  timer.every(50, updateArmAngle);
}

void loop() {
  timer.tick();
}

bool updateMaxMotorSpeed(void *) {
  maxMotorSpeed = map(analogRead(MAX_SPEED_POT_PIN), 100, 1023, 0, 255);
  return true;
}

bool updateJogButtons(void *) {
  jogButtonDown = false;
  if (digitalRead(FORWARD_BUTTON_PIN) == HIGH) {
    motorDirection = 1;
    jogButtonDown = true;
  } else if (digitalRead(REVERSE_BUTTON_PIN) == HIGH) {
    motorDirection = -1;
    jogButtonDown = true;
  }
  if (jogButtonDown) {
    if (motorDirection == 1) {
      analogWrite(REVERSE_PWM_PIN, 0);
      analogWrite(FORWARD_PWM_PIN, maxMotorSpeed);
    } else if (motorDirection == -1) {
      analogWrite(FORWARD_PWM_PIN, 0);
      analogWrite(REVERSE_PWM_PIN, maxMotorSpeed);
    }
  } else {
    motorDirection = 0;
    analogWrite(FORWARD_PWM_PIN, 0);
    analogWrite(REVERSE_PWM_PIN, 0);
  }
}

bool updateArmAngle(void *) {
  uint16_t rawValue;

  if (getSensorRawValue(rawValue) == 0) {
    armAngle = extractAngleFromRawValue(rawValue);
  } else {
    armAngle = 0.0000;
  }
  return true;
}

boolean updateDisplay(void *) {
  char motorLine [16];
  char angleLine [16];
  int motorSpeed = map(maxMotorSpeed, 0, 255, 0, 100);

  if (motorDirection == 1) {
    sprintf(motorLine, "Motor Fwd %d%%", maxMotorSpeed); 
  } else if (motorDirection == -1) {
    sprintf(motorLine, "Motor Rev %d%%", maxMotorSpeed); 
  } else {
    sprintf(motorLine, "Motor Off");
  }
  sprintf(angleLine, "Angle %d", (int) armAngle);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(motorLine);
  lcd.setCursor(0, 1);
  lcd.print(angleLine);
  return true;
}


/**
 * @param rawValue 16 bit value from AS4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float) (rawValue & BOTTOM_14_MASK) * 360.0 / (float) (1 << SENSOR_ANGLE_BITS);
}

/**
 * @param result where to store the returned value.  may be changed even if method fails.
 * @return 0 on fail, 1 on success.
 * @see https://ams.com/documents/20143/36005/AS5147_DS000307_2-00.pdf/ figure 12
 */
boolean getSensorRawValue(uint16_t &result) {
  result = 0;
  uint8_t input, parity = 0;

  // Send the request for the angle value (command 0xFFFF)
  // at the same time as receiving an angle.

  // Collect the 16 bits of data from the sensor
  digitalWrite(PIN_SENSOR_CSEL_0, LOW);
  
  for (int i = 0; i < SENSOR_TOTAL_BITS; ++i) {
    digitalWrite(PIN_SENSOR_CLK_0, HIGH);
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(PIN_SENSOR_CLK_0, LOW);
    
    input = digitalRead(PIN_SENSOR_MISO_0);
    result |= input;
    parity ^= (i > 0) & input;
  }
  digitalWrite(PIN_SENSOR_CSEL_0, HIGH);
  
  // check the parity bit
  return (parity != (result >> SENSOR_DATA_BITS));
}
