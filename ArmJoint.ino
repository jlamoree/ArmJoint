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
 *   Support for DMX Shield (CTC-DRA-10-R2) from https://sourceforge.net/projects/dmxlibraryforar/
 *   
 * Credits
 *   The setup of the 16x2 LCD panel comes from https://dronebotworkshop.com/lcd-displays-arduino/
 *   The code to interface with the AS5147 comes from https://github.com/MarginallyClever/AS5147Test
 */

#define SENSOR_CSEL_PIN_0 2
#define SENSOR_CLK_PIN_0 3
#define SENSOR_MOSI_PIN_0 4
#define SENSOR_MISO_PIN_0 5
#define SENSOR_BOTTOM_14_MASK 0x3FFF
#define SENSOR_TOTAL_BITS 16
#define SENSOR_DATA_BITS 15
#define SENSOR_ANGLE_BITS 14

#define MOTOR_FORWARD_PWM_PIN 9
#define MOTOR_FORWARD_ENABLE_PIN 7
#define MOTOR_REVERSE_PWM_PIN 10
#define MOTOR_REVERSE_ENABLE_PIN 8
#define MOTOR_DIRECTION_INVERT 1 // Enable if you're looking at the backside of the motor
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_REVERSE -1

#define DMX_DEVICE_ADDRESS 200
#define DMX_DEVICE_CHANNELS 1

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Conceptinetics.h>
#include <Easing.h>

const int en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;
const int i2c_addr = 0x27;
LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);

const byte motorSpeedMax = 60; // This is a limitor for the PWM sent to the motor board
const byte motorSpeedMin = 10; // The minimum reasonable speed
const byte motorSpeedHoming = 40; // The speed at which the arm will move to home on startup

const float armAngleDown = 180.0;
const float armAngleUp = 245.0;
const float armAngleTolerance = 0.5;
float armAngle = 0.0;
byte armState = 0; // 0: Not ready, 1: Down, 2: Moving Up, 3: Moving Down, 4: Up

DMX_Slave dmx_device(DMX_DEVICE_CHANNELS);

EasingFunc<Ease::QuadIn> easeIn;
EasingFunc<Ease::QuadOut> easeOut;
const float easeInDuration = 10.0; // Degrees to ramp up
const float easeOutDuration = 10.0; // Degrees to ramp down

void setup() {
  Wire.begin();
  delay(100);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Startup");
  delay(100);

  pinMode(SENSOR_CSEL_PIN_0, OUTPUT);
  pinMode(SENSOR_CLK_PIN_0, OUTPUT);
  pinMode(SENSOR_MOSI_PIN_0, OUTPUT);
  pinMode(SENSOR_MISO_PIN_0, INPUT);
  digitalWrite(SENSOR_CSEL_PIN_0, HIGH);
  digitalWrite(SENSOR_CLK_PIN_0, LOW);
  digitalWrite(SENSOR_MOSI_PIN_0, HIGH);
  
  pinMode(MOTOR_FORWARD_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_FORWARD_PWM_PIN, 0);
  pinMode(MOTOR_FORWARD_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_FORWARD_ENABLE_PIN, HIGH);
  pinMode(MOTOR_REVERSE_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_REVERSE_PWM_PIN, 0);
  pinMode(MOTOR_REVERSE_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_REVERSE_ENABLE_PIN, HIGH);

  dmx_device.onReceiveComplete(dmxFrameReceived);
  dmx_device.enable();
  dmx_device.setStartAddress(DMX_DEVICE_ADDRESS);
  moveToStartupPosition();

  easeIn.duration(easeInDuration);
  easeIn.scale(motorSpeedMax);
  easeOut.duration(easeOutDuration);
  easeOut.scale(motorSpeedMax);
}

void loop() {
  if (armState == 2) {
    moveToArmUpPosition();
  } else if (armState == 3) {
    moveToArmDownPosition();
  }
}

void moveToStartupPosition() {
  int direction;
  char moveLine [16];
  char angleLine [16];
  unsigned long ticks = millis();
  byte displayUpdatePeriod = 100;

  updateArmAngle();
  direction = (armAngle < armAngleDown) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;

  sprintf(moveLine, "Move to %d", (int) armAngleDown);
  sprintf(angleLine, "Angle now %d", (int) armAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(moveLine);
  lcd.setCursor(0, 1);
  lcd.print(angleLine);

  while (abs(armAngle - armAngleDown) > armAngleTolerance) {
    updateMotorDriver(direction, motorSpeedHoming);
    updateArmAngle();
    if (displayUpdatePeriod < millis() - ticks ) {
      sprintf(angleLine, "Angle now %d", (int) armAngle);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(moveLine);
      lcd.setCursor(0, 1);
      lcd.print(angleLine);
      ticks = millis();
    }
  }
  analogWrite(MOTOR_FORWARD_PWM_PIN, 0);
  analogWrite(MOTOR_REVERSE_PWM_PIN, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print(angleLine);
  armState = 1;
  delay(1);
}

void moveToArmUpPosition() {
  byte speed;
  char moveLine [16];
  char angleLine [16];
  unsigned long ticks = millis();
  byte displayUpdatePeriod = 100;

  float angleEaseInEnd = armAngleDown + easeInDuration;
  float angleEaseOutBegin = armAngleUp - easeOutDuration;

  updateArmAngle();
  sprintf(moveLine, "Move to %d", (int) armAngleUp);
  sprintf(angleLine, "Angle now %d", (int) armAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(moveLine);
  lcd.setCursor(0, 1);
  lcd.print(angleLine);

  while (armAngle < armAngleUp && (armAngleUp - armAngle) > armAngleTolerance) {
    if (armAngle < angleEaseInEnd) {
      speed = easeIn.get(armAngle - armAngleDown);
    } else if (armAngle > angleEaseOutBegin) {
      speed = motorSpeedMax - easeOut.get(easeOutDuration - (armAngleUp - armAngle));
    } else {
      speed = motorSpeedMax;
    }
    if (speed < motorSpeedMin) {
      speed = motorSpeedMin;
    }
    updateMotorDriver(MOTOR_DIRECTION_FORWARD, speed);
    updateArmAngle();

    if (displayUpdatePeriod < millis() - ticks) {
      sprintf(angleLine, "Angle now %d", (int) armAngle);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(moveLine);
      lcd.setCursor(0, 1);
      lcd.print(angleLine);
      ticks = millis();
    }
  }
  updateMotorDriver(0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print(angleLine);
  delay(1);
  armState = 4;
}

void moveToArmDownPosition() {
  byte speed;
  char moveLine [16];
  char angleLine [16];
  unsigned long ticks = millis();
  byte displayUpdatePeriod = 100;

  float angleEaseInEnd = armAngleUp - easeInDuration;
  float angleEaseOutBegin = armAngleDown + easeOutDuration;

  updateArmAngle();
  sprintf(moveLine, "Move to %d", (int) armAngleDown);
  sprintf(angleLine, "Angle now %d", (int) armAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(moveLine);
  lcd.setCursor(0, 1);
  lcd.print(angleLine);

  while (armAngle > armAngleDown && (armAngle - armAngleDown) > armAngleTolerance) {
    if (armAngle > angleEaseInEnd) {
      speed = easeIn.get(armAngleUp - armAngle);
    } else if (armAngle < angleEaseOutBegin) {
      speed = motorSpeedMax - easeOut.get(easeOutDuration - (armAngle - armAngleDown));
    } else {
      speed = motorSpeedMax;
    }
    if (speed < motorSpeedMin) {
      speed = motorSpeedMin;
    }
    updateMotorDriver(MOTOR_DIRECTION_REVERSE, speed);
    updateArmAngle();

    if (displayUpdatePeriod < millis() - ticks) {
      sprintf(angleLine, "Angle now %d", (int) armAngle);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(moveLine);
      lcd.setCursor(0, 1);
      lcd.print(angleLine);
      ticks = millis();
    }
  }
  updateMotorDriver(0, 0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print(angleLine);
  delay(1);
  armState = 1;
}

void updateMotorDriver(int direction, byte speed) {
  if (speed == 0) {
    direction == 0;
  } else if (speed > motorSpeedMax) {
    speed = motorSpeedMax;
  }
  if (MOTOR_DIRECTION_INVERT != 0 && direction == MOTOR_DIRECTION_FORWARD) {
    direction = MOTOR_DIRECTION_REVERSE;      
  } else if (MOTOR_DIRECTION_INVERT != 0 && direction == MOTOR_DIRECTION_REVERSE) {
    direction = MOTOR_DIRECTION_FORWARD;
  }

  if (direction == MOTOR_DIRECTION_FORWARD) {
    analogWrite(MOTOR_REVERSE_PWM_PIN, 0);
    analogWrite(MOTOR_FORWARD_PWM_PIN, speed);
  } else if (direction == MOTOR_DIRECTION_REVERSE) {
    analogWrite(MOTOR_FORWARD_PWM_PIN, 0);
    analogWrite(MOTOR_REVERSE_PWM_PIN, speed);
  } else {
    analogWrite(MOTOR_FORWARD_PWM_PIN, 0);
    analogWrite(MOTOR_REVERSE_PWM_PIN, 0);
  }
}

void dmxFrameReceived() {
  byte channelValue = dmx_device.getChannelValue(1);

  if (armState != 1 && armState != 4) {
    return;
  }

  if (armState == 1 && channelValue == 1) {
    armState = 2;
  } else if (armState == 4 && channelValue == 0) {
    armState = 3;
  }
}

void updateArmAngle() {
  uint16_t rawValue;
  if (getSensorRawValue(rawValue) == 0) {
    armAngle = extractAngleFromRawValue(rawValue);
  } else {
    armAngle = 0.0001;
  }
}

/**
 * @param rawValue 16 bit value from AS4157 sensor, including parity and EF bit
 * @return degrees calculated from bottom 14 bits.
 */
float extractAngleFromRawValue(uint16_t rawValue) {
  return (float) (rawValue & SENSOR_BOTTOM_14_MASK) * 360.0 / (float) (1 << SENSOR_ANGLE_BITS);
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
  digitalWrite(SENSOR_CSEL_PIN_0, LOW);
  
  for (int i = 0; i < SENSOR_TOTAL_BITS; ++i) {
    digitalWrite(SENSOR_CLK_PIN_0, HIGH);
    // this is here to give a little more time to the clock going high.
    // only needed if the arduino is *very* fast.  I'm feeling generous.
    result <<= 1;
    digitalWrite(SENSOR_CLK_PIN_0, LOW);
    
    input = digitalRead(SENSOR_MISO_PIN_0);
    result |= input;
    parity ^= (i > 0) & input;
  }
  digitalWrite(SENSOR_CSEL_PIN_0, HIGH);
  
  // check the parity bit
  return (parity != (result >> SENSOR_DATA_BITS));
}
