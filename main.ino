#include <StopWatch.h>
StopWatch sw(StopWatch::SECONDS);

#include <TimerOne.h>

#include <EEPROM.h>

#include <PrintEx.h>

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte char_tilde[8] = { // ~
  0b00000,
  0b00000,
  0b01000,
  0b10101,
  0b00010,
  0b00000,
  0b00000,
  0b00000
};

byte char_km[8] = { // km
  0b10100,
  0b11000,
  0b10100,
  0b00000,
  0b11111,
  0b10101,
  0b10101,
  0b00000
};

byte char_kW[8] = { // kW
  0b10100,
  0b11000,
  0b10100,
  0b00000,
  0b10101,
  0b10101,
  0b01010,
  0b00000
};

byte char_gradC[8] = { // °C
  0b11000,
  0b11000,
  0b00111,
  0b01000,
  0b01000,
  0b01000,
  0b00111,
  0b00000
};

const byte CHR_TILDE       = 0x00;
const byte CHR_KM          = 0x05;
const byte CHR_KW          = 0x06;
const byte CHR_GRADCELSIUS = 0x07;

#include <LcdBarGraph.h>
LcdBarGraph lbg(&lcd, 16, 0, 1);

#include <SPI.h>
#include "mcp_can.h"
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

#include <AnalogButtons.h>
Button btnRIGHT = Button(0, &btnRIGHTClick);
Button btnUP = Button(99, &btnUPClick);
Button btnDOWN = Button(255, &btnDOWNClick);
Button btnLEFT = Button(407, &btnLEFTClick);
Button btnSELECT = Button(637, &btnSELECTClick);
AnalogButtons analogButtons = AnalogButtons(A0, INPUT);

union union64
{
  unsigned char uc[8];   //  8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  byte           b[8];   //  8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  uint8_t      ui8[8];   //  8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  uint64_t       ui64;   // 64 bit (4 byte) 0 to 4,294,967,295 / 0 bis (2^64) - 1)
};

union64       buf;

boolean       flagRecv = 0;
boolean       flagDisp = 0;

byte          len = 0;

byte          pageno = 0;

uint64_t      pid_0x1f6 = UINT64_MAX;
uint64_t      pid_0x1fd = UINT64_MAX;
uint64_t      pid_0x212 = UINT64_MAX;
uint64_t      pid_0x391 = UINT64_MAX;
uint64_t      pid_0x427 = UINT64_MAX;
uint64_t      pid_0x42a = UINT64_MAX;
uint64_t      pid_0x42e = UINT64_MAX;
uint64_t      pid_0x430 = UINT64_MAX;
uint64_t      pid_0x432 = UINT64_MAX;
uint64_t      pid_0x4f8 = UINT64_MAX;
uint64_t      pid_0x5d1 = UINT64_MAX;
uint64_t      pid_0x5d7 = UINT64_MAX;
uint64_t      pid_0x5da = UINT64_MAX;
uint64_t      pid_0x5ee = UINT64_MAX;
uint64_t      pid_0x62d = UINT64_MAX;
uint64_t      pid_0x634 = UINT64_MAX;
uint64_t      pid_0x637 = UINT64_MAX;
uint64_t      pid_0x638 = UINT64_MAX;
uint64_t      pid_0x646 = UINT64_MAX;
uint64_t      pid_0x650 = UINT64_MAX;
uint64_t      pid_0x652 = UINT64_MAX;
uint64_t      pid_0x654 = UINT64_MAX;
uint64_t      pid_0x656 = UINT64_MAX;
uint64_t      pid_0x657 = UINT64_MAX;
uint64_t      pid_0x658 = UINT64_MAX;
uint64_t      pid_0x65b = UINT64_MAX;
uint64_t      pid_0x673 = UINT64_MAX;
uint64_t      pid_0x68b = UINT64_MAX;
uint64_t      pid_0x68c = UINT64_MAX;
uint64_t      pid_0x6f8 = UINT64_MAX;

PrintEx lcdEx = lcd;


const byte    DAY_BRIGHTNESS = UINT8_MAX;
const byte    PAGE_LAST = 10;


bool isCharging = false;

unsigned int LocalTime = 0;
unsigned int ChargeRemainingTime = 0;
unsigned int ChargeBeginTime = 0;
unsigned int ChargeEndTime = 0;


void setup()
{
  lcd.begin(16, 2);               // start the library
  lcd.clear();
  lcd.setCursor(0, 0);            // set the LCD cursor   position

  lcd.print(F("ZOE"));


  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    delay(100);
  }

  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt


  /*
     set mask, set both the mask to 0x3ff
  */

  //mask0
  CAN.init_Mask(0, 0, 0xc00);                         // there are 2 mask in mcp2515, you need to set both of them

  //mask1
  CAN.init_Mask(1, 0, 0x7ff);

  /*
     set filter, we can receive id from
  */

  //filter0
  CAN.init_Filt(0, 0, 0x4ff); //0x400 - 0x7ff
  CAN.init_Filt(1, 0, 0x4ff);

  //filter1
  CAN.init_Filt(2, 0, 0x391);
  CAN.init_Filt(3, 0, 0x212);
  CAN.init_Filt(4, 0, 0x1fd);
  CAN.init_Filt(5, 0, 0x1f6);


  analogButtons.add(btnRIGHT);
  analogButtons.add(btnUP);
  analogButtons.add(btnDOWN);
  analogButtons.add(btnLEFT);
  analogButtons.add(btnSELECT);

  pinMode(3, OUTPUT);
  analogWrite(3, DAY_BRIGHTNESS);

  lcd.createChar(0, char_tilde);
  lcd.createChar(5, char_km);
  lcd.createChar(6, char_kW);
  lcd.createChar(7, char_gradC);

  (EEPROM.read(0) <= PAGE_LAST) ? pageno = EEPROM.read(0) : pageno = 0;

  Timer1.initialize(250000);

  Timer1.attachInterrupt(LCD_ISR);

  sw.reset();
}


void MCP2515_ISR()
{
  flagRecv = true;
}


void LCD_ISR()
{
  flagDisp = true;
}


void btnRIGHTClick() {
  lcd.clear();
  if (pageno < PAGE_LAST) {
    pageno++;
    flagDisp = true;
  }
}

void btnUPClick() {
  //lcd.clear();
  sw.start();
  //if (brightness < UINT8_MAX) brightness = brightness + 5;
  //analogWrite(3, brightness);
}

void btnDOWNClick() {
  //lcd.clear();
  sw.stop();
  //if (brightness > 0) brightness = brightness - 5;
  //analogWrite(3, brightness);
}

void btnLEFTClick() {
  lcd.clear();
  if (pageno > 0) {
    pageno--;
    flagDisp = true;
  }
}

void btnSELECTClick() {
  lcd.clear();
  EEPROM.update(0, pageno);
}


void loop()
{
  static bool lastCharging = false;
  static bool lastPlugged = false;

  analogButtons.check();

  if (flagRecv) { // check if get data
    flagRecv = false; // clear flag

    // iterate over all pending messages
    // If either the bus is saturated or the MCU is busy,
    // both RX buffers may be in use and reading a single
    // message does not clear the IRQ conditon.
    while (CAN_MSGAVAIL == CAN.checkReceive()) {
      // read data,  len: data length, buf: data buf
      CAN.readMsgBuf(&len, buf.b);

      switch (CAN.getCanId()) {
        case 0x1f6:
          pid_0x1f6 = swap_uint64(buf.ui64);
          break;
        case 0x1fd:
          pid_0x1fd = swap_uint64(buf.ui64);
          break;
        case 0x212:
          pid_0x212 = swap_uint64(buf.ui64);
          break;
        case 0x391:
          pid_0x391 = swap_uint64(buf.ui64);
          break;
        case 0x427:
          pid_0x427 = swap_uint64(buf.ui64);
          break;
        case 0x42a:
          pid_0x42a = swap_uint64(buf.ui64);
          break;
        case 0x42e:
          pid_0x42e = swap_uint64(buf.ui64);
          break;
        case 0x430:
          pid_0x430 = swap_uint64(buf.ui64);
          break;
        case 0x432:
          pid_0x432 = swap_uint64(buf.ui64);
          break;
        case 0x4f8:
          pid_0x4f8 = swap_uint64(buf.ui64);
          break;          
        case 0x5d1:
          pid_0x5d1 = swap_uint64(buf.ui64);
          break;
        case 0x5d7:
          pid_0x5d7 = swap_uint64(buf.ui64);
          break;
        case 0x5da:
          pid_0x5da = swap_uint64(buf.ui64);
          break;
        case 0x5ee:
          pid_0x5ee = swap_uint64(buf.ui64);
          // LightSensorStatus, NightRheostatedLightMaxPercent
          (pid_0x5ee & 0x10u) ? analogWrite(3, DAY_BRIGHTNESS) : analogWrite(3, (pid_0x5ee >> 24) & 0xFFu);
          break;
        case 0x62d:
          pid_0x62d = swap_uint64(buf.ui64);
          break;
        case 0x634:
          pid_0x634 = swap_uint64(buf.ui64);
          break;          
        case 0x637:
          pid_0x637 = swap_uint64(buf.ui64);
          break;
        case 0x638:
          pid_0x638 = swap_uint64(buf.ui64);
          break;
        case 0x646:
          pid_0x646 = swap_uint64(buf.ui64);
          break;
        case 0x650:
          pid_0x650 = swap_uint64(buf.ui64);
          break;
        case 0x652:
          pid_0x652 = swap_uint64(buf.ui64);
          break;
        case 0x654:
          pid_0x654 = swap_uint64(buf.ui64);
          ChargeRemainingTime = (((pid_0x654 >> 22) & 0x3ffu) < 0x3ff) ? (pid_0x654 >> 22) & 0x3ffu : 0;
          if (!lastPlugged & ((pid_0x654 >> 61) & 0x1u)) {
            sw.stop();
            sw.reset();
          }
          lastPlugged = ((pid_0x654 >> 61) & 0x1u);
          break;
        case 0x656:
          pid_0x656 = swap_uint64(buf.ui64);
          break;
        case 0x657:
          pid_0x657 = swap_uint64(buf.ui64);
          break;
        case 0x658:
          pid_0x658 = swap_uint64(buf.ui64);
          isCharging = (pid_0x658 & 0x200000u);
          if (isCharging) ChargeEndTime = LocalTime + ChargeRemainingTime;
          if (isCharging != lastCharging) {
            if (isCharging) {
              sw.start();
              ChargeBeginTime = LocalTime;
            } else {
              sw.stop();
              ChargeEndTime = LocalTime;
            }
          }
          lastCharging = isCharging;
          break;
        case 0x65b:
          pid_0x65b = swap_uint64(buf.ui64);
          break;
        case 0x673:
          pid_0x673 = swap_uint64(buf.ui64);
          break;
        case 0x68b:
          pid_0x68b = swap_uint64(buf.ui64);
          break;
        case 0x68c:
          pid_0x68c = swap_uint64(buf.ui64);
          LocalTime = ((pid_0x68c >> 32) & 0x7ffu);
          break;
        case 0x6f8:
          pid_0x6f8 = swap_uint64(buf.ui64);
          break;
      }
    }
  }

  if (flagDisp) { // check if get data
    flagDisp = false; // clear flag

    lcd.setCursor(0, 0);

    switch (pageno) {
      case 0: // battery bar
        lcdEx.printf("%4.1fkWh", ((pid_0x427 >> 6) & 0x1FFu) * 0.1f); // AvailableEnergy
        lcdEx.printf("  %6.2f%%", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02f); // UserSOC
        lbg.drawValue((pid_0x42e >> 51) & 0x1FFFu, 5000); // UserSOC BAR
        break;
      case 1: // battery
        lcd.print(F("BAT "));
        lcdEx.printf("%6.2f%% ", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02f); // UserSOC
        switch ((pid_0x432 >> 26) & 0x3u) { //  HVBatConditionningMode
          case 1:
            lcd.print(F("C"));
            break;
          case 2:
            lcd.print(F("H"));
            break;
          default:
            lcd.print(F(" "));
            break;
        }
        lcdEx.printf("%2d", ((pid_0x42e >> 13) & 0x7Fu) - 40); // HVBatteryTemp
        lcd.write(CHR_GRADCELSIUS);
        lcd.setCursor(0, 1);
        lcdEx.printf("%3u%%", (pid_0x658 >> 24) & 0x7Fu); // HVBatHealth
        lcdEx.printf(" %4.1f", ((pid_0x427 >> 6) & 0x1FFu) * 0.1f); // AvailableEnergy
        lcd.write(CHR_KW); lcd.print(F("h"));
        lcdEx.printf(" %3fV", ((pid_0x42e >> 29) & 0x3FFu) * 0.5f); // HVNetworkVoltage
        break;
      case 2: // charge
        lcd.print(F("CHG "));
        ((pid_0x427 >> 5) & 0x1u) ? lcd.write(CHR_TILDE) : lcd.print(F(" ")); // ChargeAvailable
        //((pid_0x658 >> 21) & 0x1u) ? lcd.print(F("=")) : lcd.print(F(" ")); // ChargeInProgress
        lcdEx.printf(" %3u%%", (pid_0x654 >> 32) & 0x7Fu); // HVBatteryEnergyLevel
        lcdEx.printf(" %4.1f", (pid_0x42e & 0xFFu) * 0.3f); // ChargingPower
        lcd.write(CHR_KW);
        lcd.setCursor(0, 1);
        lcdEx.printf("%2uA", (pid_0x42e >> 20) & 0x3Fu); // MaxChargingNegotiatedCurrent
        lcdEx.printf(" %5uW", ((pid_0x62d >> 35) & 0x1FFu) * 100); // BCBPowerMains
        lcdEx.printf(" %4.1f", ((pid_0x427 >> 16) & 0xFFu) * 0.3f); // AvailableChargingPower
        lcd.write(CHR_KW);
        break;
      case 3: // times
        lcd.print(F("TIM  "));
        lcdEx.printf("%02u:%02u", ChargeBeginTime / 60u, ChargeBeginTime % 60u);
        lcdEx.printf("-%02u:%02u", ChargeEndTime / 60u, ChargeEndTime % 60u);
        lcd.setCursor(0, 1);
        lcdEx.printf("%02u", (sw.value() / 3600u) % 24u);
        lcdEx.printf(":%02u", (sw.value() / 60u) % 60u);
        lcdEx.printf(":%02u   ", sw.value() % 60u);
        lcdEx.printf("%02u:%02u", LocalTime / 60u, LocalTime % 60u);
        break;
      case 4: // clima
        lcd.print(F("CLM "));
        // ClimLoopMode
        switch ((pid_0x42a >> 13) & 0x7u) {
          case 0:
            lcd.print(F("n/a   "));
            break;
          case 1:
            lcd.print(F("Cool  "));
            break;
          case 2:
            lcd.print(F("De-Ice"));
            break;
          case 4:
            lcd.print(F("Heat  "));
            break;
          case 6:
            lcd.print(F("Demist"));
            break;
          case 7:
            lcd.print(F("Idle  "));
            break;
          default:
            lcd.print(F("      "));
            break;
        }
        // PTCActivationRequest
        (((pid_0x42a >> 11) & 0x3u) == 1) ? lcd.print(F("P")) : lcd.print(F(" "));
        lcdEx.printf(" %3f%%", ((pid_0x42a >> 34) & 0x3Fu) * 2.12766f); // ClimAirFlow
        lcd.setCursor(0, 1);
        lcdEx.printf("%3f", (((pid_0x42a >> 48) & 0x3FFu) * 0.1f) - 40); // EvaporatorTempSetPoint
        lcdEx.printf("%3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1f) - 40); // EvaporatorTempMeasure
        lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5f) - 30); // CompTemperatureDischarge
        lcd.write(CHR_GRADCELSIUS);
        lcdEx.printf("%5dW", (((pid_0x1fd >> 16) & 0xffu) * -25) + 5000); // ClimAvailablePower
        break;
      case 5: // mission
        lcd.print(F("MIS   "));
        lcd.write(byte(246));
        // ConsumptionSinceMissionStart + AuxConsumptionSinceMissionStart - RecoverySinceMissionStart = TotalConsumptionSinceMissionStart
        lcdEx.printf(":%5.1fkWh", (((int)((pid_0x637 >> 54) & 0x3FFu) + (int)((pid_0x637 >> 34) & 0x3FFu)) - (int)((pid_0x637 >> 44) & 0x3FFu)) * 0.1f);
        lcd.setCursor(0, 1);
        // ConsumptionSinceMissionStart, RecoverySinceMissionStart, AuxConsumptionSinceMissionStart
        lcdEx.printf("%4.1f %4.1f %4.1f", ((pid_0x637 >> 54) & 0x1FFu) * 0.1f, ((pid_0x637 >> 44) & 0x1FFu) * 0.1f, ((pid_0x637 >> 34) & 0x1FFu) * 0.1f);
        lcd.write(CHR_KW); lcd.print(F("h"));
        break;
      case 6: // range
        lcd.print(F("RNG    min. max."));
        lcd.setCursor(0, 1);
        lcdEx.printf("%4dkm", (pid_0x654 >> 12) & 0x3FFu); // VehicleAutonomy
        lcdEx.printf("%4d", (pid_0x638 >> 46) & 0x3FFu); // VehicleAutonomyMin
        lcd.write(CHR_KM);
        lcdEx.printf("%4d", (pid_0x638 >> 36) & 0x3FFu); // VehicleAutonomyMax
        lcd.write(CHR_KM);
        break;
      case 7: // instant consumption
        lcd.print(F("ICS   "));
        lcdEx.printf("Aux: %3dkW", (pid_0x638 >> 27) & 0x1Fu); // AuxInstantConsumption
        lcd.setCursor(1, 1);
        lcdEx.printf("Traction: %3dkW", ((pid_0x638 >> 56) & 0xFFu) - 80); // TractionInstantConsumption
        break;
      case 8: // wheels
        lcd.print(F("WHL F:"));
        // FrontLeftWheelPressure
        (((pid_0x673 >> 16) & 0xffu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 16) & 0xFFu) * 0.013725f) : lcd.print(F(" ----"));
        // FrontRightWheelPressure
        (((pid_0x673 >> 24) & 0xffu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 24) & 0xFFu) * 0.013725f) : lcd.print(F(" ----"));
        lcd.setCursor(0, 1);
        lcd.print(F("Bar R:"));
        // RearLeftWheelPressure
        (((pid_0x673 >> 32) & 0xffu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 32) & 0xFFu) * 0.013725f) : lcd.print(F(" ----"));
        // RearRightWheelPressure
        (((pid_0x673 >> 40) & 0xffu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 40) & 0xFFu) * 0.013725f) : lcd.print(F(" ----"));
        break;
      case 9: // 14v battery and DCDC converter
        lcd.print(F("14V     "));
        lcdEx.printf("%7.4fV", ((pid_0x6f8 >> 40) & 0xFFu) * 0.0625f); // BatteryVoltage
        lcd.setCursor(0, 1);
        lcdEx.printf("%4.1f%%", ((pid_0x1fd >> 56) & 0xFFu) * 0.390625f); // DCDCLoad
        lcdEx.printf(" %4fW", ((pid_0x1fd >> 56) & 0xFFu) * ((pid_0x1f6 >> 56) & 0x1Fu) * 0.390625f);
        lcdEx.printf(" %3fA", (((pid_0x1fd >> 56) & 0xFFu) * ((pid_0x1f6 >> 56) & 0x1Fu)) / ((pid_0x6f8 >> 40) & 0xFFu) * 6.25f);
        break;
      case 10: // temperatures
        lcd.print(F("T"));
        lcdEx.printf("%3f", (((pid_0x42a >> 48) & 0x3FFu) * 0.1f) - 40); // EvaporatorTempSetPoint
        lcdEx.printf("%3f", (((pid_0x430 >> 4) & 0x3FFu) * 0.1f) - 40); // HvBatteryEvaporatorSetpoint*
        lcdEx.printf("%3d", ((pid_0x42e >> 13) & 0x7Fu) - 40); // HVBatteryTemp
        lcdEx.printf("%3d", (pid_0x42a >> 40) & 0x7Fu); // WaterTempSetPoint
        lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5f) - 30); // CompTemperatureDischarge
        lcd.setCursor(0, 1);
        lcd.write(CHR_GRADCELSIUS);
        lcdEx.printf("%3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1f) - 40); // EvaporatorTempMeasure
        lcdEx.printf("%3f", (((pid_0x430 >> 14) & 0x3FFu) * 0.1f) - 40); // HvBatteryEvaporatorTempMeasure*
        lcdEx.printf("%3d", ((pid_0x432 >> 28) & 0x7Fu) - 40); // HVBattCondTempAverage
        lcdEx.printf("%3d", ((pid_0x5da >> 56) & 0xFFu) - 40); // EngineCoolantTemp
        lcdEx.printf("%3d", ((pid_0x656 >> 8) & 0xFFu) - 40); // ExternalTemp
        break;
    }
  }
}


uint64_t swap_uint64(uint64_t val)
{
  val = ((val << 8) & 0xFF00FF00FF00FF00ULL ) | ((val >> 8) & 0x00FF00FF00FF00FFULL );
  val = ((val << 16) & 0xFFFF0000FFFF0000ULL ) | ((val >> 16) & 0x0000FFFF0000FFFFULL );
  return (val << 32) | (val >> 32);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
