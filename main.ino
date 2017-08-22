#include <PrintEx.h>

#include <avr/pgmspace.h>

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

unsigned char flagRecv = 0;
unsigned char len = 0;

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
uint64_t      pid_0x5d1 = UINT64_MAX;
uint64_t      pid_0x5d7 = UINT64_MAX;
uint64_t      pid_0x5da = UINT64_MAX;
uint64_t      pid_0x5ee = UINT64_MAX;
uint64_t      pid_0x62d = UINT64_MAX;
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
uint64_t      pid_0x68c = UINT64_MAX;
uint64_t      pid_0x6f8 = UINT64_MAX;

PrintEx lcdEx = lcd;


const byte    DAY_BRIGHTNESS = UINT8_MAX;


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
}


void MCP2515_ISR()
{
  flagRecv = 1;
}


void btnRIGHTClick() {
  lcd.clear();
  if (pageno < 6) {
    pageno++;
  }
}

void btnUPClick() {
  //if (brightness < UINT8_MAX) brightness = brightness + 5;
  //analogWrite(3, brightness);
}

void btnDOWNClick() {
  //if (brightness > 0) brightness = brightness - 5;
  //analogWrite(3, brightness);
}

void btnLEFTClick() {
  lcd.clear();
  if (pageno > 0) {
    pageno--;
  }
}

void btnSELECTClick() {
  lcd.clear();
}


void loop()
{
  analogButtons.check();

  if (flagRecv) { // check if get data
    flagRecv = 0; // clear flag

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
          (pid_0x5ee & 0x10) ? analogWrite(3, DAY_BRIGHTNESS) : analogWrite(3, (pid_0x5ee >> 24) & 0xFFu);
          break;
        case 0x62d:
          pid_0x62d = swap_uint64(buf.ui64);
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
          break;
        case 0x656:
          pid_0x656 = swap_uint64(buf.ui64);
          break;
        case 0x657:
          pid_0x657 = swap_uint64(buf.ui64);
          break;
        case 0x658:
          pid_0x658 = swap_uint64(buf.ui64);
          break;
        case 0x65b:
          pid_0x65b = swap_uint64(buf.ui64);
          break;
        case 0x673:
          pid_0x673 = swap_uint64(buf.ui64);
          break;
        case 0x68c:
          pid_0x68c = swap_uint64(buf.ui64);
          break;
        case 0x6f8:
          pid_0x6f8 = swap_uint64(buf.ui64);
          break;
      }
    }
  }

  lcd.setCursor(0, 0);

  switch (pageno) {
    case 0: // drive
      // AvailableEnergy
      lcdEx.printf("%4.1fkWh", ((pid_0x427 >> 6) & 0x1FFu) * 0.1f);    
      // UserSOC
      lcd.setCursor(9, 0);      
      lcdEx.printf("%6.2f%%", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02f);
      // UserSOC BAR
      lbg.drawValue((pid_0x42e >> 51) & 0x1FFFu, 5000);
      break;
    case 1: // battery
      lcd.print(F("BAT"));
      // UserSOC
      lcd.setCursor(4, 0);
      lcdEx.printf("%6.2f%%", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02f);
      //  HVBatConditionningMode
      lcd.setCursor(11, 0);
      switch ((pid_0x432 >> 26) & 0x3u) {
        case 0:
          lcd.print(F("u"));
          break;
        case 1:
          lcd.print(F("C"));
          break;
        case 2:
          lcd.print(F("H"));
          break;
        case 3:
          lcd.print(F(" "));
          break;
      }
      // HVBatteryTemp
      lcd.setCursor(12, 0);
      lcdEx.printf("%3d", ((pid_0x42e >> 13) & 0x7Fu) - 40);
      lcd.write(CHR_GRADCELSIUS);
      // HVBatHealth
      lcd.setCursor(0, 1);
      lcdEx.printf("%3u%%", (pid_0x658 >> 24) & 0x7Fu);
      // AvailableEnergy
      lcd.setCursor(5, 1);
      lcdEx.printf("%4.1f", ((pid_0x427 >> 6) & 0x1FFu) * 0.1f);
      lcd.write(CHR_KW); lcd.print(F("h"));
      // HVNetworkVoltage
      lcd.setCursor(12, 1);
      lcdEx.printf("%3fV", ((pid_0x42e >> 29) & 0x3FFu) * 0.5f);
      break;
    case 2: // charge
      lcd.print(F("CHG"));
      // ChargeAvailable
      lcd.setCursor(4, 0);
      ((pid_0x427 >> 5) & 0x1u) ? lcd.write(CHR_TILDE) : lcd.print(F(" "));
      // ChargeInProgress
      //lcd.setCursor(5, 0);
      //((pid_0x658 >> 21) & 0x1u) ? lcd.print(F("=")) : lcd.print(F(" "));      
      // HVBatteryEnergyLevel
      lcd.setCursor(6, 0);
      lcdEx.printf("%3u%%", (pid_0x654 >> 32) & 0x7Fu);
      // ChargingPower
      lcd.setCursor(11, 0);
      lcdEx.printf("%4.1f", (pid_0x42e & 0xFFu) * 0.3f);
      lcd.write(CHR_KW);
      // MaxChargingNegotiatedCurrent
      lcd.setCursor(0, 1);
      lcdEx.printf("%2uA", (pid_0x42e >> 20) & 0x3Fu);      
      // BCBPowerMains
      lcd.setCursor(4, 1);
      lcdEx.printf("%5uW", ((pid_0x62d >> 35) & 0x1FFu) * 100);
      // AvailableChargingPower
      lcd.setCursor(11, 1);
      lcdEx.printf("%4.1f", ((pid_0x427 >> 16) & 0xFFu) * 0.3f);
      lcd.write(CHR_KW);
      break;
    case 3: // clima
      lcd.print(F("CLM"));
      // ClimLoopMode
      lcd.setCursor(4, 0);
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
      lcd.setCursor(11, 0);
      (((pid_0x42a >> 11) & 0x3u) == 1) ? lcd.print(F("P")) : lcd.print(F(" "));
      // ClimAirFlow
      lcd.setCursor(12, 0);
      lcdEx.printf("%3f%%", ((pid_0x42a >> 34) & 0x3Fu) * 2.12766f);
      // EvaporatorTempSetPoint
      lcd.setCursor(0, 1);
      lcdEx.printf("%3f", (((pid_0x42a >> 48) & 0x3FFu) * 0.1f) - 40);
      // EvaporatorTempMeasure
      lcd.setCursor(3, 1);
      lcdEx.printf("%3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1f) - 40);
      // CompTemperatureDischarge
      lcd.setCursor(6, 1);
      lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5f) - 30);
      lcd.write(CHR_GRADCELSIUS);
      // ClimAvailablePower
      lcd.setCursor(10, 1);
      lcdEx.printf("%5dW", (((pid_0x1fd >> 16) & 0xffu) * -25) + 5000);
      // EngineFanSpeedRequestPWM
      //lcdEx.printf("%3u%%", ((pid_0x42a >> 3) & 0x1fu) * 5);
      break;
    case 4: // wheels
      lcd.print(F("WHL F:"));
      lcd.setCursor(0, 1);
      lcd.print(F("Bar R:"));
      // FrontLeftWheelPressure
      lcd.setCursor(7, 0);
      (((pid_0x673 >> 16) & 0xffu) < UINT8_MAX) ? lcd.print(((pid_0x673 >> 16) & 0xFFu) * 0.013725f, 2) : lcd.print(F("----"));
      // FrontRightWheelPressure
      lcd.setCursor(12, 0);
      (((pid_0x673 >> 24) & 0xffu) < UINT8_MAX) ? lcd.print(((pid_0x673 >> 24) & 0xFFu) * 0.013725f, 2) : lcd.print(F("----"));
      // RearLeftWheelPressure
      lcd.setCursor(7, 1);
      (((pid_0x673 >> 32) & 0xffu) < UINT8_MAX) ? lcd.print(((pid_0x673 >> 32) & 0xFFu) * 0.013725f, 2) : lcd.print(F("----"));
      // RearRightWheelPressure
      lcd.setCursor(12, 1);
      (((pid_0x673 >> 40) & 0xffu) < UINT8_MAX) ? lcd.print(((pid_0x673 >> 40) & 0xFFu) * 0.013725f, 2) : lcd.print(F("----"));
      break;
    case 5: // total
      lcd.print(F("MIS"));
      lcd.setCursor(4, 0);
      // AvrgConsumptionSinceMissionStart
      ((pid_0x427 >> 48) < 0x3FFu) ? lcdEx.printf("%5.1f", ((pid_0x427 >> 48) & 0x3FFu) * 0.1f) : lcd.print(F("-----"));
      lcd.write(CHR_KW); lcd.print(F("h/100")); lcd.write(CHR_KM);
      // ConsumptionSinceMissionStart, RecoverySinceMissionStart, AuxConsumptionSinceMissionStart
      lcd.setCursor(0, 1);
      lcdEx.printf("%4.1f %3.1f %3.1fkWh", ((pid_0x637 >> 54) & 0x3FFu) * 0.1f, ((pid_0x637 >> 44) & 0x3FFu) * 0.1f, ((pid_0x637 >> 34) & 0x3FFu) * 0.1f);
      break;
    case 6: // temps
      lcd.print(F("T"));
      lcd.setCursor(1, 0);
      lcdEx.printf("%3f", (((pid_0x42a >> 48) & 0x3FFu) * 0.1f) - 40);
      lcdEx.printf("%3f", (((pid_0x430 >> 50) & 0x3FFu) * 0.1f) - 40);
      lcdEx.printf("%3d", ((pid_0x42e >> 13) & 0x7Fu) - 40);
      lcdEx.printf("%3d", (pid_0x42a >> 40) & 0x7Fu);
      lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5f) - 30);
      lcd.setCursor(0, 1);
      lcd.write(CHR_GRADCELSIUS);
      lcd.setCursor(1, 1);
      lcdEx.printf("%3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1f) - 40);
      lcdEx.printf("%3f", (((pid_0x430 >> 40) & 0x3FFu) * 0.1f) - 40);
      lcdEx.printf("%3d", ((pid_0x432 >> 28) & 0x7Fu) - 40);
      lcdEx.printf("%3d", ((pid_0x5da >> 56) & 0xFFu) - 40);
      lcdEx.printf("%3d", ((pid_0x656 >> 8) & 0xFFu) - 40);
      break;
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
