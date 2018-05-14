/*********************************************************************************************************
  Renault ZOE owners assistant display
*********************************************************************************************************/

#include <StopWatch.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <PrintEx.h>
#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
#include <mcp_can.h>
#include <AnalogButtons.h>

//#include <OneWire.h>
//#include <DallasTemperature.h>


#define CAN_INT 2
#define SPI_CS_PIN 10
#define ANALOG_BUTTON_PIN A0
//#define ONE_WIRE_BUS A1


enum screens : byte {
  SCRN_TIM,     // Timers
  SCRN_NRG,     // Energy
  SCRN_CHG,     // Charging
  SCRN_BAT,     // HV Battery
  SCRN_PCT,     // Percent
  SCRN_CLM,     // Clima
  SCRN_MIS,     // Mission
  SCRN_RNG,     // Range
  SCRN_ICS,     // Instant Consumption
  SCRN_WHL,     // Wheels
  SCRN_14V,     // 14V Network
  SCRN_TMP,     // Temperatures
  SCRN_PID,     // Onboard OBD PID Decoder
  SCRN_DBG      // Debugging Screen
};

enum timer_mode : byte {
  TM_CHARGE,    // charging
  TM_MAINS,     // mains on, ready to charge
  TM_PLUGGED,   // plug connected
  TM_DRIVING    // speed > 0
};


union union64 {
  unsigned char uc[8];   // 8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  byte           b[8];   // 8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  uint8_t      ui8[8];   // 8 bit (1 byte) 0 bis 255 / 0 bis (2^8)-1)
  uint64_t       ui64;   // 64 bit (4 byte) 0 to 4,294,967,295 / 0 bis (2^64) - 1)
};


const uint64_t PID_INIT_VALUE = 0;
const byte DAY_BRIGHTNESS = UINT8_MAX;
const screens PAGE_LAST = SCRN_PID;
const char timerModeChar[] = "CMPD";

//define custom LCD CGRAM char locations
const byte CHR_TILDE       = 0x00;
//note: chars 0x01-0x04 are occupied by LcdBarGraph lib
const byte CHR_KM          = 0x05;
const byte CHR_KW          = 0x06;
const byte CHR_GRADCELSIUS = 0x07;


StopWatch sw(StopWatch::SECONDS);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LcdBarGraph lbg(&lcd, 16, 0, 1);
PrintEx lcdEx = lcd;
MCP_CAN CAN(SPI_CS_PIN);
Button btnRIGHT = Button(0, &btnRIGHTClick);
Button btnUP = Button(99, &btnUPClick, &btnUPHold, 500, 125);
Button btnDOWN = Button(255, &btnDOWNClick, &btnDOWNHold, 500, 125);
Button btnLEFT = Button(407, &btnLEFTClick);
Button btnSELECT = Button(637, &btnSELECTClick, &btnSELECTHold, 2000, UINT16_MAX);
AnalogButtons analogButtons = AnalogButtons(ANALOG_BUTTON_PIN, INPUT);
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
//DeviceAddress tempDeviceAddress;


//custom LCD CGRAM bitmaps
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

//internal pid buffers
uint64_t pid_0x1f6 = PID_INIT_VALUE;
uint64_t pid_0x1fd = PID_INIT_VALUE;
uint64_t pid_0x212 = PID_INIT_VALUE; //-
uint64_t pid_0x391 = PID_INIT_VALUE;
uint64_t pid_0x427 = PID_INIT_VALUE;
uint64_t pid_0x42a = PID_INIT_VALUE;
uint64_t pid_0x42e = PID_INIT_VALUE;
uint64_t pid_0x430 = PID_INIT_VALUE;
uint64_t pid_0x432 = PID_INIT_VALUE;
uint64_t pid_0x5ee = PID_INIT_VALUE;
uint64_t pid_0x5d7 = PID_INIT_VALUE;
uint64_t pid_0x62d = PID_INIT_VALUE;
uint64_t pid_0x637 = PID_INIT_VALUE;
uint64_t pid_0x638 = PID_INIT_VALUE;
uint64_t pid_0x654 = PID_INIT_VALUE;
uint64_t pid_0x656 = PID_INIT_VALUE;
uint64_t pid_0x658 = PID_INIT_VALUE;
uint64_t pid_0x673 = PID_INIT_VALUE;
uint64_t pid_0x68c = PID_INIT_VALUE;
uint64_t pid_0x6f8 = PID_INIT_VALUE;

//user PID decoder buffer
uint64_t pid_0xPID = PID_INIT_VALUE;

bool timerEdit = false;
bool priceEdit = false;
bool pidnoEdit = false;
bool freezePID = false;
bool singleByteMode = false;
bool screenRefresh = false;

byte pageno = 0;
byte byteno = 0;
byte timerMode = TM_CHARGE;

//isr var
static volatile byte intCount = 0;

unsigned int LocalTime = 0;
unsigned int ChargeRemainingTime = 0;
unsigned int ChargeBeginTime = 0;
unsigned int ChargeEndTime = 0;
unsigned int selectedPID = 0x69f;

unsigned long energy = 0;

float ChargeBeginKwh = 0.0;
float ChargeEndKwh = 0.0;
float priceKwh = 0.0;
float temperature = 0.0;


void setup()
{
  //Initialize display
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();

  lcd.print(F("ZOE"));

  //Initialize CAN shield
  CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  pinMode(CAN_INT, INPUT);

  //Setup CAN PID filters
  //there are 2 mask in mcp2515, you need to set both of them
  //mask0
  CAN.init_Mask(0, 0, 0x0c000000);
  //filter0
  CAN.init_Filt(0, 0, 0x04ff0000); // 0x400 - 0x7ff
  CAN.init_Filt(1, 0, 0x04ff0000);  
  //mask1
  CAN.init_Mask(1, 0, 0x07ff0000);
  //filter1
  CAN.init_Filt(2, 0, 0x03910000);
  CAN.init_Filt(3, 0, 0x02120000); // currently not requiered
  CAN.init_Filt(4, 0, 0x01fd0000); // t = 100 ms
  CAN.init_Filt(5, 0, 0x01f60000); // t = 10 ms

  CAN.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

  //Button assignments
  analogButtons.add(btnRIGHT);
  analogButtons.add(btnUP);
  analogButtons.add(btnDOWN);
  analogButtons.add(btnLEFT);
  analogButtons.add(btnSELECT);

  //adjust LCD Brightness using OC2B PWM (Timer2)
  pinMode(3, OUTPUT);
  analogWrite(3, DAY_BRIGHTNESS);

  //Load custom character bitmaps
  lcd.createChar(0, char_tilde);
  lcd.createChar(5, char_km);
  lcd.createChar(6, char_kW);
  lcd.createChar(7, char_gradC);

  //Read user-stored Page number from EEPROM
  (EEPROM.read(0x00) <= PAGE_LAST) ? pageno = EEPROM.read(0x00) : pageno = 0;
  EEPROM.get(0x10, priceKwh);
  EEPROM.get(0x20, energy);
  EEPROM.get(0x30, ChargeBeginTime);
  EEPROM.get(0x40, ChargeEndTime);
  EEPROM.get(0x50, ChargeBeginKwh);
  EEPROM.get(0x60, ChargeEndKwh);
  EEPROM.get(0x70, selectedPID);
  EEPROM.get(0x80, timerMode);
  
  if (isnan(priceKwh) || isnan(energy) || isnan(ChargeBeginKwh) || isnan(ChargeEndKwh)) {
    priceKwh = 0.0;
    energy = 0;
    ChargeBeginKwh = 0.0;
    ChargeEndKwh = 0.0;
  }

  //Initialize internal temperature sensor
  //sensors.begin();
  //sensors.getAddress(tempDeviceAddress, 0);
  //sensors.setResolution(tempDeviceAddress, 9);
  //sensors.setWaitForConversion(false);
  //sensors.requestTemperatures();
  ////lastTempRequest = millis();

  //Initialize display refresh timer (Timer1)
  Timer1.initialize(250000);
  Timer1.attachInterrupt(LCD_ISR);

  //Initialize stopwatch
  sw.reset();
}


void LCD_ISR()
{
  intCount++;
}


void btnRIGHTClick()
{
  lcd.clear();
  if (pageno < PAGE_LAST) pageno++;
  else pageno = 0;
  screenRefresh = true;
}


void btnUPClick()
{
  switch (pageno) {
    case SCRN_TIM:
      if (timerEdit) timerMode = constrain(timerMode + 1, TM_CHARGE, TM_DRIVING);
      else sw.start();
      break;
    case SCRN_NRG:
      if (priceEdit) priceKwh = constrain(priceKwh + 0.0001, 0.0, 9.9999);
      break;
    case SCRN_PID:
      if (pidnoEdit) {
        selectedPID = constrain(selectedPID + 0x001, 0x1f6, 0x7ff);
        pid_0xPID = PID_INIT_VALUE;
      }
      else freezePID = !freezePID;
      break;
  }
  screenRefresh = true;
}


void btnUPHold()
{
  switch (pageno) {
    case SCRN_NRG:
      if (priceEdit) priceKwh = constrain(priceKwh + 0.001, 0.0, 9.9999);
      break;
    case SCRN_PID:
      if (pidnoEdit) {
        selectedPID = constrain(selectedPID + 0x010, 0x1f6, 0x7ff);
        pid_0xPID = PID_INIT_VALUE;
      }
      break;
  }
  screenRefresh = true;
}


void btnDOWNClick()
{
  switch (pageno) {
    case SCRN_TIM:
      if (timerEdit) timerMode = constrain(timerMode - 1, TM_CHARGE, TM_DRIVING);
      else sw.stop();
      break;
    case SCRN_NRG:
      if (priceEdit) priceKwh = constrain(priceKwh - 0.0001, 0.0, 9.9999);
      break;
    case SCRN_PID:
      if (pidnoEdit) {
        selectedPID = constrain(selectedPID - 0x001, 0x1f6, 0x7ff);
        pid_0xPID = PID_INIT_VALUE;
      }
      else {
        if (singleByteMode) byteno = (byteno + 1) & 0x7;
        else singleByteMode = true;
      }
      break;
  }
  screenRefresh = true;
}


void btnDOWNHold()
{
  switch (pageno) {
    case SCRN_TIM:
      sw.stop();
      sw.reset();
      ChargeBeginTime = 0;
      ChargeEndTime = 0;
      break;
    case SCRN_NRG:
      if (priceEdit) priceKwh = constrain(priceKwh - 0.001, 0.0, 9.9999);
      else {
        energy = 0;
        ChargeBeginKwh = ((pid_0x427 >> 6) & 0x1FFu) * 0.1;;
        ChargeEndKwh = ChargeBeginKwh;
      }
      break;
    case SCRN_PID:
      if (pidnoEdit) {
        selectedPID = constrain(selectedPID - 0x010, 0x1f6, 0x7ff);
        pid_0xPID = PID_INIT_VALUE;
      }
      else singleByteMode = false;
      break;
  }
  screenRefresh = true;
}


void btnLEFTClick()
{
  lcd.clear();
  if (pageno > 0) pageno--;
  else pageno = PAGE_LAST;
  screenRefresh = true;
}


void btnSELECTClick()
{
  switch (pageno) {
    case SCRN_TIM: timerEdit = !timerEdit; break;
    case SCRN_NRG: priceEdit = !priceEdit; break;
    case SCRN_PID: pidnoEdit = !pidnoEdit; break;
  }
  screenRefresh = true;
}


void btnSELECTHold()
{
  saveState();
  EEPROM.update(0x00, pageno);
}


void saveState()
{
  lcd.clear();
  lcd.home();
  lcd.print(F("Saving state..."));
  EEPROM.put(0x10, priceKwh);
  EEPROM.put(0x20, energy);
  EEPROM.put(0x30, ChargeBeginTime);
  EEPROM.put(0x40, ChargeEndTime);
  EEPROM.put(0x50, ChargeBeginKwh);
  EEPROM.put(0x60, ChargeEndKwh);
  EEPROM.put(0x70, selectedPID);
  EEPROM.put(0x80, timerMode);
  //lcd.clear();
}


void loop()
{
  union64 buf;
  
  static bool lastCharging = false;
  static bool lastMains = false;
  static bool lastPlugged = true;
  static bool lastDriving = false;

  static float energymeter = 0.0;

  //perf counter vars
  static unsigned long startCycle = 0;
  static unsigned long lastCycle = 0;
  static unsigned long minCycle = 0;
  static unsigned long maxCycle = 0;
  static unsigned long countCycle = 0;

  //car status
  static bool isPlugged = false;
  static bool isMains = false;
  static bool isCharging = false;
  static bool isDriving = false;

  //pid decoder timing vars
  static unsigned long lastPidSeen = 0;
  static unsigned long lastPidCycleDuration = 0;
  //static unsigned long lastTempRequest = 0;

  //perf counter
  countCycle++;
  minCycle = min(minCycle, lastCycle);
  maxCycle = max(maxCycle, lastCycle);
  startCycle = millis();

  //CAN receiver
  if(!digitalRead(CAN_INT)) { //while (CAN_MSGAVAIL == CAN.checkReceive())
    long unsigned int rxId;
    byte len = 0;
    buf.ui64 = PID_INIT_VALUE;
    CAN.readMsgBuf(&rxId, &len, buf.b);
    //user pid decoder
    if (rxId == selectedPID) {
      lastPidCycleDuration = millis() - lastPidSeen;
      lastPidSeen = millis();
      if (!freezePID) pid_0xPID = swap_uint64(buf.ui64);
    }
    switch (rxId) {
      case 0x1f6: pid_0x1f6 = swap_uint64(buf.ui64); break;
      case 0x1fd: pid_0x1fd = swap_uint64(buf.ui64); break;
      case 0x212: pid_0x212 = swap_uint64(buf.ui64); break;
      case 0x391: pid_0x391 = swap_uint64(buf.ui64); break;
      case 0x427: pid_0x427 = swap_uint64(buf.ui64);
        isMains = pid_0x427 & 0x20u;  // ChargeAvailable
        if (isMains != lastMains) {
          if (isMains) {
            if (timerMode == TM_MAINS) sw.start();
          } else {
            if (timerMode == TM_MAINS) sw.stop();
          }
        }
        lastMains = isMains;
        break;
      case 0x42a: pid_0x42a = swap_uint64(buf.ui64); break;
      case 0x42e: pid_0x42e = swap_uint64(buf.ui64); break;
      case 0x430: pid_0x430 = swap_uint64(buf.ui64); break;
      case 0x432: pid_0x432 = swap_uint64(buf.ui64); break;
      case 0x5d7: pid_0x5d7 = swap_uint64(buf.ui64);
        isDriving = (pid_0x5d7 >> 48) & 0xFFFFu;  // VehicleSpeed
        if (isDriving != lastDriving) {
          if (isDriving) {
            if (timerMode == TM_DRIVING) sw.start();
          } else {
            if (timerMode == TM_DRIVING) sw.stop();
          }
        }
        lastDriving = isDriving;
        break;
      case 0x5ee: pid_0x5ee = swap_uint64(buf.ui64);
        // LightSensorStatus, NightRheostatedLightMaxPercent
        (pid_0x5ee & 0x10u) ? analogWrite(3, DAY_BRIGHTNESS) : analogWrite(3, (pid_0x5ee >> 24) & 0xFFu);
        break;
      case 0x62d: pid_0x62d = swap_uint64(buf.ui64); break;
      case 0x637: pid_0x637 = swap_uint64(buf.ui64); break;
      case 0x638: pid_0x638 = swap_uint64(buf.ui64); break;
      case 0x654: pid_0x654 = swap_uint64(buf.ui64);
        isPlugged = (pid_0x654 >> 61) & 0x1u; // ChargingPlugConnected
        ChargeRemainingTime = (((pid_0x654 >> 22) & 0x3ffu) < 0x3ff) ? (pid_0x654 >> 22) & 0x3ffu : 0;
        if (isPlugged != lastPlugged) { // EVENT plugged-in state changed
          if (isPlugged) {
            sw.stop();
            sw.reset();
            ChargeBeginTime = 0;
            ChargeEndTime = 0;
            energy = 0;
            ChargeBeginKwh = ((pid_0x427 >> 6) & 0x1FFu) * 0.1;;
            ChargeEndKwh = ChargeBeginKwh;
            if (timerMode == TM_PLUGGED) sw.start();
          } else {
            if (timerMode == TM_PLUGGED) sw.stop();
          }
        }
        lastPlugged = isPlugged;
        break;
      case 0x656: pid_0x656 = swap_uint64(buf.ui64); break;
      case 0x658: pid_0x658 = swap_uint64(buf.ui64);
        isCharging = pid_0x658 & 0x200000u; // ChargeInProgress
        if (isCharging) { // STATE charge in progress
          ChargeEndTime = LocalTime + ChargeRemainingTime;
          ChargeEndKwh = ((pid_0x427 >> 6) & 0x1FFu) * 0.1;
        }
        if (isCharging != lastCharging) { // EVENT charge state changed
          if (isCharging) { // EVENT started charging
            if (timerMode == TM_CHARGE) sw.start();
            ChargeBeginTime = LocalTime;
          } else { // EVENT stopped charging
            if (timerMode == TM_CHARGE) sw.stop();
            ChargeEndTime = LocalTime;
            ChargeEndKwh = ((pid_0x427 >> 6) & 0x1FFu) * 0.1;
            saveState();
          }
        }
        lastCharging = isCharging;
        break;
      case 0x673: pid_0x673 = swap_uint64(buf.ui64); break;
      case 0x68c: pid_0x68c = swap_uint64(buf.ui64);
        LocalTime = (pid_0x68c >> 32) & 0x7ffu;
        break;
      case 0x6f8: pid_0x6f8 = swap_uint64(buf.ui64); break;
    }
  }

  //read buttons
  analogButtons.check();

  //read local temperature sensor
  //  if (millis() - lastTempRequest >= 7500) {
  //    temperature = sensors.getTempCByIndex(0);
  //    sensors.requestTemperatures();
  //    lastTempRequest = millis();
  //  }

  if (intCount | screenRefresh) {
    //energy meter
    while (intCount) {
      intCount--;
      energy += (pid_0x62d >> 35) & 0x1FFu; // BCBPowerMains 1h/250ms = 14400
      energymeter = energy / 144000.0;
    }

    //display screens
    screenRefresh = false;
    lcd.home();

    switch (pageno) {
      case SCRN_TIM: // timers
        lcd.print(F("TIM  "));
        lcdEx.printf("%02u:%02u", (ChargeBeginTime / 60u) % 24u, ChargeBeginTime % 60u);
        lcdEx.printf("-%02u:%02u", (ChargeEndTime / 60u) % 24u, ChargeEndTime % 60u);
        lcd.setCursor(0, 1);
        lcd.print(timerModeChar[timerMode]);
        lcd.print(sw.isRunning() ? F("*") : F("="));
        lcdEx.printf("%02u", (sw.value() / 3600u) % 24u);
        lcdEx.printf(":%02u", (sw.value() / 60u) % 60u);
        lcdEx.printf(":%02u", sw.value() % 60u);
        lcdEx.printf(" %02u:%02u", LocalTime / 60u, LocalTime % 60u);
        if (timerEdit) {
          lcd.setCursor(0, 1);
          lcd.blink();
        } else lcd.noBlink();
        break;
      case SCRN_NRG: // energy
        lcd.print(F("NRG "));
        lcdEx.printf("%4.1f", ChargeEndKwh - ChargeBeginKwh);
        lcdEx.printf("%6.2f", energymeter); lcd.write(CHR_KW); lcd.print(F("h"));
        lcd.setCursor(0, 1);
        lcdEx.printf("%6.4f %6.2fEUR", priceKwh, energymeter * priceKwh); //lcd.write(CHR_EURO);
        if (priceEdit) {
          lcd.setCursor(5, 1);
          lcd.blink();
        } else lcd.noBlink();
        break;
      case SCRN_CHG: // charge
        lcd.print(F("CHG "));
        ((pid_0x427 >> 5) & 0x1u) ? lcd.write(CHR_TILDE) : lcd.print(F(" ")); // ChargeAvailable
        //((pid_0x658 >> 21) & 0x1u) ? lcd.print(F("=")) : lcd.print(F(" ")); // ChargeInProgress
        lcdEx.printf(" %3u%%", (pid_0x654 >> 32) & 0x7Fu); // HVBatteryEnergyLevel
        lcdEx.printf(" %4.1f", (pid_0x42e & 0xFFu) * 0.3); // ChargingPower
        lcd.write(CHR_KW);
        lcd.setCursor(0, 1);
        lcdEx.printf("%2uA", (pid_0x42e >> 20) & 0x3Fu); // MaxChargingNegotiatedCurrent
        lcdEx.printf(" %5uW", ((pid_0x62d >> 35) & 0x1FFu) * 100); // BCBPowerMains
        lcdEx.printf(" %4.1f", ((pid_0x427 >> 16) & 0xFFu) * 0.3); // AvailableChargingPower
        lcd.write(CHR_KW);
        break;
      case SCRN_BAT: // battery
        lcd.print(F("BAT "));
        lcdEx.printf("%6.2f%% ", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02); // UserSOC
        switch ((pid_0x432 >> 26) & 0x3u) { // HVBatConditionningMode
          case 1:  lcd.print(F("C")); break;
          case 2:  lcd.print(F("H")); break;
          default: lcd.print(F(" ")); break;
        }
        lcdEx.printf("%2d", ((pid_0x42e >> 13) & 0x7Fu) - 40); // HVBatteryTemp
        lcd.write(CHR_GRADCELSIUS);
        lcd.setCursor(0, 1);
        lcdEx.printf("%3u%%", (pid_0x658 >> 24) & 0x7Fu); // HVBatHealth
        lcdEx.printf(" %4.1f", ((pid_0x427 >> 6) & 0x1FFu) * 0.1); // AvailableEnergy
        lcd.write(CHR_KW); lcd.print(F("h"));
        lcdEx.printf(" %3fV", ((pid_0x42e >> 29) & 0x3FFu) * 0.5); // HVNetworkVoltage
        break;
      case SCRN_PCT: // battery bar
        lcdEx.printf("%4.1fkWh", ((pid_0x427 >> 6) & 0x1FFu) * 0.1); // AvailableEnergy
        lcdEx.printf("  %6.2f%%", ((pid_0x42e >> 51) & 0x1FFFu) * 0.02); // UserSOC
        lbg.drawValue((pid_0x42e >> 51) & 0x1FFFu, 5000); // UserSOC BAR
        break;
      case SCRN_CLM: // clima
        lcd.print(F("CLM "));
        // ClimLoopMode
        switch ((pid_0x42a >> 13) & 0x7u) {
          case 0:  lcd.print(F("n/a   ")); break;
          case 1:  lcd.print(F("Cool  ")); break;
          case 2:  lcd.print(F("De-Ice")); break;
          case 4:  lcd.print(F("Heat  ")); break;
          case 6:  lcd.print(F("Demist")); break;
          case 7:  lcd.print(F("Idle  ")); break;
          default: lcd.print(F("      ")); break;
        }
        (((pid_0x391 >> 24) & 0xFu) > 0) ? lcd.print(F("P")) : lcd.print(F(" ")); // PTCNumberThermalRequest
        //lcdEx.printf("%3d", (pid_0x391 >> 24) & 0xFu); // PTCNumberThermalRequest
        lcdEx.printf(" %3f%%", ((pid_0x42a >> 34) & 0x3Fu) * 2.12766); // ClimAirFlow
        lcd.setCursor(0, 1);
        lcdEx.printf("%3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1) - 40); // EvaporatorTempMeasure
        lcdEx.printf("%3f", (((pid_0x430 >> 14) & 0x3FFu) * 0.1) - 40); // HvBatteryEvaporatorTempMeasure*
        lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5) - 30); // CompTemperatureDischarge
        lcd.write(CHR_GRADCELSIUS);
        lcdEx.printf("%5dW", (((pid_0x1fd >> 16) & 0xFFu) * -25) + 5000); // ClimAvailablePower
        break;
      case SCRN_MIS: // mission
        lcd.print(F("MIS   "));
        lcd.write(byte(246));
        // ConsumptionSinceMissionStart + AuxConsumptionSinceMissionStart - RecoverySinceMissionStart = TotalConsumptionSinceMissionStart
        lcdEx.printf(":%5.1fkWh", (((int)((pid_0x637 >> 54) & 0x3FFu) + (int)((pid_0x637 >> 34) & 0x3FFu)) - (int)((pid_0x637 >> 44) & 0x3FFu)) * 0.1);
        lcd.setCursor(0, 1);
        // ConsumptionSinceMissionStart, RecoverySinceMissionStart, AuxConsumptionSinceMissionStart
        lcdEx.printf("%4.1f %4.1f %4.1f", ((pid_0x637 >> 54) & 0x1FFu) * 0.1, ((pid_0x637 >> 44) & 0x1FFu) * 0.1, ((pid_0x637 >> 34) & 0x1FFu) * 0.1);
        lcd.write(CHR_KW); lcd.print(F("h"));
        break;
      case SCRN_RNG: // range
        lcd.print(F("RNG    min. max."));
        lcd.setCursor(0, 1);
        lcdEx.printf("%4dkm", (pid_0x654 >> 12) & 0x3FFu); // VehicleAutonomy
        lcdEx.printf("%4d", (pid_0x638 >> 46) & 0x3FFu); // VehicleAutonomyMin
        lcd.write(CHR_KM);
        lcdEx.printf("%4d", (pid_0x638 >> 36) & 0x3FFu); // VehicleAutonomyMax
        lcd.write(CHR_KM);
        break;
      case SCRN_ICS: // instant consumption
        lcd.print(F("ICS   "));
        lcdEx.printf("Aux: %3dkW", (pid_0x638 >> 27) & 0x1Fu); // AuxInstantConsumption
        lcd.setCursor(1, 1);
        lcdEx.printf("Traction: %3dkW", ((pid_0x638 >> 56) & 0xFFu) - 80); // TractionInstantConsumption
        break;
      case SCRN_WHL: // wheels
        lcd.print(F("WHL F:"));
        // FrontLeftWheelPressure
        (((pid_0x673 >> 16) & 0xFFu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 16) & 0xFFu) * 0.013725) : lcd.print(F(" ----"));
        // FrontRightWheelPressure
        (((pid_0x673 >> 24) & 0xFFu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 24) & 0xFFu) * 0.013725) : lcd.print(F(" ----"));
        lcd.setCursor(0, 1);
        lcd.print(F("Bar R:"));
        // RearLeftWheelPressure
        (((pid_0x673 >> 32) & 0xFFu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 32) & 0xFFu) * 0.013725) : lcd.print(F(" ----"));
        // RearRightWheelPressure
        (((pid_0x673 >> 40) & 0xFFu) < UINT8_MAX) ? lcdEx.printf(" %4.2f", ((pid_0x673 >> 40) & 0xFFu) * 0.013725) : lcd.print(F(" ----"));
        break;
      case SCRN_14V: // 14v battery and DCDC converter
        lcd.print(F("14V     "));
        lcdEx.printf("%7.4fV", ((pid_0x6f8 >> 40) & 0xFFu) * 0.0625); // BatteryVoltage
        lcd.setCursor(0, 1);
        if (((pid_0x1fd >> 56) & 0xFFu) < 0xFEu) {
          lcdEx.printf("%4.1f%%", ((pid_0x1fd >> 56) & 0xFFu) * 0.390625); // DCDCLoad
          lcdEx.printf(" %4fW", ((pid_0x1fd >> 56) & 0xFFu) * ((pid_0x1f6 >> 56) & 0x1Fu) * 0.390625);
          lcdEx.printf(" %3fA", (((pid_0x1fd >> 56) & 0xFFu) * ((pid_0x1f6 >> 56) & 0x1Fu)) / (float)((pid_0x6f8 >> 40) & 0xFFu) * 6.25);
        } else {
          lcd.print(F("-----    0W   0A")); // DCDC converter off
        }
        break;
      case SCRN_TMP: // temperatures
        lcd.print(F("TMP "));
        lcdEx.printf("%3f", (((pid_0x42a >> 48) & 0x3FFu) * 0.1) - 40); // EvaporatorTempSetPoint
        lcdEx.printf("%3f", (((pid_0x430 >> 4) & 0x3FFu) * 0.1) - 40); // HvBatteryEvaporatorSetpoint*
        lcdEx.printf("%3d", ((pid_0x42e >> 13) & 0x7Fu) - 40); // HVBatteryTemp
        //lcdEx.printf("%3d", (pid_0x42a >> 40) & 0x7Fu); // WaterTempSetPoint
        lcdEx.printf("%3d", ((pid_0x656 >> 8) & 0xFFu) - 40); // ExternalTemp
        lcd.setCursor(0, 1);
        lcd.write(CHR_GRADCELSIUS);
        lcdEx.printf("   %3f", (((pid_0x42a >> 24) & 0x3FFu) * 0.1) - 40); // EvaporatorTempMeasure
        lcdEx.printf("%3f", (((pid_0x430 >> 14) & 0x3FFu) * 0.1) - 40); // HvBatteryEvaporatorTempMeasure*
        lcdEx.printf("%3d", ((pid_0x432 >> 28) & 0x7Fu) - 40); // HVBattCondTempAverage
        lcdEx.printf("%3f", (((pid_0x430 >> 30) & 0x3FFu) * 0.5) - 30); // CompTemperatureDischarge
        //lcdEx.printf("%3d", ((pid_0x5da >> 56) & 0xFFu) - 40); // EngineCoolantTemp
        //lcdEx.printf("%3f", temperature); // InternalTemp (DS18x20 Sensor)
        break;
      case SCRN_PID: // pid decoder
        lcd.print(F("PID 0x"));
        if (selectedPID < 0x100) lcd.print(F("0"));
        if (selectedPID < 0x010) lcd.print(F("0"));
        lcd.print(selectedPID, HEX);
        freezePID ? lcd.print(F("*")) : lcd.print(F(" "));
        lcdEx.printf("%4dms", lastPidCycleDuration);
        lcd.setCursor(0, 1);
        buf.ui64 = pid_0xPID;
        if (singleByteMode) {
          lcd.print(byteno); lcdEx.printf(":%02d|", byteno * 8 + 7);
          for (byte mask = 0x80; mask; mask >>= 1) {
            lcd.print(mask & buf.ui8[byteno] ? "1" : "0");
          }
          lcdEx.printf("|%02d", byteno * 8);
        } else {
          byte i; i = 8; while (i-- > 0) {
            if (buf.ui8[i] < 0x10) lcd.print(F("0"));
            lcd.print(buf.ui8[i], HEX);
          }
        }
        if (pidnoEdit) {
          lcd.setCursor(8, 0);
          lcd.blink();
        } else lcd.noBlink();
        break;
      case SCRN_DBG: // debug/test/performance
        lcd.print(F("DBG "));
        lcdEx.printf("%2d", minCycle); // Timing
        lcdEx.printf("%4d", maxCycle); // Timing
        lcdEx.printf("%4dms", lastCycle); // Timing
        lcd.setCursor(0, 1);
        lcdEx.printf("%6d", countCycle); // Timing
        break;
    }
    //perfmon cycle reset
    minCycle = UINT32_MAX;
    maxCycle = 0;
    countCycle = 0;
  }
  lastCycle = millis() - startCycle;
}


uint64_t swap_uint64(uint64_t val)
{
  val = ((val << 8) & 0xFF00FF00FF00FF00ull) | ((val >> 8) & 0x00FF00FF00FF00FFull);
  val = ((val << 16) & 0xFFFF0000FFFF0000ull) | ((val >> 16) & 0x0000FFFF0000FFFFull);
  return (val << 32) | (val >> 32);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
