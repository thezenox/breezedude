#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <BMP280.h>
#include <SPL06.h>
#include <Adafruit_BMP3XX.h>
#include <LibPrintf.h>
#include <RadioLib.h>
#include <SdFat.h>
#include <SAMD_InternalFlash.h>
#include <TinyGPS++.h>
#include <ArduinoUniqueID.h>

#include "ff.h"
#include "diskio.h"
#include "logging.h"
#include "sleep.h"
#include "types.h"
#include "display.h"
#include "hist.h"

 #define HAS_HEATER // support for Heater (V1.x)

// Todo list:
// * Read bootloader version
// * detect bad readings
//  * wind > gust --> ignore
//  * gust too high (water in sensor) --> ignore
//  * Sensor frozen?
// * extend send interval on low battery

// Hardware:
// move I2C conntor (colission with antenna)
// add p-fet to power off WS80 (if battery is empty?)
// Simple UVP for battery?
// OUTPUT in deepsleep prüfen (Pfet ws80)

// https://github.com/adafruit/Adafruit_TinyUSB_Arduino
// https://github.com/gereic/GXAirCom
// https://github.com/adafruit/ArduinoCore-samd
// https://github.com/Mollayo/SAMD_InternalFlash
// https://github.com/Microsoft/uf2

//Modifications:
// SAMD_InternalFlash.cpp:  
// use last 40kb of flash as FAT12 disk for settings file
//    _flash_address = (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)

// Improvements
// LDO: 
// S-1313D33 1,5µA (200mA max)
// HT7533-1 2.5µA (100mA max)?

//Power consumption:
// Deepsleep 120µA (without WS80 sensor, davis speed and dir open)
// avg @40s send interval: 650µA (with davis 6410)
// avg @40s send interval: 1.65mA (with WS80)

#define FANET_VENDOR_ID 0xBD
#define VBATT_LOW 3.35 // Volt

Adafruit_USBD_MSC usb_msc;
#define DISK_BLOCK_SIZE 512 // Block size in bytes for tinyUSB flash drive. Should be always 512
#define BUFFERSIZE 1024 // size of linebuffer

InternalFlash my_internal_storage;
Adafruit_FlashTransport_InternalFlash flashTransport(&my_internal_storage);
Adafruit_InternalFlash_Wrapper flash(&flashTransport);
FatFileSystem fatfs;
TinyGPSPlus tinyGps;

#define BROADCAST_INTERVAL 40*1000 // dafault value,  overwritten by settingfile

#define DEBUGSER Serial1
#define WS80_UART Serial1
#define GPS_SERIAL Serial1 // if no WS80 is connected a GPS receiver can be used. Mainly for OGN range/coverage check

// https://github.com/adafruit/ArduinoCore-samd/blob/master/variants/itsybitsy_m0/variant.cpp
// 0 { PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
// 1 { PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

// SERCOM 4 - SPI RFM95W LoRa
#define PIN_LORA_MISO MISO  // 28 PA12  - SERCOM2/PAD[0] | alt SERCOM4/PAD[0]
#define PIN_LORA_MOSI MOSI  // 29 PB10  -                | alt SERCOM4/PAD[2]
#define PIN_LORA_SCK SCK    // 30 PB11  -                | alt SERCOM4/PAD[3]

#define PIN_LORA_CS 5    // PA15 - SERCOM2/PAD[3] | alt SERCOM4/PAD[3]
#define PIN_LORA_RESET 2 // PA14 - SERCOM2/PAD[2] | alt SERCOM4/PAD[2]
#define PIN_LORA_DIO0 11 // PA16 - SERCOM1/PAD[0] | alt SERCOM3/PAD[0]
#define PIN_LORA_DIO1 10 // PA18 - SERCOM1/PAD[2] | alt SERCOM3/PAD[2]
#define PIN_LORA_DIO2 12 // PA19 - SERCOM1/PAD[3] | alt SERCOM3/PAD[3]
#define PIN_LORA_DIO3 3  // PA09 - SERCOM0/PAD[1], ADC
#define PIN_PV_CHARGE 38// PB23
#define PIN_PV_DONE 37 // PB22
#define PIN_PS_WS 36 // PB03 PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  // SPI Flash MISO on ItsyBitsy M0

#define PIN_SERCOM1_TX 40 // PA00 SERCOM1 PAD0
#define PIN_SERCOM1_RX 10 // PA01 SERCOM1 PAD1

// Pins with solder jumpers 
#define PIN_ID0 6 // PA20 - SERCOM5/PAD[2]
#define PIN_ID1 7 // PA21 - SERCOM5/PAD[3]


// Serial2 for Degugging on HW > 1.3
Uart Serial2(&sercom1, PIN_SERCOM1_RX, PIN_SERCOM1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;

#ifdef HAS_HEATER
  #define HEATER_MAX_V 9.20 // Maximum setpoint voltage, thermal limit by inductor (~1,8A conntiniuos, actual voltage ~8V)
  #define PIN_EN_DCDC 19 // A5 19 PB02
  #define PIN_EN_HEATER 4 //D4/A8 PA08
#endif


#define PIN_DAVIS_SPEED 17 // A3 17 PA04
#define PIN_DAVIS_DIR 18 // A4 18 PA05
#define PIN_DAVIS_POWER 9 // 9 PA07 - power for direction potentiometer. Turn off if not used to save power

// SERCOM 3 - I²C Barometer / Digipot
#define PIN_SDA 26 // PA22
#define PIN_SCL 27 // PA23

// I2C Barometer
BMP280 bmp280; // Bosch BMP280
SPL06 spl; // Goertek SPL06-001
Adafruit_BMP3XX bmp3xx;
// DPS310 as alternative?

// DAVIS6410 Pinout
// Black - Wind speed contact (closure to ground)
// Red - Ground
// Green - Wind direction pot wiper (20KΩ potentiometer)
// Yellow - Pot supply voltage

// SERCOM 0 - uart serial Sensor/ GPS / Debug
#define PIN_RX 0 // A6 PA11
#define PIN_TX 1 // A7 PA10

#define PIN_STATUSLED 13 // PA17 // blue, 1.2mA @ 2.77V = 440Ohm
#define PIN_ERRORLED 4 //D4/A8 PA08 // red 2.9mA @ 2.16V = 390Ohm
#define PIN_V_READ_TRIGGER A2 // D16 PB09
#define PIN_V_READ A1 // D15 PB08 - { PORTB,  8, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, // ADC/AIN[2]

#define VERSIONFILE (char*) "version.txt"
#define SETTINGSFILE (char*) "settings.txt"


#define MCP4652_I2C_ADDR	0x2F
#define WRITE_WIPER_DCDC	(0b0000<<4) // Wiper 0 = DCDC
#define WRITE_WIPER_MPPT	(0b0001<<4) // Wiper 1 = MPPT
#define WRITE_TCON	(0b0100<<4)
#define CMD_WRITE	(0b00<<2)

#define LORA_SYNCWORD 0xF1 //SX1262: 0xF4 0x14 https://blog.classycode.com/lora-sync-word-compatibility-between-sx127x-and-sx126x-460324d1787a is handled by RadioLib

SX1276 radio_sx1276 = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RESET, PIN_LORA_DIO1);
SX1262 radio_sx1262 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);
LLCC68 radio_llcc68 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);

PhysicalLayer* radio_phy = nullptr;
bool transmittedFlag = false;

// Pulsecounter
volatile uint32_t pulsecount =0; // pulses from wind speed sensor using reed switch

// WDT and CPU Clock
#define WDT_PERIOD 2500 // ms for wdt to wait to reset mcu if not reset in time
bool use_wdt = false; // use watchdog reset
int div_cpu = 1; // current div
int div_cpu_fast = 1; // fast CPU clock 1= 48Mhz/1
int div_cpu_slow = 1; // if > 1, cpu clocks down by 48/n Mhz

bool first_sleep = true; // first sleep after reset, USB perephial could still be on
bool usb_connected = false;

uint32_t sleep_allowed = 0; // time() when is is ok so eenter deepsleep
uint32_t sleeptime_cum = 0; // cumulative time spend in sleepmode, used for time() calculation
int heading_offset = 0;

// ### Variables for storing settings from file, may be overewritten #####
String station_name="";
float pos_lat = 0;
float pos_lon = 0;
float altitude = -1;

#ifdef HAS_HEATER
  bool is_heater = false;
  float mppt_voltage = 5.5;
  float heater_voltage = 4.5;
  uint32_t heater_on_time_cum = 0;
  #define MAX_HEAT_TIME 30*60*1000 //30 minv
#endif
bool use_mcp4652 = true; // used on first version of PCB (<=1.3) to set MPPT and DCDC voltage

enum HW_Version{
  HW_unknown,
  HW_1_3,
  HW_2_0
};

HW_Version hw_version = HW_unknown;

enum LORA_MODULE{
  LORA_NONE,
  LORA_SX1276,
  LORA_SX1262,
  LORA_LLCC68
};

LORA_MODULE lora_module = LORA_NONE;

enum BARO_CHIP{
  BARO_NONE,
  BARO_BMP280,
  BARO_SPL06,
  BARO_BMP3xx
};

// Sensor selction
bool undervoltage = false;
bool is_ws80 = false;
bool is_davis6410 = false;
bool is_baro = false;
bool is_gps = false;
bool testmode = false; // send without weather station to check lora coverage
bool is_beacon = false; // use as FANET/FLARM Beacon. requires GPS
uint32_t gps_baud = 9600;
BARO_CHIP baro_chip = BARO_NONE;
uint32_t wind_age = WIND_AGE; 
uint32_t gust_age = GUST_AGE;

bool settings_ok = false;
uint32_t next_baro_reading = 0;

// Message timing
uint32_t broadcast_interval_weather = BROADCAST_INTERVAL;
uint32_t last_msg_weather = 0;
uint32_t broadcast_interval_name = 1000*60*5; // 5 min
uint32_t last_msg_name = 0;
uint32_t broadcast_interval_info = 0;
uint32_t last_msg_info = 0;

uint32_t last_ws80_data = 0;
uint32_t last_fnet_send = 0; // last package send
uint32_t fanet_cooldown = 4000;

// debugging
bool test_with_usb = false;
bool no_sleep = false;
bool test_heater = false;
bool debug_enabled = true; // enable uart debug messages
bool errors_enabled = true;
bool forward_serial_while_usb = false;

// Measurement values
float baro_pressure = 0;
float baro_temp = 0; // temp from bmp280, inside case/on pcb
int wind_dir_raw = 0;
int wind_heading = 0;
float wind_speed = 0;
float wind_gust = 0;
float temperature = 0;
int humidity = 0;
int light_lux = 0;
float uv_level = 0;
float ws80_vcc = 0;
float cap_voltage = 0; // WS85 supercap voltage
float batt_volt = 0;
int batt_perc = 0;


#define WAKEUP_NONE 0
#define WAKEUP_RTC 1
#define WAKEUP_EIC 2
#define WAKEUP_WDT 3
int wakeup_source = WAKEUP_NONE;
const char* wakeup_source_string [4] = {"NONE", "RTC", "EIC", "WDT"};

bool pv_charging; // currently charging, state from pv charger
bool pv_done; // battery fully charged, state from pv charger


uint32_t sleep_offset =0; //time passed since last increment of time(). used for ws80 deepsleep reading time offset from rtc

// Function prototypes
bool setup_flash();
void setup_usb_msc();
DSTATUS disk_status(BYTE pdrv);
DSTATUS disk_initialize(BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv,BYTE cmd,	void *buff);
bool format_flash();
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize);
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize);
void msc_flush_cb (void);
bool msc_writable_callback(void);


// Helper ----------------------------------------------------------------------------------------------------------------------

// get current millis since reset, including time spend in deepsleep. Missing time waiting for UART ws80 sensor
uint32_t time(){
  return millis() + sleeptime_cum + sleep_offset;
}



bool led_status(bool s){
static bool state = false;
bool ret = false;
    if(s){
      pinMode(PIN_STATUSLED,OUTPUT);
      digitalWrite(PIN_STATUSLED, 1);
      if(state){ret = true;} // was on before
      state = true;
    } else {
      state = false;
      digitalWrite(PIN_STATUSLED, 0);
      pinDisable(PIN_STATUSLED);
    }
  return ret;
}


// sets red LED pin to output and turns it on
bool led_error(bool s){
static bool state = false;
bool ret = false;
  if( hw_version >= HW_2_0){
    if(s){
      pinMode(PIN_ERRORLED,OUTPUT);
      digitalWrite(PIN_ERRORLED, 1);
      if(state){ret = true;} // was on before
      state = true;
    } else {
      state = false;;
      digitalWrite(PIN_ERRORLED, 0);
      pinDisable(PIN_ERRORLED);
    }
  } else {
    ret = led_status(s);
  }
  return ret;
}

void i2c_scan(){
  byte error, address;
  int nDevices;
  nDevices = 0;
  for(address = 1; address < 127; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0){
      DEBUGSER.print("I2C device found at address 0x");
      if (address<16)
        DEBUGSER.print("0");
      DEBUGSER.println(address,HEX);

      if(address == 0x2F){hw_version = HW_1_3;} // only HW1.3 has digipot
 
      nDevices++;
    }
    else if (error==4){
      DEBUGSER.print("Unknown error at address 0x");
      if (address<16)
        DEBUGSER.print("0");
      DEBUGSER.println(address,HEX);
    }    
  }
  if (nDevices == 0){DEBUGSER.println("No I2C devices found\n");}
}


// read solderjumers one and disable pins again
uint8_t read_id(){
  uint8_t id= 0;
  pinMode(PIN_ID0, INPUT_PULLUP);
  pinMode(PIN_ID1, INPUT_PULLUP);

  id &= digitalRead(PIN_ID0);
  id &= digitalRead(PIN_ID1) << 1;

  pinDisable(PIN_ID0);
  pinDisable(PIN_ID1);
  return id;
}

// process line in 'key=value' format and hand to callback function
bool process_line(char * in, int len, bool (*cb)(char*, char*)){
  #define BUFFLEN 35
  char name [BUFFLEN];
  char value [BUFFLEN];
  memset(name,'\0',BUFFLEN);
  memset(value,'\0',BUFFLEN);
  bool literal = false; // 
  char* ptr = name; //start with name filed
  int c = 0; // counter
  int oc= 0; // output counter
  bool cont = true; //continue flag

  while (cont && (c < max(len,255)) && (oc < (BUFFLEN-1)) ){ // limit to 255 chars per line
    switch (in[c]) {
      case '\r': cont = false; break;
      case '\n': cont = false; break;
      //case '-': cont = false; break; // catches negative temps
      case '>': cont = false; break;
      case '!':  cont = false; break;
      case '#':  cont = false; break;
      case '?':  literal = true; break; // enable literal mode
      case '=':  if(oc && (ptr == name) ){ *(ptr+oc) = '\0'; ptr = value; oc = 0;} break; // switch to value
      case ' ':  if(!literal) {break;} // avoid removing whitespaces from name
      default:   *(ptr+oc) = in[c]; oc++; break; // copy char
    }
    c++;
  }
  if(oc && (ptr == value)){ // value is set
    return cb(name, value);
  }
  return false;
}


/*
========== WS85 Ver:1.0.7 ===========
>> g_RrFreqSel = 868M
>> Device_ID  = 0x002794
-------------------------------------
WindDir      = 76
WindSpeed    = 0.5
WindGust     = 0.6
GXTS04Temp   = 24.4

UltSignalRssi  = 2
UltStatus      = 0
SwitchCnt      = 0
RainIntSum     = 0
Rain           = 0.0
WaveCnt[CH1]   = 0
WaveCnt[CH2]   = 0
WaveRain       = 0
ToaltWave[CH1] = 0
ToaltWave[CH2] = 0
ResAdcCH1      = 4095
ResAdcSloCH1   = 0.0
ResAdcCH2      = 4095
ResAdcSloCH2   = 0.0
CapVoltage     = 0.80V
BatVoltage     = 3.26V
=====================================
*/

// Set measurement value read from WS80 UART
bool set_value(char* key,  char* value){
  //printf("%s = %s\r\n",key, value);
  if(strcmp(key,"WindDir")==0) {wind_dir_raw = atoi(value); add_wind_history_dir(wind_dir_raw); return false;}
  if(strcmp(key,"WindSpeed")==0) {wind_speed = atof(value)*3.6; add_wind_history_wind(wind_speed); printf("%s = %0.2f\n",key, wind_speed); return false;}
  if(strcmp(key,"WindGust")==0) {wind_gust = atof(value)*3.6; add_wind_history_gust(wind_gust); printf("%s = %0.2f\n",key, wind_gust); return false;}
  if(strcmp(key,"Temperature")==0) {temperature = atof(value); return false;}
  if(strcmp(key,"GXTS04Temp")==0) {temperature = atof(value); return false;} // WS85
  if(strcmp(key,"Humi")==0) {humidity = atoi(value); return false;}
  if(strcmp(key,"Light")==0) {light_lux = atoi(value); return false;}
  if(strcmp(key,"UV_Value")==0) {uv_level = atof(value); return false;}
  if(strcmp(key,"CapVoltage")==0) {cap_voltage = atof(value); return false;} // WS85
  if(strcmp(key,"BatVoltage")==0) {
    ws80_vcc = atof(value); 
    last_ws80_data =time();
    log_flush();
    //log_i("WS80 data complete\r\n");
    return true;
  }
  return false;
}

#ifdef HAS_HEATER
// Digipot, Solar & DCDC ----------------------------------------------------------------------------------------------------------------------

// Calc register value for digipot with resistor constellation given
int calc_regval(float val, float setpoint, int r1, int r2, int steps, int rmax){
  int regval = ((setpoint * r1 / (val-setpoint)) - r2)/rmax*steps;
  if(regval > 255){regval = 255;}
  if(regval < 0 ){regval = 0;}
  return regval;
}

// Solar MPPT voltage to register value
int calc_cn3791(float val){
  const int r1 = 300;
  const int r2 = 12;
  const int steps = 256;
  const float setpoint = 1.205;
  const int rmax = 100; // 100k
  return calc_regval(val, setpoint, r1, r2, steps, rmax);
}

// DCDC voltage to register value
int calc_mt3608(float val){
  const int r1 = 510;
  const int r2 = 27;
  const int steps = 256;
  const float setpoint = 0.6;
  const int rmax = 100; // 100k
  return calc_regval(val, setpoint, r1, r2, steps, rmax);

}
// Set output of digipot channel
void mcp4652_write(unsigned char addr, unsigned char value){
  if(use_mcp4652){
    //log_i("Setting Whiper to: ", (uint32_t) value);
    unsigned char cmd_byte = 0;
    cmd_byte |= (addr | CMD_WRITE);
    //Wire.begin();
    Wire.beginTransmission(MCP4652_I2C_ADDR);
    Wire.write(cmd_byte);
    Wire.write(value);
    if(Wire.endTransmission() != 0){
      log_e("Faild to set MCP4652. Disabling\r\n");
      use_mcp4652 = false;
      hw_version = HW_2_0;
      return;
    }
    hw_version = HW_1_3;
  }
}

// Write both outputs of digipot
void apply_mcp4652(){
  if(use_mcp4652){
    if(heater_voltage > HEATER_MAX_V){
        heater_voltage = HEATER_MAX_V;
    }
    //log_i("Setting Digipot\r\n"); log_flush();
    mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(heater_voltage));
    delay(3);
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
  }
}
#endif

void switch_WS_power (bool state){
  static bool current_power_state = false;
  if(state && !current_power_state){ // turn on
    current_power_state = true;
    pinMode(PIN_PS_WS,OUTPUT);
    digitalWrite(PIN_PS_WS, 0);
  }
  if(!state && current_power_state){ // turn off
    current_power_state = false;
    digitalWrite(PIN_PS_WS, 1);
  }
  
}

// read status pins of solar charger
void get_solar_charger_state(){
  pinMode(PIN_PV_CHARGE, INPUT_PULLUP);
  pinMode(PIN_PV_DONE, INPUT_PULLUP);
  pv_done = !digitalRead(PIN_PV_DONE);
  pv_charging = !digitalRead(PIN_PV_CHARGE);
  pinDisable(PIN_PV_CHARGE);
  pinDisable(PIN_PV_DONE);
}

// Trigger ADC and calc battery value in percent and volts
void read_batt_perc(){
  static uint32_t last_battery_reading=0;
  static int batt_p = 50;
  // only sample if last reading is older than 100ms
  if(time()- last_battery_reading > 100){
    last_battery_reading = time();
    analogReference(AR_INTERNAL1V0);
    analogReadResolution(10);
    pinMode(PIN_V_READ, INPUT);
    delay(1);
    float val=0;
    digitalWrite(PIN_V_READ_TRIGGER,0);
    delayMicroseconds(10);
    for( int i= 0; i< 4; i++){
      val += (float) analogRead(PIN_V_READ);
    }
    digitalWrite(PIN_V_READ_TRIGGER,1);

    //pinDisable(PIN_V_READ_TRIGGER); //disable at sleep begin
    val /=4;
    pinDisable(PIN_V_READ);
    //log_i("V_Batt_raw: ", val);
    if(hw_version == HW_1_3){
      val *= 0.00432; // 100k/360k 1.0V Vref
    }
     else if(hw_version == HW_2_0){
      val *= 0.0040925; // 100k/330k 1.0V Vref
    }
    
    batt_volt = val;
    float v = val;

  // gets remapped by fanet (State of Charge  (+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%))
    if( v < 3.4)  {batt_p = 0; switch_WS_power(0);} // Turn off power for WS80
    if( v > 3.5)  {batt_p = 0; switch_WS_power(1); undervoltage = false;}
    if( v > 3.55) {batt_p = 10;}
    if( v > 3.65) {batt_p = 20;}
    if( v > 3.70) {batt_p = 30;}
    if( v > 3.75) {batt_p = 40;}
    if( v > 3.80) {batt_p = 50;}
    if( v > 3.85) {batt_p = 60;}
    if( v > 3.90) {batt_p = 70;}
    if( v > 3.95) {batt_p = 80;}
    if( v > 4.05) {batt_p = 90;}
    if( v > 4.15) {batt_p = 99;}
    if(( v > 4.15) && pv_done) {batt_p = 100;}
  }
    batt_perc =  batt_p;
}



// Sensors ----------------------------------------------------------------------------------------------------------------------
// request baro sampling & conversion
void baro_start_reading(){
  //sercom3.resetWIRE();
  //Wire.begin();
  if(!next_baro_reading){
    if(baro_chip == BARO_BMP280){ 
      next_baro_reading = time() + bmp280.startMeasurment();
    }
    if(baro_chip == BARO_SPL06){ 
      //log_i("Baro start ", time());
      //spl.start_measure();
      next_baro_reading = time() + 27;
    }
    if(baro_chip == BARO_BMP3xx){ 
      bmp3xx.setOutputDataRate(BMP3_ODR_50_HZ);
    // bmp3xx.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    // bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    // bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp3xx.performReading();
      next_baro_reading = time() + 5;
    }
    } else {
      if(time() > (next_baro_reading +200)){
        next_baro_reading = 0;
        //log_i("Baro timeout ", time());
      }
    }
}

// read value of baro, needs baro_start_reading in advance
void read_baro(){
  bool data_ok = false;
  double T,P;

  if(baro_chip == BARO_BMP280){
    if(next_baro_reading && (time() > next_baro_reading)){
      uint8_t result = bmp280.getTemperatureAndPressure(T,P);
      if(result!=0){
        data_ok = true;
      }
    }
  }

  else if(baro_chip == BARO_SPL06){
    if(next_baro_reading && (time() > next_baro_reading)){
      P = spl.get_pressure();
      T = spl.get_temp_c();
      //spl.sleep(); 
      
      if(P > 0){
        data_ok = true;
        //log_i("Baro finish ", time());
      }
    }
  }
  else if(baro_chip == BARO_BMP3xx){ 
    P = bmp3xx.pressure/100;
    T = bmp3xx.temperature;
    bmp3xx.setOutputDataRate(BMP3_ODR_0_001_HZ);
    if(P > 0){data_ok = true;}
  }

  if(data_ok){
    baro_temp = T;
    next_baro_reading = 0;
    if (altitude > -1){
        baro_pressure = (P * pow(1-(0.0065*altitude/(temperature + (0.0065*altitude) + 273.15)),-5.257));
    } else {
      baro_pressure = P;
    }
  } else {
    //log_i("Baro no data, retry\n");
    //baro_start_reading();
  }
}

// while USB is connected, forward ws80 data to usb serial port
void forward_ws80_serial(){
  if(forward_serial_while_usb){
    static char buffer [512];
    static int bufferpos = 0;
    while (WS80_UART.available()){
      buffer[bufferpos] = WS80_UART.read();
      bufferpos++;
      if(bufferpos > 512){bufferpos =0;}
    

      if(bufferpos && (buffer[bufferpos] == '\n')){
        if(usb_connected){Serial.write(buffer, bufferpos);}
        Serial1.write(buffer, bufferpos);
        bufferpos = 0;
      }
    }
  }
}


bool parse_wsdat(char* input, int len){
    char buffer[20];
    int num=0;
    strncpy(buffer, input, len+1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token = strtok(buffer, ",");
    while (token != NULL) {
        if(num == 0){
          if(strcmp(token, "$WSDAT")==0){
            printf("%s\n", token);
            num =1;
          }
        }
        else if(num == 1){
          wind_speed = atof(token);
          num =2;
        }
        else if(num == 2){
          wind_gust = atof(token);
          num =3;
        }
        else if(num == 3){
          wind_dir_raw = atoi(token);
          //printf("WSDAT: %0.2f, %0.2f, %i\n", wind_speed, wind_gust, wind_dir_raw);
          //DEBUGSER.flush();
          return true;
        }
        token = strtok(NULL, ",");
    }
    return false;
}

// this function is meant to just read the $WSDAT output of the WS80. 
// This does not require a 10k pullup on T7 and also reduces power consuption to 0.3mA instead of 1.3mA with debug enabled. 
// The problem: the slow wakeup of the SAMD21 misses the serial data very often
void read_wsdat(){
  // $WSDAT,0.0,0.7,224*4A
  static char buffer [50];
  int co = 0;
  uint32_t last_data = micros();
  led_error(1);

  while( (micros()- last_data < 2000) ){ // 
    while (WS80_UART.available()){
      led_status(1);
      buffer[co] = WS80_UART.read();
      last_data = micros();
      //DEBUGSER.write(buffer[co]);
      if(buffer[co] == '*'){
        //DEBUGSER.println();
        if(parse_wsdat(buffer, co)){
          co = 0;
          led_status(0);
          led_error(0);
          return;
        }
        co = 0;
        
      }
      led_status(0);
      co++;
      if(co >= 50){
        log_e("Buffer size exeeded\n");
        return;
      }
    }
  } 
led_error(0);
}

// read UART and process input buffer, needs to be called periodically until new block is complete (last_ws80_data = time())
bool read_ws80(){
  static char buffer [BUFFERSIZE];
  static int co = 0;
  uint32_t last_data = micros();
  uint32_t cpy_last_ws80_data = last_ws80_data;

  while( cpy_last_ws80_data == last_ws80_data &&(micros()- last_data < 1900)){ // 
    while (WS80_UART.available()){
      //led_error(1); // blink LED, for debugging
      buffer[co] = WS80_UART.read();
      last_data = micros();
      if(buffer[co] == '\n'){
        if(forward_serial_while_usb && usb_connected){
          Serial.write(buffer, co);
        }
        if(process_line(buffer, co, &set_value)){ // Line complete
          co = 0;
          return true;
        }
        co = -1; // gets +1 below
      }
      co++;
      if(co >= BUFFERSIZE){
        log_e("Buffer size exeeded\r\n");
        //led_error(0);
        return false;
      }
    }
    //led_error(0);
  } 
  return false;
}

// Davis 6410 Sensor ----------------------------------------------------------------------------------------------------------------------
int read_wind_dir(){
  int val = 0;
  analogReference(AR_DEFAULT); //3.3V refernece
  pinMode(PIN_DAVIS_DIR, INPUT);
  pinMode(PIN_DAVIS_POWER,OUTPUT);
  digitalWrite(PIN_DAVIS_POWER,1);

int d = analogRead(PIN_DAVIS_DIR);


if(is_davis6410){
  // Variable resistance 0 - 20KΩ; 10KΩ = south, 180°)
  val = (int)(360.0/1023.0 * (float)d);
}
 // ... other analog sensors
digitalWrite(PIN_DAVIS_POWER,0);
pinDisable(PIN_DAVIS_POWER);
pinDisable(PIN_DAVIS_DIR);
return val;
}


void calc_pulse_sensor(uint32_t pulses, uint32_t dmillis){
  log_i("delta_t: ", dmillis);
  log_i("pulses: ", pulses); 
  wind_dir_raw = read_wind_dir();
  add_wind_history_dir(wind_dir_raw);
  
  if(is_davis6410){
    wind_speed = (float) pulses * 1.609 * (2250.0/((float)dmillis+1) ); // avoid div/0
  }
  // ... other analog sensors
  add_wind_history_wind(wind_speed);
  add_wind_history_gust(wind_speed);
  save_history(wind_speed, temperature, humidity, light_lux, batt_volt, pv_charging, pv_done);
}

// Heater ----------------------------------------------------------------------------------------------------------------------
  #ifdef HAS_HEATER
void run_heater(){
  static uint32_t last_heater_calc = 0;
  static uint32_t h_switch_on_time = 0;
  static float current_output_v = 0;
  static bool en_heater = false;
  static uint32_t rampstep = 0;

  // calc if heater needs to be on or off
  if(time()- last_heater_calc > 10000){
    last_heater_calc = time();
    read_batt_perc(); // get new voltage if outdated
    uint32_t light_hist= history_sum_light(24);
    uint32_t wind_hist= history_sum_wind(24);
    //log_i("# Heater Info\r\n");
    log_i("light_hist: ", light_hist);
    log_i("wind_hist: ",wind_hist);
    if(h_switch_on_time){
      log_i("Heater on since [s]: ", (time()-h_switch_on_time)/1000);
    }
    // Turn on heater
    if(( temperature < 3 && \
        batt_volt > 3.6 && \
        light_hist < 500 && \
        wind_hist < 10 && \
        time() > 1800000) // min 30 min on
        || test_heater
        ){
          en_heater = true;
        }
  }
  // Turn off (checked on every call)
  if( (h_switch_on_time && (time()-h_switch_on_time > MAX_HEAT_TIME) ) || (batt_volt <= 3.1) || !test_heater){  // limit heatertime
    heater_on_time_cum += time()-h_switch_on_time; 
    en_heater = false;
  }

  if( (hw_version == HW_1_3) && en_heater){
    if(!current_output_v){ // is currently off
      current_output_v = 3; // start at 3V (non switching)
      mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v)); // to avoid 1,1A short circuit detection set to half voltage at load switch enable
      pinMode(PIN_EN_HEATER,OUTPUT);
      pinMode(PIN_EN_DCDC,OUTPUT);
      digitalWrite(PIN_EN_HEATER,1);
      delay(5);
      digitalWrite(PIN_EN_DCDC,1);
      h_switch_on_time = time();
      log_i("Heater turned on: ", time());
    }

    if(time() - h_switch_on_time > 20000){ // after 20 secs ramp up
      if(time()- rampstep > 1000){
        rampstep = time();
        if(current_output_v < heater_voltage){
          current_output_v += 0.025;
          mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v));
        }
        if(current_output_v > heater_voltage){
          current_output_v = heater_voltage;
          mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(current_output_v));
        }
      }
    }
    // VBAT = 3V, max 5V output ~3W
    // 3.2V = 5,5V ~ 3,36W
    // 3,5V = 6V
    
  // turn heater off if running
  } else {
    if(h_switch_on_time){
      h_switch_on_time=0;
      current_output_v=0;
      digitalWrite(PIN_EN_HEATER,0);
      digitalWrite(PIN_EN_DCDC,0);
      pinDisable(PIN_EN_HEATER);
      pinDisable(PIN_EN_DCDC);
      log_i("Heater turned off: ", time());
    }
  }
}
  #endif

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_EIC(){
  wakeup_source = WAKEUP_EIC;
}

// enable interrupt on uart rx pin and wait for data
uint32_t sleep_til_serial_data(){
  uint32_t sleepcounter =0;
  //log_i("sleep\r\n"); log_flush();
  attachInterruptWakeup(PIN_RX, wakeup_EIC, FALLING, false);
  sleep(false); // false
  WS80_UART.begin(115200*div_cpu); // begin again, because we used the RX pin as wakeup interrupt.
  detachInterrupt(PIN_RX);
  sleepcounter = read_time_counter();
  //log_i("Actual_sleep: ", sleepcounter);
  //log_i("Wakeup_source: "); log_i(wakeup_source_string[wakeup_source]);log_i("\r\n");

  //Wire.setClock(100000);
    return sleepcounter;
}

// called after sleep
void wakeup(){
  // USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; // Re-enable USB, no need, not working?
  log_i("\r\n\r\n\r\n#####################");
  log_i("\r\nWakeup: ", time()); 
  log_i("Wakeup_source: "); log_i(wakeup_source_string[wakeup_source]);log_i("\r\n");
  wakeup_source = WAKEUP_NONE;

  pinMode(PIN_V_READ_TRIGGER, OUTPUT); // prepare voltage measurement, charge trigger cap
  digitalWrite(PIN_V_READ_TRIGGER,1);

  get_solar_charger_state();
  if(is_baro){
    baro_start_reading(); // request data aquisition, will be read later
  }

  read_batt_perc();
  sleep_allowed = time() + 100; // go back to sleep after 6 secs as fallback
}

// calc time to sleep til next fanet message needs to be send
uint32_t calc_time_to_sleep(){
  uint32_t tts_weather = -1;
  uint32_t tts_name = -1;
  uint32_t tts_info = -1;
  uint32_t tts = 0;

  if( last_msg_weather && broadcast_interval_weather){
    if( (last_msg_weather + broadcast_interval_weather) > time() ){
      tts_weather = broadcast_interval_weather - (time()-last_msg_weather);
    } else {
      tts_weather = 0;
    }
  }
  if( last_msg_name && broadcast_interval_name){
    if( (last_msg_name + broadcast_interval_name) > time()){
      tts_name = broadcast_interval_name - (time()-last_msg_name);
    } else {
      tts_name = 0;
    }
  }
  if( last_msg_info && broadcast_interval_info){
    if( (last_msg_info + broadcast_interval_info) > time()){
      tts_info = broadcast_interval_info - (time()-last_msg_info);
    }else {
      tts_info = 0;
    }
  }
  //log_i("tts_weather: ", tts_weather);
  //log_i("tts_name: ", tts_name);
  //log_i("tts_info: ", tts_info);
  
  tts = min(min(tts_name, tts_info), tts_weather);
  if( fanet_cooldown && last_fnet_send  && (time() - last_fnet_send + tts < fanet_cooldown)){
    tts += fanet_cooldown - (time()-last_fnet_send);
    }
  if(tts == (uint32_t)-1){ tts=0;}

  if(batt_volt && batt_volt < VBATT_LOW){
    tts = 3600*500; // sleep 30min
    undervoltage = true;
    log_i("Undervoltage");
  }
  return tts;
}

// RTC Handler callback, do not rename. gets called on rtc (timer) interrupt
void RTC_Handler(void){
  if (RTC->MODE1.INTFLAG.bit.OVF && RTC->MODE1.INTENSET.bit.OVF) {  // Check if an overflow caused the interrupt
    RTC->MODE1.INTFLAG.bit.OVF = 1;                                 // Reset the overflow interrupt flag
    wakeup_source = WAKEUP_RTC;
  }
}

// shut everything down, enable deepsleep
void go_sleep(){
  uint32_t actual_sleep = 0;

  //fanet.end(); // set RFM95 to sleep
  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep && !(test_with_usb)){
    log_i("Disable USB\n");
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;
    if(set_cpu_div(div_cpu_slow)){ //USB needs 48Mhz clock, as we are finished with USB we can lower the cpu clock now.
      div_cpu = div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist still 48M, so every clock needs to by multiplied manually
    }
  }

// if never send a weather msg, assume it was send now
  if(!last_msg_weather){
    last_msg_weather = time();
  }

  int32_t time_to_sleep = calc_time_to_sleep();
  if(last_ws80_data == 0){ time_to_sleep = 12000;} // if no data from WS80 received 
  if(!settings_ok){ 
    log_e("No settings file\n");
    log_e("Sleeping forever\r\n");
    time_to_sleep = 0xFFFFFFF;
    } // if settings not ok sleep forever
  if(!time_to_sleep){ return;} // if time_to_sleep = 0, do not sleep at all

  log_i("will sleep for ", time_to_sleep > 20000?-1: time_to_sleep);
  log_flush();

// disable wdt during sleep
  if(use_wdt) {
    wdt_disable();
  }

// Using TC4 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(!undervoltage && is_davis6410){ // pulse counting anemometer
    actual_sleep = rtc_sleep_cfg(time_to_sleep);
    setup_pulse_counter(); // need to setup GCLK6 before
    while(wakeup_source != WAKEUP_RTC){ // ignore other wakeups (external pin Interrupt, if configured)
      sleep(false);
    }
    sleeptime_cum += actual_sleep;
    pulsecount = read_pulse_counter();
    calc_pulse_sensor(pulsecount, actual_sleep);
    // elseif (is_other_pulsecounting_sensor){
      //calc_...(pulsecount, actual_sleep);
    //}

// UART sensor, just sleep
  } else if(!undervoltage && is_ws80){ // no pulse counting anemometer, no interrupts
    int32_t t =0;
    if(settings_ok && (time()> 2500)  && !usb_connected && !no_sleep && !testmode){
      reset_time_counter();
      actual_sleep = rtc_sleep_cfg(time_to_sleep);
      while(wakeup_source != WAKEUP_RTC){
        t = sleep_til_serial_data();
        if(read_ws80()){ // if true, we received a data block, so it is ok to sleep for ~ 4 seconds without listening to serial data
          if(time_to_sleep > t ){
            actual_sleep = rtc_sleep_cfg( ((time_to_sleep - t) < 4685) ? time_to_sleep - t:4685); // sleep 4750ms if tts is still longer, or less if it less
            //log_i("will sleep1: ", actual_sleep); log_flush();
            sleep(false);
            t = read_time_counter();
            if(time_to_sleep > t ){
              time_to_sleep -= t;
              rtc_sleep_cfg( time_to_sleep);
              wakeup_source = WAKEUP_NONE; // reset wakeup reason
            }
            sleeptime_cum += t;
            reset_time_counter();
          }
        }
      }
    t = read_time_counter();
    sleeptime_cum += t;
    }
    sleep_offset = 0; // reset temporary offset
    
  } else { // no sensor configured
    actual_sleep = rtc_sleep_cfg(time_to_sleep);
    log_flush();
    sleep(false);
    sleeptime_cum += actual_sleep;
  }

// If sleep is disabled for debugging, use delay
  if(!usb_connected && (no_sleep || testmode)){
    log_i("INSOMNIA or Testmode enabled, unsing delay() instead of deepsleep");
    delay(time_to_sleep);
    sleeptime_cum += time_to_sleep;
  }

// re-enable wdt after sleep
  if(use_wdt) {
    wdt_enable(WDT_PERIOD,false);
  }
  if(set_cpu_div(div_cpu_fast)){
    div_cpu = div_cpu_fast;
    log_ser_begin();
  }
  wakeup();
  if(test_with_usb){
    sleep_allowed = time() + time_to_sleep;
  }
}

// Settings ----------------------------------------------------------------------------------------------------------------------
bool apply_setting(char* settingName,  char* settingValue){
  if(debug_enabled){printf("Setting: %s = %s\r\n",settingName, settingValue); log_flush();}
  
  if(strcmp(settingName,"NAME")==0) {station_name = settingValue; return true;}
  if(strcmp(settingName,"LON")==0) {pos_lon = atof(settingValue); return true;}
  if(strcmp(settingName,"LAT")==0) {pos_lat = atof(settingValue); return true;}
  if(strcmp(settingName,"ALT")==0) {altitude = atof(settingValue); return true;}

#ifdef HAS_HEATER
  if(strcmp(settingName,"HEATER")==0) {is_heater = atoi(settingValue); return true;}
  if(strcmp(settingName,"V_HEATER")==0) {heater_voltage  = atof(settingValue); return true;}
  if(strcmp(settingName,"V_MPPT")==0) { mppt_voltage = atof(settingValue); return true;}
#endif

  if(strcmp(settingName,"HEADING_OFFSET")==0) {heading_offset = atoi(settingValue); return true;}
  if(strcmp(settingName,"GUST_AGE")==0) {gust_age = atoi(settingValue)*1000; return true;}
  if(strcmp(settingName,"WIND_AGE")==0) {wind_age = atoi(settingValue)*1000; return true;}
  
  if(strcmp(settingName,"BROADCAST_INTERVAL_WEATHER")==0) {broadcast_interval_weather = atoi(settingValue)*1000; return true;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_NAME")==0) {broadcast_interval_name = atoi(settingValue)*1000; return true;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_INFO")==0) {broadcast_interval_info = atoi(settingValue)*1000; return true;}

  if(strcmp(settingName,"SENSOR_BARO")==0) {is_baro = atoi(settingValue); return true;}
  if(strcmp(settingName,"SENSOR_BMP280")==0) {is_baro = atoi(settingValue); return true;} // for backward compatibility, may be removed some time
  if(strcmp(settingName,"SENSOR_DAVIS6410")==0) {is_davis6410 = atoi(settingValue); return true;}
  if(strcmp(settingName,"SENSOR_WS80")==0) {is_ws80 = atoi(settingValue); return true;}

  if(strcmp(settingName,"SENSOR_GPS")==0) {is_gps = atoi(settingValue); return true;}
  if(strcmp(settingName,"GPS_BAUD")==0) {gps_baud = atoi(settingValue); return true;}

  if(strcmp(settingName,"DEBUG")==0) {debug_enabled = atoi(settingValue); return true;}
  if(strcmp(settingName,"ERRORS")==0) {errors_enabled = atoi(settingValue); return true;}
  if(strcmp(settingName,"INSOMNIA")==0) {no_sleep = atoi(settingValue); return true;}
  if(strcmp(settingName,"TEST_USB")==0) {test_with_usb = atoi(settingValue); return true;}
  if(strcmp(settingName,"TESTMODE")==0) {testmode = atoi(settingValue); return true;}
  if(strcmp(settingName,"WDT")==0) {use_wdt = atoi(settingValue); return true;}
  if(strcmp(settingName,"DIV_CPU_SLOW")==0) {div_cpu_slow = atoi(settingValue); return true;}
  if(strcmp(settingName,"FORWARD_UART")==0) {forward_serial_while_usb = atoi(settingValue); return true;}
  

  if(strcmp(settingName,"GPS_BEACON")==0) {is_beacon = atoi(settingValue); return true;}

// Test commands
  if(strcmp(settingName,"TEST_HEATER")==0) {test_heater = atoi(settingValue); return true;}
  if(strcmp(settingName,"SLEEP")==0) {usb_connected =false; return true;}
  if(strcmp(settingName,"FORMAT")==0) {if(format_flash()){NVIC_SystemReset();} else {log_i("Error Formating Flash\r\n");} return true;}
  if(strcmp(settingName,"RESET")==0) {setup(); return true;}
  if(strcmp(settingName,"DELAY")==0) {delay(atoi(settingValue)); return true;} // delay for WDT testing
  if(strcmp(settingName,"REBOOT")==0) {NVIC_SystemReset(); return true;}
  return false;
}

void print_settings(){
  if(debug_enabled){
    log_i("Name: "); log_i(station_name.c_str()); log_i("\r\n");
    log_i("Lon: ", pos_lon);
    log_i("Lat: ", pos_lat);
    log_i("Alt: ", altitude);
    log_flush();
  #ifdef HAS_HEATER
    log_i("Heater Voltage: ", heater_voltage);
    log_i("MPPT Voltage: ", mppt_voltage);
  #endif
    log_i("Heading Offset: ", heading_offset); 
    log_i("Broadcast Interval Weather: ", broadcast_interval_weather);
    log_flush();
  }
}
void print_data(){
  if(debug_enabled){
    log_i("\r\nmillis: ", millis()); 
    log_i("time: ", time()); 
    //log_i("Wind dir_raw: ", wind_dir_raw);
    log_i("Wind Heading: ", wind_heading);
    log_i("Wind Speed: ", wind_speed);
    log_i("Wind Gust: ", wind_gust);
    log_i("Temp: ", temperature);
    //log_i("Humd: ", humidity);
    if(is_baro){
    log_i("Baro: ", baro_pressure);
    log_i("PCB_Temp: ", baro_temp);
    }
    if(is_ws80) {
      log_i("VCC: ", ws80_vcc);
      //log_i("LUX: ", light_lux);
      //log_i("UV: ", uv_level);
    }
    log_i("\r\n");
    log_i("V_Bat: ", batt_volt);
    log_i("Bat_perc: ", batt_perc);
    log_i("PV_charge: ", pv_charging);
    log_i("PV_done: ", pv_done);
  }
}


// this is not working yet. The file is written to flash, but the record is not added to the FAT correctly.
bool create_versionfile(char * filename){
  //flash.setIndicator(PIN_ERRORLED, 1);
  File f;
  if (fatfs.begin(&flash) ){
    if(fatfs.exists(filename)){ 
      log_i("Version file exists\n");
      //return false;
    }
    log_i("creating file\n");
    f = fatfs.open(filename, FILE_WRITE);
    if (f) {
      log_i("Creating version file\n");
      f.print("Version: "); f.println(VERSION);
      f.print("FW Build: ") + f.print(__DATE__); f.println(__TIME__);
      f.print("FANET ID: "); f.print(FANET_VENDOR_ID,HEX); f.println(UniqueID[0] + ((UniqueID[1])<<8),HEX);
      f.print("HW Version: "); 
        if(hw_version == HW_1_3) { f.println("V1.3");}
        if(hw_version == HW_2_0) { f.println("V2.0");}
      f.print("LoRa Module: "); 
        if(lora_module == LORA_SX1276) { f.println("SX1276");}
        if(lora_module == LORA_SX1262) { f.println("SX1262");}
        if(lora_module == LORA_LLCC68) { f.println("LLCC68");}
      f.print("Barometer: "); 
        if(baro_chip == BARO_BMP280) { f.println("BMP280");}
        if(baro_chip == BARO_BMP3xx) { f.println("BMP3xx");}
        if(baro_chip == BARO_SPL06) { f.println("SPL06");}
      if(!f.close()){log_i("file close failed\n");}
      if(fatfs.exists(filename)){ 
      log_i("Version file sucess\n");
      }
      return true;
    } else {log_i("open file error\n");}
  } else {
    log_i("fs start fail\n");
  }
  return false;
}

// parse settingsfile
bool parse_file(char * filename){
  bool ret = false;
  led_status(1);
  File f;
  char filebuffer [BUFFERSIZE];
  int c = 0;
  int co = 0;
  int filesize =0;
  log_i("Reading Settings from file\r\n");

  if (fatfs.begin(&flash) ){
    f = fatfs.open(filename, FILE_READ);
    if (f) {
        filesize = f.available();
        log_i("Filesize: ", filesize);
        while (filesize - c > 0) {
          filebuffer[co] = f.read();

          if(filebuffer[co] == '\n'){
            process_line(filebuffer, co, &apply_setting); // Line complete
            co=-1; // gets +1 below
          }
          c++;
          co++;
          if(co >= BUFFERSIZE){
            log_e("File buffer error\r\n");
            return false;
          }
        }
        //DEBUGSER.println("End of file");
        process_line(filebuffer, co, &apply_setting);
        f.close();
        log_i("Settingsfile closed\n");
        //DEBUGSER.println("file closed");
        if(pos_lat != 0 && pos_lon != 0){
          ret = true;
        }
        led_status(0); // if LED stay on, settings failed
    }else {
      log_i("File not exists\r\n");
    }
    my_internal_storage.flush_buffer(); // sync with flash
  } else {
    log_e("Failed to start FS\r\n");
  }
  if(!ret){
    led_status(0);
    led_error(1);
    }
  return ret;
}

// Serial reads ----------------------------------------------------------------------------------------------------------------------
// read serial data from USB, for debugging
void read_serial_cmd(){
  static char buffer [BUFFERSIZE];
  static int co = 0;
  bool ok = false;

  while (Serial.available()){
    buffer[co] = Serial.read();
    //DEBUGSER.write(buffer[co]);
    if(buffer[co] == '\n'){
      ok = process_line(buffer, co, &apply_setting); // Line complete
      co=-1; // against +1 below
    }
    co++;
  }
  if(ok){
    // Apply new mppt voltage
  #ifdef HAS_HEATER
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    //apply_mcp4652(); // set voltages
    //Serial.println("V set");
  #endif

  }
}

// Send ----------------------------------------------------------------------------------------------------------------------

void send_msg_weather(){
  led_status(1);
  WindSample current_wind = get_wind_from_hist(wind_age);

  wind_gust = get_gust_from_hist(gust_age);
  wind_speed = current_wind.wind/10.0;
  wind_dir_raw = current_wind.dir_raw;
  wind_heading = wind_dir_raw + heading_offset;
  if(wind_heading > 359){ wind_heading -=360;}
  if(wind_heading < 0){ wind_heading +=360;}

  if( wind_speed > (wind_gust +3)){return;}
  weatherData wd;
  wd.vid = FANET_VENDOR_ID;
  wd.fanet_id =  UniqueID[0] + ((UniqueID[1])<<8);
  wd.lat = pos_lat;
  wd.lon = pos_lon;
  wd.bWind = true;
  wd.wHeading = wind_heading;
  wd.wSpeed = wind_speed;
  wd.wGust = wind_gust;      
  wd.bTemp = true;
  wd.temp = temperature;
  wd.bHumidity = true;
  wd.Humidity = humidity;
  if(is_baro){
    wd.bBaro = true;
    wd.Baro = baro_pressure;    
  } else {
    wd.bBaro = true;
    wd.Baro = -1;  
  }
  wd.bStateOfCharge = true;
  wd.Charge = batt_perc;

  if(testmode){
    wd.bBaro = true;
    wd.Baro = baro_pressure;  
    wd.wHeading = 123;
    wd.wSpeed = 5;
    wd.wGust = 7;      
    wd.temp = 10;
    wd.Humidity = 15;
    log_i("\r\nTESTMODE - Fake values\r\n");
  }

  log_i("\r\nSending Weather\r\n");

  int msgSize = sizeof(fanet_packet_t4);
  uint8_t* buffer = new uint8_t[msgSize];
  pack_weatherdata(&wd, buffer);

// write buffer content to console
#if 0
  for (int i = 0; i< msgSize; i++){
    printf("%02X ", (buffer)[i]);
  }
  Serial1.println();
#endif

  radio_phy->standby();
  radio_phy->startTransmit(buffer, msgSize);

  print_data();
  led_status(0);
  save_history(wind_speed, temperature, humidity, light_lux, batt_volt, pv_charging, pv_done); // only save history on send
  
}

void set_fanet_send_flag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

// check if everything is ok to send the wather data now
bool allowed_to_send_weather(){
  bool ok = settings_ok;
  

  if (ok){
    if(is_baro){ok &= (next_baro_reading == 0);}
    if(is_ws80) { ok &= (last_ws80_data || testmode); } // only send if weather data is up to date or testmode is enabled // && (time()- last_ws80_data < 9000))
    if(is_gps)  { ok &= (tinyGps.location.isValid()); } // only send if position is valid
  }

  if(!ok){
    if(last_ws80_data){
      //log_i("WS80 data age: ", time()- last_ws80_data);
    }
  }


  return ok;
}

// GPS ----------------------------------------------------------------------------------------------------------------------
void read_gps(){
  while (GPS_SERIAL.available() > 0){
    uint8_t c = GPS_SERIAL.read();
    //DEBUGSER.print(c);
    tinyGps.encode(c);
    if (tinyGps.location.isUpdated() && tinyGps.location.isValid() ){
        pos_lat = tinyGps.location.lat();
        pos_lon = tinyGps.location.lng();
        altitude = tinyGps.altitude.meters();
        //DEBUGSER.println("Position Update");
    }
  }
}

// check if last fanet package want sent recenctly
bool fanet_cooldown_ok(){
  if(time() -last_fnet_send > fanet_cooldown){
    return true;
  }
  return false;
}

void send_name(const char* name, int len){

  uint8_t* buffer = new uint8_t[len+4];
  fanet_header header;
  header.type = 2;
  header.vendor = FANET_VENDOR_ID;
  header.forward = false;
  header.ext_header = false;
  header.address = UniqueID[0] + ((UniqueID[1])<<8);

  memcpy(buffer, (uint8_t*)&header, 4);
  memcpy(&buffer[4], name, len);

// write buffer content to console
#if 0
  for (int i = 0; i< len+4; i++){
    printf("%02X ", (buffer)[i]);
  }
  Serial1.println();
#endif

  radio_phy->standby();
  radio_phy->startTransmit((uint8_t*) buffer, len+4);
}


bool radio_init(){
    if(radio_sx1276.begin(868.2, 250, 7, 5, LORA_SYNCWORD, 10, 12, 0) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_sx1276;
      log_i("Found LoRa SX1276\n");
      lora_module = LORA_SX1276;
      return true;
    } else if(radio_llcc68.begin(868.2, 250, 7, 5, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_llcc68;
      log_i("Found LoRa LLCC68\n");
      lora_module = LORA_LLCC68;
      return true;
    } else if(radio_sx1262.begin(868.2, 250, 7, 5, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      // NiceRF SX1262 issue https://github.com/jgromes/RadioLib/issues/689
      radio_phy = (PhysicalLayer*)&radio_sx1262;
      log_i("Found LoRa SX1262\n");
      lora_module = LORA_SX1262;
      return true;
    }  else {
        log_i("No LoRa found\n");
        return false;
    }
}


// Setup ----------------------------------------------------------------------------------------------------------------------

extern uint32_t __etext;

void setup(){
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock

  printf_init(DEBUGSER);
  log_i("\r\n--------------- RESET -------------------\r\n");
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("FW Build Time: ");  log_i(__DATE__); log_i(" "); log_i(__TIME__); log_i("\r\n");

  
  //printf("code end: %p\n", (void *)(&__etext));
  //printf("flash_start: %p\n", my_internal_storage.get_flash_address());
  //printf("flash_size: %lu\n", my_internal_storage.get_flash_size());

  hw_version = HW_2_0; 
  Wire.begin();
  i2c_scan();

  setup_display();

  if(!radio_init()){
    log_i("LoRa Module failed");
  }
  radio_phy->setPacketSentAction(set_fanet_send_flag);
  radio_phy->sleep();

  //printf("FANET ID: %02X%04X\r\n",fmac.myAddr.manufacturer,fmac.myAddr.id);

  if(setup_flash()){
    settings_ok = parse_file(SETTINGSFILE);
    if(!debug_enabled){
      DEBUGSER.println("Debug messages disabled");
    }
  }

  if(settings_ok){
    led_error(0);
    // Add altitude to station name, gets splittet by breezedude ogn parser
    if(altitude > -1){
      station_name += " (" + String(int(altitude)) + "m)"; // Testation (1234m)
    }
    setup_PM(is_davis6410); // powermanagement add || other sensors using counter
    wdt_enable(WDT_PERIOD,false); // setup clocks
    if(!use_wdt) {
      wdt_disable();
    }
  if(is_baro){
    bool baro_ok = false;
    if(!baro_ok && bmp280.begin()){ //0x76
      baro_ok = true;
      baro_chip = BARO_BMP280;
      log_i("BMP280 setup ok\r\n");
      bmp280.setOversampling(4);
    }
    if(!baro_ok && spl.begin(0x77)){
      baro_ok = true;
      baro_chip = BARO_SPL06;
      log_i("SPL06 setup ok\r\n");
    }
    if(!baro_ok && bmp3xx.begin_I2C(0x76)){
      baro_ok = true;
      baro_chip = BARO_BMP3xx;
      log_i("BMP3XX setup ok\r\n");
    }
    if(!baro_ok){
      log_e("Baro not found\r\n");
      is_baro = false;
    }
  }
  
#ifdef HAS_HEATER
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    apply_mcp4652();
#else
hw_version = HW_2_0;
#endif
  print_settings();
  } 
  if(!settings_ok) {
    // Needed for deepsleep
    setup_PM(false);
    wdt_enable(WDT_PERIOD,false); // setup clocks
    wdt_disable();
    setup_rtc_time_counter();
  }

  if(is_ws80){
    switch_WS_power(1); // Turn on WS80 Power supply with P-MOSFET (HW >= 2.2)
    setup_rtc_time_counter();
    WS80_UART.begin(115200);
  }
  if(is_gps){
    GPS_SERIAL.begin(gps_baud);
    log_i("Starting GPS with baud: ", gps_baud);
  }

// init history array
  for( int i = 0; i< HISTORY_LEN; i++){
    history[i].set = false;
  }
  for( int i=0; i< WIND_HIST_LEN; i++){
    wind_history[i].time = 0;
    wind_history[i].gust = 0;
    wind_history[i].dir_raw = 0;
    wind_history[i].wind = 0;
  }
  // create_versionfile(VERSIONFILE); // create version file if not exists (not working)
  if(hw_version == HW_unknown){log_i("Hardware detection failed\n");}
  if(hw_version == HW_1_3){log_i("Detected HW1.3\n");}
  if(hw_version == HW_2_0){log_i("Detected HW2.0\n");}
  log_flush();
  wakeup();
}

// loop ----------------------------------------------------------------------------------------------------------------------

void loop(){


static uint32_t send_active=0; // if > 0, time() last message was send to tx queue, reset to 0 if send is complete
static uint32_t last_settings_check = 0; // timee() ckecked if a settings file is present if settings not read yet sucessfully

// print millis as alive counter
static uint32_t last_call = 0;
static bool s = false;

if(last_call && (time()-last_call > 15)){
  if(!s){led_status(0);}
}
if(usb_connected && time()-last_call > 500){
  //log_i("Time: ", time());
  //Serial.println(millis());
  Serial1.println(millis());
  // Store led states and restore after blink
  s = led_status(1);
  last_call=time();
}



#ifdef HAS_HEATER
  if(is_heater && (hw_version == HW_1_3)){run_heater();}
#endif

  if(is_baro){read_baro();}
  if(is_gps){read_gps();}

  if(fanet_cooldown_ok() && broadcast_interval_name && ( (time()- last_msg_name) > broadcast_interval_name) ){ // once a hour
    if(station_name.length() > 1){
      led_status(1);
      send_name(station_name.c_str(),station_name.length());
      log_i("Send name: "); log_i(station_name.c_str()); log_i("\r\n");
      last_fnet_send = time();
      last_msg_name = time();
      send_active = time();
      led_status(0);
    }
  }

#if 0 // not implemented yet
  if(fanet_cooldown_ok() && broadcast_interval_info && ( (time()- last_msg_info) > broadcast_interval_info) ){
    if(station_name.length() > 1){ // replace with message length
      led_status(1);
      //fanet.sendMSG("Testmessage");
      log_i("Send message: "); log_i("Testmessage");
      last_fnet_send = time();
      last_msg_info = time();
      send_active = time();
      led_status(0);
    }
  }
#endif

  if(fanet_cooldown_ok() && broadcast_interval_weather && ( (time()- last_msg_weather) > broadcast_interval_weather) ){
    if( allowed_to_send_weather() ){
      send_msg_weather();
      last_fnet_send = time();
      last_msg_weather = time();
      send_active = time();
    } else {
      if(last_ws80_data && (time()- last_ws80_data > 7000)){
        log_e("Wdata not ready. Sleep\r\n");
        sleep_allowed = time() + 1;
        last_ws80_data = 0;
      }
      
    }
  }

  if(send_active){
    if( (time()- send_active > (3500))){
      log_i("Send timed out\r\n");
      send_active =0;
      sleep_allowed = time() + (1);
      radio_phy->sleep();
    }

    if(transmittedFlag){
      transmittedFlag = false;
      //log_i("Send complete\r\n");
      send_active = 0;
      sleep_allowed = time() + (1);
      radio_phy->finishTransmit();
      radio_phy->sleep();
    }
  }

// Check if everything is done --> sleep
  if(!send_active && sleep_allowed && (time() > sleep_allowed) && (!usb_connected || test_with_usb) && (time() > 2500)){ // allow sleep after 2500 ms to get a change to detect usb connected
    go_sleep();
  }

  if(!settings_ok){ // Settings not ok. Try few times, then sleep
    if(!sleep_allowed){ 
      sleep_allowed = time() + 180000; // Sleep after 3 minutes
    }
    if(time()- last_settings_check > 15000){
      last_settings_check = time();
      log_e("\r\nFailed to obtain settings from file. Trying again\r\n");
      settings_ok = parse_file(SETTINGSFILE);
      if(settings_ok){ 
        NVIC_SystemReset();      // processor software reset
        }
    }
  }

  // during Dev
  if(usb_connected){
    if(test_with_usb){read_ws80();} // to simulate normal behavior without sleep read and parse data from serial port
    else {forward_ws80_serial();} // otherwise just forward the data
    read_serial_cmd(); // read setting values from serial for testing
    if(!no_sleep && !test_with_usb && (time() > 15*60*1000)){
      usb_connected = false;
      NVIC_SystemReset();
      } // keep usb alive for 15 min
  }

  if(use_wdt){
    wdt_reset();
  }
  
}


// Flash ----------------------------------------------------------------------------------------------------------------------
#define DISK_LABEL  "Breezedude"
FATFS elmchamFatfs;

DSTATUS disk_status(BYTE pdrv){
  (void) pdrv;
	return 0;
}

DSTATUS disk_initialize(BYTE pdrv){
  (void) pdrv;
	return 0;
}

DRESULT disk_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count){
  (void) pdrv;
	return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count){
  (void) pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl (BYTE pdrv,BYTE cmd,	void *buff){
  (void) pdrv;
  switch ( cmd ){
    case CTRL_SYNC:
      flash.syncBlocks();
      return RES_OK;
    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = flash.size()/512;
      return RES_OK;
    case GET_SECTOR_SIZE:
      *((WORD*) buff) = 512;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 8;    // erase block size in units of sector size
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

bool format_flash(){
  uint8_t workbuf[4096]; // Working buffer for f_fdisk function.
  FRESULT r = f_mkfs12("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
     log_i("Error, f_mkfs failed with error code: ", r);
    return false;
  }
  r = f_mount(&elmchamFatfs, "0:", 1);   // mount to set disk label
  if (r != FR_OK) {
     //log_i("Error, f_mount failed with error code: ", r);
    return false;
  }
   // log_i("Setting disk label to: "); log_i(DISK_LABEL); log_i("\r\n");
  r = f_setlabel(DISK_LABEL);
  if (r != FR_OK) {
     //log_i("Error, f_setlabel failed with error code: ", r);
    return false;
  }
  f_unmount("0:"); // unmount
  flash.syncBlocks(); // sync to make sure all data is written to flash
  return true;
}

int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize){
  //Serial.printf("Reading at %d with size %d\n",lba,bufsize);
  my_internal_storage.read(lba*DISK_BLOCK_SIZE, buffer, bufsize);
  return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize){
  //Serial.printf("Writing at %d with size %d\n",lba,bufsize);
  // Erase should be done before every writing to the flash
  my_internal_storage.erase(lba*DISK_BLOCK_SIZE, bufsize);
  // Write to the flash
  my_internal_storage.write(lba*DISK_BLOCK_SIZE, buffer, bufsize);
  return bufsize;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache / sync with flash
void msc_flush_cb (void){
  my_internal_storage.flush_buffer();
}
// Invoked to check if device is writable as part of SCSI WRITE10
// Default mode is writable
bool msc_writable_callback(void){
  // true for writable, false for read-only
  if(!usb_connected){
    log_i("USB Connected: ", time());
  }
  usb_connected = true;
  return true;
}

void setup_usb_msc(){
  log_i("USB MSC Setup\n");
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("BrezeDu", "Mass Storage", "1.0");
   
  // Set disk size
  uint32_t DISK_BLOCK_NUM = my_internal_storage.get_flash_size()/DISK_BLOCK_SIZE;
  //log_i("DISK_BLOCK_NUM: ", DISK_BLOCK_NUM);
  usb_msc.setCapacity(DISK_BLOCK_NUM-1, DISK_BLOCK_SIZE);

  // Set callbacks
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setWritableCallback(msc_writable_callback);

  // Set Lun ready (internal flash is always ready)
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  Serial.begin(115200); // USB Serial
  //while ( !Serial ) delay(10);   // wait for native usb
  delay(10);
}

bool setup_flash(){
  setup_usb_msc();
    //if(debug_enabled){printf("Internal flash with address %p and size %ld\r\n", my_internal_storage.get_flash_address(), my_internal_storage.get_flash_size());}
  if (flash.begin()){
    //log_i("Internal flash successfully set up\r\n");
  }else{
    log_e("Error: failed to set up the internal flash\r\n");
  }

  // The file system object from SdFat to read/write to the files in the internal flash
  int count = 0;
  while ( !fatfs.begin(&flash) ){
    if( (count == 0) && !format_flash()){
      log_e("Error: failed to FAT format flash\r\n");
      return false;
    }
    if(count == 1) {
      log_e("Error: file system not existing. failed to format. The internal flash drive should first be formated with Windows or fdisk on Linux\r\n");
      return false;
    }
    count ++;
  }

#if 0
  DEBUGSER.print("Clusters:          ");
  DEBUGSER.println(fatfs.clusterCount());
  DEBUGSER.print("Blocks x Cluster:  ");
  DEBUGSER.println(fatfs.sectorsPerCluster());
  DEBUGSER.print("Total Blocks:      ");
  DEBUGSER.println(fatfs.sectorsPerCluster() * fatfs.clusterCount());
  
  DEBUGSER.print("Volume type is: FAT");
  DEBUGSER.println(fatfs.fatType(), DEC);
  uint32_t volumesize;
  volumesize = fatfs.sectorsPerCluster();
  volumesize *= fatfs.clusterCount();

  volumesize /= 2;
  DEBUGSER.print("Volume size (Kb):  ");
  DEBUGSER.println(volumesize);

  volumesize = 0;
  volumesize = fatfs.sectorsPerCluster();    // clusters are collections of blocks
  volumesize *= fatfs.freeClusterCount();       // we'll have a lot of clusters

  volumesize /= 2;                           // SD card blocks are always 512 bytes 
  DEBUGSER.print("Free space size (Kb):  ");
  DEBUGSER.println(volumesize);

  //DEBUGSER.println("\nFiles found on the card (name, date and size in bytes): ");
  //fatfs.rootDirStart();
  //// list all files in the card with date and size
  //fatfs.ls(LS_R | LS_DATE | LS_SIZE);
#endif

  return true; 
}

/* Heater voltage/current:
3.5V 280mA
3.9 320mA
3.7 300mA
4.2v 340mA
5V 400mA
8V 650mA
10V 800mA
12V 980mA
*/

// WS80 extended serial output

/*========== WH80 Ver:1.2.5 ===========
>> RF_FreqSel = 868M
>> Device_ID  = 0x00048
-------------------------------------
WindDir      = 338
WindSpeed    = 0.0
WindGust     = 0.8

-------SHT40--------
Temperature  = 24.3
Humi         = 57%

-------Si1132-------
Light        = 2630 lux
UV_Value     = 0.2

Not Detected Pressure Sensor!
Pressure     = --

BatVoltage      = 2.60V
=====================================

=====================================
max = 787, min = 783
max -min = 4
max = 786, min = 783
max -min = 3
max = 788, min = 783
max -min = 5
max = 787, min = 785
max -min = 2
------------------
CH_1 mag. normal
CH_2 mag. normal
CH_3 mag. normal
CH_4 mag. normal
------------------
Vol_CH1_3 = 252
Vol_CH3_1 = 262
Vol_CH4_2 = 264
Vol_CH2_4 = 265
SqWave_CH1_3 = 2
SqWave_CH3_1 = 2
SqWave_CH4_2 = 2
SqWave_CH2_4 = 2
min_index = 0
Min_Voltage = 252
absTv0 = 5
Source_CH1_3 = 100.00,3200
Source_CH3_1 = 99.81,3194
Source_CH4_2 = 99.88,3196
Source_CH2_4 = 99.56,3186
g_UltTimeV01_3 = 99.91,3197
datCnt1_3 = 2
g_UltTimeV04_2 = 99.72,3191
datCnt4_2 = -5
x_y = 53
Get_Cali_Ult_X = 51778
direction = 290
wind = 3

=====================================
max = 787, min = 784
max -min = 3
max = 784, min = 782
max -min = 2
max = 787, min = 783
max -min = 4
max = 789, min = 781
max -min = 8
------------------
CH_1 mag. normal
CH_2 mag. normal
CH_3 mag. normal
CH_4 mag. normal
------------------
Vol_CH1_3 = 254
Vol_CH3_1 = 261
Vol_CH4_2 = 263
Vol_CH2_4 = 263
SqWave_CH1_3 = 2
SqWave_CH3_1 = 2
SqWave_CH4_2 = 2
SqWave_CH2_4 = 2
min_index = 0
Min_Voltage = 254
absTv0 = 5
Source_CH1_3 = 100.06,3202
Source_CH3_1 = 100.00,3200
Source_CH4_2 = 99.91,3197
Source_CH2_4 = 99.78,3193
g_UltTimeV01_3 = 100.03,3201
datCnt1_3 = 6
g_UltTimeV04_2 = 99.84,3195
datCnt4_2 = 1
x_y = 60
Get_Cali_Ult_X = 55200
direction = 9
wind = 4
*/