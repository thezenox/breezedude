#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <BMP280.h>
#include <LibPrintf.h>
#include <FanetLora.h>
#include <SdFat.h>
#include <SAMD_InternalFlash.h>
#include <TinyGPS++.h>

#include "ff.h"
#include "diskio.h"
#include "sleep.h"

// https://github.com/adafruit/Adafruit_TinyUSB_Arduino
// https://github.com/gereic/GXAirCom
// https://github.com/adafruit/ArduinoCore-samd
// https://github.com/Mollayo/SAMD_InternalFlash
// https://github.com/Microsoft/uf2

//Modifications:
// SAMD_InternalFlash.cpp:  
//    _flash_address = (uint8_t *) 0x00021800; // <-- Fixed starting address. Move if program gets to large
//    _flash_address += FLASH_BLOCK_SIZE*8; // <-- move some blocks from end-marking made by linker

// Improvements
// LDO: 
// S-1313D33 1,5µA (200mA max)
// HT7533-1 2.5µA (100mA max)?

//Power consumption:
// Deepsleep 120µA (without WS80 sensor, davis speed and dir open)
// avg @40s send interval: 650µA (with davis 6410)
// avg @40s send interval: 1.65mA (with WS80)

Adafruit_USBD_MSC usb_msc;
#define DISK_BLOCK_SIZE 512 // Block size in bytes for tinyUSB flash drive. Should be always 512
#define BUFFERSIZE 1024 // size of linebuffer

InternalFlash my_internal_storage;
Adafruit_FlashTransport_InternalFlash flashTransport(&my_internal_storage);
Adafruit_InternalFlash_Wrapper flash(&flashTransport);
FatFileSystem fatfs;
TinyGPSPlus tinyGps;

#define BROADCAST_INTERVAL 40*1000 // dafault value,  overwritten by settingfile
#define HISTORY_INTERVAL 30*1000 // 3600*1000 // 1h
#define HISTORY_LEN 15*24 // in hours, 15 days

#define DEBUGSER Serial1
#define WS80_UART Serial1
#define GPS_SERIAL Serial1 // if no WS80 is connected a GPS receiver can be used. Mainly for OGN rangee/coverage check

// SERCOM 4 - SPI RFM95W LoRa
#define PIN_LORA_MISO MISO  // 28 PA12  - SERCOM2/PAD[0] | alt SERCOM4/PAD[0]
#define PIN_LORA_MOSI MOSI  // 29 PB10  -                | alt SERCOM4/PAD[2]
#define PIN_LORA_SCK SCK    // 30 PB11  -                | alt SERCOM4/PAD[3]

#define PIN_LORA_CS 5    // PA15 - SERCOM2/PAD[3] | alt SERCOM4/PAD[3]
#define PIN_LORA_RESET 2 // PA14 - SERCOM2/PAD[2] | alt SERCOM4/PAD[2]
#define PIN_LORA_DIO0 11 // PA16 - SERCOM1/PAD[0] | alt SERCOM3/PAD[0]
#define PIN_LORA_DIO1 10 // PA18 - SERCOM1/PAD[2] | alt SERCOM3/PAD[2]
#define PIN_LORA_DIO2 12 // PA19 - SERCOM1/PAD[3] | alt SERCOM3/PAD[3]
#define PIN_LORA_GPIO -1
#define PIN_PV_CHARGE 38// PB23
#define PIN_PV_DONE 37 // PB22

#define PIN_SERCOM1_TX 40 // PA00 SERCOM1 PAD0
#define PIN_SERCOM1_RX 10 // PA01 SERCOM1 PAD1

// Pins with solder jumpers 
#define PIN_ID0 6 // PA20 - SERCOM5/PAD[2]
#define PIN_ID1 7 // PA21 - SERCOM5/PAD[3]
#define PIN_ID2 3 // PA09 - SERCOM0/PAD[1], ADC

// Serial2 for Degugging on HW > 1.3
Uart Serial2(&sercom1, PIN_SERCOM1_RX, PIN_SERCOM1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0 ) ;

#define HEATER_MAX_V 9.20 // Maximum setpoint voltage, thermal limit by inductor (~1,8A conntiniuos, actual voltage ~8V)
#define PIN_EN_DCDC 19 // A5 19 PB02
#define PIN_EN_HEATER 4 //D4/A8 PA08
#define PIN_DAVIS_SPEED 17 // A3 17 PA04
#define PIN_DAVIS_DIR 18 // A4 18 PA05
#define PIN_DAVIS_POWER 9 // 9

// SERCOM 3 - I²C Barometer / Digipot
#define PIN_SDA 26 // PA22
#define PIN_SCL 27 // PA23

BMP280 bmp; // DPS310 as replacement? or china Goertek SPL06-001

// DAVIS6410 Pinout
// Black - Wind speed contact (closure to ground)
// Red - Ground
// Green - Wind direction pot wiper (20KΩ potentiometer)
// Yellow - Pot supply voltage

// SERCOM 0 - uart serial Sensor/ GPS / Debug
#define PIN_RX 0 // A6 PA11
#define PIN_TX 1 // A7 PA10

#define PIN_STATUSLED 13 // PA17
#define PIN_V_READ_TRIGGER A2 // D16 PB09
#define PIN_V_READ A1 // D15 PB08 - { PORTB,  8, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, // ADC/AIN[2]

#define SETTINGSFILE (char*) "settings.txt"

#define MCP4652_I2C_ADDR	0x2F
#define WRITE_WIPER_DCDC	(0b0000<<4) // Wiper 0 = DCDC
#define WRITE_WIPER_MPPT	(0b0001<<4) // Wiper 1 = MPPT
#define WRITE_TCON	(0b0100<<4)
#define CMD_WRITE	(0b00<<2)

FanetLora fanet;
long frequency = FREQUENCY868;
uint8_t radioChip = RADIO_SX1276;
uint8_t rfMode = 0x03;
// SYC RFMODE=11  set RF-Mode
//              Bit 0 .... FANET Receive enable
//              Bit 1 .... FANET Transmit enable
//              Bit 2 .... Legacy Receive enable
//              Bit 3 .... Legacy Transmit enable
//              default 11 --> Fanet Receive/Transmit and Legacy-Transmit

// Pulsecounter
volatile uint32_t pulsecount =0; // pulses from wind speed sensor using reed switch

// WDT and CPU Clock
#define WDT_PERIOD 8000 // ms for wdt to wait to reset mcu if not reset in time
bool use_wdt = false; // use watchdog reset
int div_cpu = 1; // current div
int div_cpu_fast = 1; // fast CPU clock 1= 48Mhz/1
int div_cpu_slow = 1; // if > 1, cpu clocks down by 48/n Mhz

bool first_sleep = true; // first sleep after reset, USB perephial could still be on
bool usb_connected = false;

uint32_t sleep_allowed = 0; // time() when is is ok so eenter deepsleep
uint32_t sleeptime_cum = 0; // cumulative time spend in sleepmode, used for time() calculation
uint32_t heater_on_time_cum = 0;
#define MAX_HEAT_TIME 30*60*1000 //30 min

// ### Variables for storing settings from file, may be overewritten #####
String station_name="";
float pos_lat = 0;
float pos_lon = 0;
float altitude = -1;
float mppt_voltage = 5.5;
float heater_voltage = 4.5;
int heading_offset = 0;

// Sensor selction
bool is_ws80 = false;
bool is_davis6410 = false;
bool is_bmp280 = false;
bool is_heater = false;
bool is_gps = false;
bool testmode = false; // send without weather station to check lora coverage
uint32_t gps_baud = 9600;

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
bool no_sleep = false;
bool test_heater = false;
bool debug_enabled = true; // enable uart debug messages
bool errors_enabled = true;

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

// Gust History, for data transmission
typedef struct g{
  uint32_t time;
  float val;
} Gust;

#define GUST_AGE 1000*60*15 // 15 min history
#define GUST_HIST_STEP 1000*60*3 //ms history slots, 3min
#define GUST_HIST_LEN 6 // number so slots. should match GUST_AGE / GUST_HIST_STEP
Gust gust_history[GUST_HIST_LEN]; // gust ringbuffer
uint8_t gust_hist_pos = 0; // current position in ringbuffer

// Data histroy, mainly for heater control
typedef struct h{
  bool set = false;
  int8_t wind;
  int8_t temp;
  int8_t humd;
  int8_t light;
  int8_t batt;
  bool pv_charging;
  bool pv_done;
} History;

History history[HISTORY_LEN];
int hist_count =0;
uint32_t last_history =0;

// Function prototypes
bool setup_flash();
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
bool setup_flash();


// Helper ----------------------------------------------------------------------------------------------------------------------

// get current millis since reset, including time spend in deepsleep. Missing time waiting for UART ws80 sensor
uint32_t time(){
  return millis() + sleeptime_cum;
}

void log_i(const char * msg){
  if(debug_enabled){
    DEBUGSER.print(msg);
  }
}
void log_i(const char * msg, uint32_t num){
  if(debug_enabled){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
  }
}

void log_e(const char * msg){
  if(errors_enabled){
    DEBUGSER.print(msg);
  }
}

void log_flush(){
  if(debug_enabled){
    DEBUGSER.flush();
  }
}

void log_ser_begin(){
  if(debug_enabled){
    DEBUGSER.begin(115200*div_cpu);
  }
}


// ssets LED pin to utput and turns it on
void led_on(){
  pinMode(PIN_STATUSLED,OUTPUT);
  digitalWrite(PIN_STATUSLED, 1);
}

// turns off LED and disables pin
void led_off(){
  digitalWrite(PIN_STATUSLED, 0);
  pinDisable(PIN_STATUSLED);
}

// read solderjumers one and disable pins again
uint8_t read_id(){
  uint8_t id= 0;
  pinMode(PIN_ID0, INPUT_PULLUP);
  pinMode(PIN_ID1, INPUT_PULLUP);
  pinMode(PIN_ID2, INPUT_PULLUP);

  id &= digitalRead(PIN_ID0);
  id &= digitalRead(PIN_ID1) << 1;
  id &= digitalRead(PIN_ID2) << 2;

  pinDisable(PIN_ID0);
  pinDisable(PIN_ID1);
  pinDisable(PIN_ID2);
  return id;
}

// process line in 'key=value' format and hand to callback function
bool process_line(char * in, int len, bool (*cb)(char*, char*)){
  #define BUFFLEN 35
  char name [BUFFLEN];
  char value [BUFFLEN];
  memset(name,'\0',BUFFLEN);
  memset(value,'\0',BUFFLEN);
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
      case ' ':  break;
      case '=':  if(oc && (ptr == name) ){ *(ptr+oc) = '\0'; ptr = value; oc = 0;} break; // switch to value
      default:   *(ptr+oc) = in[c]; oc++; break; // copy char
    }
    c++;
  }
  if(oc && (ptr == value)){ // value is set
    return cb(name, value);
  }
  return false;
}

// Adds/updates gust value in ringbuffer
void add_gust_history(float val){
  // Agg slot time is over, switch to next
  if( (time() - gust_history[gust_hist_pos].time) > GUST_HIST_STEP){
    gust_hist_pos++;
    if(gust_hist_pos == GUST_HIST_LEN){
      gust_hist_pos = 0;
    }
    // reset values if already set (overwrite ringbuffer)
    gust_history[gust_hist_pos].val = 0;
    gust_history[gust_hist_pos].time = 0;
  }
  if(val > gust_history[gust_hist_pos].val){
    gust_history[gust_hist_pos].val = val;
    gust_history[gust_hist_pos].time = time();
  }
}

// gets the highest gust value in ringbuffer, not older than age
float get_gust_from_hist(uint32_t age){
  float max_gust = 0;

  uint8_t p = gust_hist_pos;
  for( int i = 0; i < GUST_HIST_LEN; i++){
    if(p == GUST_HIST_LEN){
      p -= GUST_HIST_LEN;
    }
    if( ( gust_history[p].time && (time()- gust_history[p].time) < age) && (gust_history[p].val > max_gust) ){
      max_gust = gust_history[p].val;
    }
    p++;
  }
  return max_gust;
}

// Set measurement value read from WS80 UART
bool set_value(char* key,  char* value){
  //printf("%s = %s\r\n",key, value);
  if(strcmp(key,"WindDir")==0) {wind_dir_raw = atoi(value); return true;}
  if(strcmp(key,"WindSpeed")==0) {wind_speed = atof(value); return true;}
  if(strcmp(key,"WindGust")==0) {wind_gust = atof(value); add_gust_history(wind_gust); return true;}
  if(strcmp(key,"Temperature")==0) {temperature = atof(value); return true;}
  if(strcmp(key,"Humi")==0) {humidity = atoi(value); return true;}
  if(strcmp(key,"Light")==0) {light_lux = atoi(value); return true;}
  if(strcmp(key,"UV_Value")==0) {uv_level = atof(value); return true;}
  if(strcmp(key,"BatVoltage")==0) {
    ws80_vcc = atof(value); 
    last_ws80_data =time();
    return true;
  }
  return false;
}

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
  //log_i("Setting Whiper to: ", (uint32_t) value);
	unsigned char cmd_byte = 0;
	cmd_byte |= (addr | CMD_WRITE);
  //Wire.begin();
	Wire.beginTransmission(MCP4652_I2C_ADDR);
	Wire.write(cmd_byte);
	Wire.write(value);
	Wire.endTransmission();
}

// Write both outputs of digipot
void apply_mcp4652(){
  if(heater_voltage > HEATER_MAX_V){
      heater_voltage = HEATER_MAX_V;
  }
  //log_i("Setting Digipot\r\n"); log_flush();
  mcp4652_write(WRITE_WIPER_DCDC, calc_mt3608(heater_voltage));
  delay(3);
  mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
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
    pinMode(PIN_V_READ, INPUT);
    delay(1);
    digitalWrite(PIN_V_READ_TRIGGER,0);
    delayMicroseconds(30);
    float val = analogRead(PIN_V_READ);
    val += analogRead(PIN_V_READ);
    val += analogRead(PIN_V_READ);
    val += analogRead(PIN_V_READ);
    digitalWrite(PIN_V_READ_TRIGGER,1);
    //pinDisable(PIN_V_READ_TRIGGER); //disable at sleep begin
    val /=4;
    pinDisable(PIN_V_READ);
    //log_i("V_Batt_raw: ", val);
    val *= 0.00432; // 100k/360k 1.0V Vref
    batt_volt = val;

  // gets remapped by fanet (State of Charge  (+1byte lower 4 bits: 0x00 = 0%, 0x01 = 6.666%, .. 0x0F = 100%))
    if( val < 3.5)  {batt_p = 0;}
    if( val > 3.55) {batt_p = 10;}
    if( val > 3.65) {batt_p = 20;}
    if( val > 3.70) {batt_p = 30;}
    if( val > 3.75) {batt_p = 40;}
    if( val > 3.80) {batt_p = 50;}
    if( val > 3.85) {batt_p = 60;}
    if( val > 3.90) {batt_p = 70;}
    if( val > 3.95) {batt_p = 80;}
    if( val > 4.05) {batt_p = 90;}
    if( val > 4.15) {batt_p = 99;}
    if(( val > 4.15) && pv_done) {batt_p = 100;}
  }
    batt_perc =  batt_p;
}

// History ----------------------------------------------------------------------------------------------------------------------
// save history value if time is ready  for a new one
void save_history(){
  static uint32_t last_history = 0;
  if(time() - last_history > HISTORY_INTERVAL){
    last_history = time();
    hist_count++;
    if(hist_count == HISTORY_LEN){hist_count = 0;}
    log_i("Saving Hist no: ", hist_count);
    history[hist_count].set = true;
    history[hist_count].wind = wind_speed*10;
    history[hist_count].temp = temperature;
    history[hist_count].humd = humidity;
    history[hist_count].light = light_lux/100;
    history[hist_count].batt = (batt_volt-2)*100;
    history[hist_count].pv_charging = pv_charging;
    history[hist_count].pv_done = pv_done;
  }
}

// calc sum of light sensor reading in n last ringbuffer entries
uint32_t history_sum_light(int len){
  int sum = 0;
  for (int i=0; i<len; i++){
    int hpos = hist_count-i;
    if(hpos <0){hpos += HISTORY_LEN;} // get pos of ringbuffer

    if(history[hpos].set){
      sum += history[hpos].light;
      if(sum > 2147480000){break;} // abort if very high value
    }
  }
  return sum;
}

// calc sum of wind speed reading in n last ringbuffer entries
uint32_t history_sum_wind(int len){
  int sum = 0;
  for (int i=0; i<len; i++){
    int hpos = hist_count-i;
    if(hpos <0){hpos += HISTORY_LEN;} // get pos of ringbuffer

    if(history[hpos].set){
      sum += history[hpos].wind;
      if(sum > 2147480000){break;} // abort if very high value
    }
  }
  return sum;
}

// Sensors ----------------------------------------------------------------------------------------------------------------------
// request baro sampling & conversion
void baro_start_reading(){
  //sercom3.resetWIRE();
  //Wire.begin();
  next_baro_reading = time() + bmp.startMeasurment();
}

// read value of baro, needs baro_start_reading in advance
void read_bmp280(){
  if(next_baro_reading && (time() > next_baro_reading)){
    uint8_t result;
    double T,P;
    result = bmp.getTemperatureAndPressure(T,P);
    if(result!=0){
      //Wire.end();
      next_baro_reading = 0;
      baro_temp = T;
      //baro = bmp.readPressure() / pow(1.0 - (altitude / 44330.0), 5.255)/100.0;
      if (altitude > -1){
          baro_pressure = (P * pow(1-(0.0065*altitude/(temperature + (0.0065*altitude) + 273.15)),-5.257)); ///100.0;
      } else {
        baro_pressure = P; ///100.0;
      }
    }
  }
}

// read UART and process input buffer, needs to be called periodically until new block is complete (last_ws80_data = time())
void read_ws80(){
  static char buffer [BUFFERSIZE];
  static int co = 0;
  uint32_t last_data = micros();

  while(!last_ws80_data && (micros()- last_data < 500)){
    while (WS80_UART.available()){
      buffer[co] = WS80_UART.read();
      last_data = micros();
      //DEBUGSER.write(buffer[co]);
      if(buffer[co] == '\n'){
        // DEBUGSER.write(buffer, co);
        // DEBUGSER.println();
        process_line(buffer, co, &set_value); // Line complete
        co = -1; // gets +1 below
      }
      co++;
      if(co >= BUFFERSIZE){
        log_e("Buffer size exeeded\r\n");
        return;
      }
    }
  }
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
  
  if(is_davis6410){
    wind_speed = (float) pulses * 1.609 * (2250.0/((float)dmillis+1) ); // avoid div/0
  }
  // ... other analog sensors
  add_gust_history(wind_speed);
  save_history();
}

// Heater ----------------------------------------------------------------------------------------------------------------------
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
    log_i("# Heater Info\r\n");
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

  if(en_heater){
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

// Sleep ----------------------------------------------------------------------------------------------------------------------

// dummy function
void wakeup_2(){
}

// enable interrupt on uart rx pin and wait for data
void sleep_til_serial_data(){
  attachInterruptWakeup(PIN_RX, wakeup_2, CHANGE, false);
  if(use_wdt) {
    wdt_disable();
  }
  sleep(false);
  detachInterrupt(PIN_RX);
  WS80_UART.begin(115200*div_cpu); // begin again, because we used the RX pin as wakeup interrupt.

  if(use_wdt) {
    wdt_enable(WDT_PERIOD,false);
  }
  //Wire.setClock(100000);
}

// called after sleep
void wakeup(){
  // USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; // Re-enable USB, no need, not working?
  log_i("\r\n\r\n\r\n#####################\r\nWakeup: ", time()); 
  log_i("Wakeup_source: "); log_i(wakeup_source_string[wakeup_source]);log_i("\r\n");
  wakeup_source = WAKEUP_NONE;

  get_solar_charger_state();
  if(is_bmp280){
    baro_start_reading(); // request data aquisition, will be read later
  }

// WS80 sensor serial data read. Enable pin interrupt and wait for RX
// sleep until new sensor reading.  (230µA avg, no bmp, no rf module, no sensor)
  if(is_ws80 && settings_ok && (time()> 2500)  && !usb_connected && !no_sleep && !testmode){
    last_ws80_data = 0;
    while(!last_ws80_data){
      sleep_til_serial_data();
      read_ws80();
    }
    save_history();
  }
  pinMode(PIN_V_READ_TRIGGER, OUTPUT); // prepare voltage measurement, charge trigger cap
  digitalWrite(PIN_V_READ_TRIGGER,1);
  sleep_allowed = time() + 6000; // go back to sleep after 6 secs as fallback
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

  fanet.end(); // set RFM95 to sleep
  pinDisable(PIN_V_READ_TRIGGER);

// shut down the USB peripheral
  if(first_sleep){
    USB->DEVICE.CTRLA.bit.ENABLE = 0;                   
    while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
    first_sleep = false;
    usb_connected = false;
    if(set_cpu_div(div_cpu_slow)){ //USB needs 48Mhz clock, as we are finished with USB we can lower the cpu clock now.
      div_cpu = div_cpu_slow;
      DEBUGSER.begin(115200*div_cpu); // F_CPU ist sill 48M, so avery clock needs to by multiplied manually
    }
  }

// if never send a weather msg, assume it was send now
  if(!last_msg_weather){
    last_msg_weather = time();
  }

  int32_t time_to_sleep = calc_time_to_sleep();
  if(!settings_ok){ 
    log_e("No valid settings file. Sleeping forever\r\n");
    time_to_sleep = 0x7FFFFFFF;
    } // if settings not ok sleep forever
  if(!time_to_sleep){ return;} // if time_to_sleep = 0, do not sleep at all

  log_i("Goint to sleep for: ", time_to_sleep);
  log_flush();

// disable wdt during sleep
  if(use_wdt) {
    wdt_disable();
  }

// Using TC4 for hardware pulsecounting on Falling edge on pin PA04 (D17). No interrupts needed.
  if(is_davis6410){ // pulse counting anemometer
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
  } else { // no pulse counting anemometer, no interrupts
    actual_sleep = rtc_sleep_cfg(time_to_sleep);
    sleep(false); // use rtc to sleep
    sleeptime_cum += actual_sleep;
  }
  
  //log_i"sleep for: ", actual_sleep);
  //log_flush();

// If sleep is disabled for debugging, use delay
  if(no_sleep){
    log_i("INSOMNIA enabled, delay() instead");
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
}

// Settings ----------------------------------------------------------------------------------------------------------------------
bool apply_setting(char* settingName,  char* settingValue){
  if(debug_enabled){("%s = %s\r\n",settingName, settingValue);}
  log_flush();
  if(strcmp(settingName,"NAME")==0) {station_name = settingValue; return true;}
  if(strcmp(settingName,"LON")==0) {pos_lon = atof(settingValue); return true;}
  if(strcmp(settingName,"LAT")==0) {pos_lat = atof(settingValue); return true;}
  if(strcmp(settingName,"ALT")==0) {altitude = atof(settingValue); return true;}
  if(strcmp(settingName,"HEATER")==0) {is_heater = atoi(settingValue); return true;}
  if(strcmp(settingName,"V_HEATER")==0) {heater_voltage  = atof(settingValue); return true;}
  if(strcmp(settingName,"V_MPPT")==0) { mppt_voltage = atof(settingValue); return true;}
  if(strcmp(settingName,"HEADING_OFFSET")==0) {heading_offset = atoi(settingValue); return true;}
  
  if(strcmp(settingName,"BROADCAST_INTERVAL_WEATHER")==0) {broadcast_interval_weather = atoi(settingValue)*1000; return true;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_NAME")==0) {broadcast_interval_name = atoi(settingValue)*1000; return true;}
  if(strcmp(settingName,"BROADCAST_INTERVAL_INFO")==0) {broadcast_interval_info = atoi(settingValue)*1000; return true;}

  if(strcmp(settingName,"SENSOR_BMP280")==0) {is_bmp280 = atoi(settingValue); return true;}
  if(strcmp(settingName,"SENSOR_DAVIS6410")==0) {is_davis6410 = atoi(settingValue); return true;}
  if(strcmp(settingName,"SENSOR_WS80")==0) {is_ws80 = atoi(settingValue); return true;}

  if(strcmp(settingName,"SENSOR_GPS")==0) {is_gps = atoi(settingValue); return true;}
  if(strcmp(settingName,"GPS_BAUD")==0) {gps_baud = atoi(settingValue); return true;}

  if(strcmp(settingName,"DEBUG")==0) {debug_enabled = atoi(settingValue); return true;}
  if(strcmp(settingName,"ERRORS")==0) {errors_enabled = atoi(settingValue); return true;}
  if(strcmp(settingName,"INSOMNIA")==0) {no_sleep = atoi(settingValue); return true;}
  if(strcmp(settingName,"TESTMODE")==0) {testmode = atoi(settingValue); return true;}
  if(strcmp(settingName,"WDT")==0) {use_wdt = atoi(settingValue); return true;}
  if(strcmp(settingName,"DIV_CPU_SLOW")==0) {div_cpu_slow = atoi(settingValue); return true;}

// Test commands
  if(strcmp(settingName,"TEST_HEATER")==0) {test_heater = atoi(settingValue); return true;}
  if(strcmp(settingName,"SLEEP")==0) {usb_connected =false; return true;}
  return false;
}

void print_settings(){
  if(debug_enabled){
    DEBUGSER.print("Lon: "); DEBUGSER.println(pos_lon);
    DEBUGSER.print("Latt: "); DEBUGSER.println(pos_lat);
    DEBUGSER.print("Alt: "); DEBUGSER.println(altitude);
    DEBUGSER.print("Heater Voltage: "); DEBUGSER.println(heater_voltage);
    DEBUGSER.print("MPPT Voltage: "); DEBUGSER.println(mppt_voltage);
    DEBUGSER.print("Heading Offset: "); DEBUGSER.println(heading_offset); 
    DEBUGSER.print("BROADCAST_INTERVAL: "); DEBUGSER.println(broadcast_interval_weather);
    DEBUGSER.flush();
  }
}
void print_data(){
  if(debug_enabled){
    DEBUGSER.print("\r\nmillis: "); DEBUGSER.println(millis()); 
    DEBUGSER.print("time: "); DEBUGSER.println(time()); 
    DEBUGSER.print("Wind dir_raw: "); DEBUGSER.println(wind_dir_raw);
    DEBUGSER.print("Wind Heading: "); DEBUGSER.println(wind_heading);
    DEBUGSER.print("Wind Speed: "); DEBUGSER.println(int(wind_speed));
    DEBUGSER.print("Wind Gust: "); DEBUGSER.println(int(wind_gust));
    DEBUGSER.print("Temp: "); DEBUGSER.println(int(temperature));
    DEBUGSER.print("Humd: "); DEBUGSER.println(humidity);
    if(is_bmp280){
    DEBUGSER.print("Baro: "); DEBUGSER.println(int(baro_pressure));
    DEBUGSER.print("PCB_Temp: "); DEBUGSER.println(int(baro_temp));
    }
    if(is_ws80) {
      DEBUGSER.print("VCC: "); DEBUGSER.println(int(ws80_vcc*100));
      DEBUGSER.print("LUX: "); DEBUGSER.println(light_lux);
      DEBUGSER.print("UV: "); DEBUGSER.println(uv_level);
    }
    DEBUGSER.print("\r\n");
    DEBUGSER.print("V_Bat: "); DEBUGSER.println(int(batt_volt*100));
    DEBUGSER.print("Bat_perc: "); DEBUGSER.println(batt_perc);
    DEBUGSER.print("PV_charge: "); DEBUGSER.println(pv_charging);
    DEBUGSER.print("PV_done: "); DEBUGSER.println(pv_done);
  }
}

// parse settingsfile
bool parse_file(char * filename){
  bool ret = false;
  led_on();
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
        while (filesize - c > 0) {
          filebuffer[co] = f.read();

          if(filebuffer[co] == '\n'){
            process_line(filebuffer, co, &apply_setting); // Line complete
            co=-1; // gets +1 below
          }
          c++;
          co++;
          if(co >= BUFFERSIZE){
            return false;
          }
        }
        //DEBUGSER.println("End of file");
        process_line(filebuffer, co, &apply_setting);
        f.close();
        //DEBUGSER.println("file closed");

        ret = true;
        led_off(); // if LED stay on, settings failed
    }else {
      log_i("File not exists\r\n");
    }
    my_internal_storage.flush_buffer(); // sync with flash
  } else {
    log_e("Failed to start filesystem\r\n");
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
    if(buffer[co] == '\n'){
      ok = process_line(buffer, co, &apply_setting); // Line complete
      co=-1; // against +1 below
    }
    co++;
  }
  if(ok){
    // Apply new mppt voltage
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    //apply_mcp4652(); // set voltages
    //Serial.println("V set");
  }
}

// Send ----------------------------------------------------------------------------------------------------------------------

void send_msg_weather(){
  led_on();
  fanet.setRFMode(rfMode);
  fmac.setRegion(pos_lat,pos_lon);

  wind_gust = get_gust_from_hist(GUST_AGE);
  wind_heading = wind_dir_raw + heading_offset;
  if(wind_heading > 359){ wind_heading -=360;}
  if(wind_heading < 0){ wind_heading +=360;}
  FanetLora::weatherData fanetWeatherData;
  fanetWeatherData.lat = pos_lat;
  fanetWeatherData.lon = pos_lon;
  fanetWeatherData.bWind = true;
  fanetWeatherData.wHeading = wind_heading;
  fanetWeatherData.wSpeed = wind_speed;
  fanetWeatherData.wGust = wind_gust;      
  fanetWeatherData.bTemp = true;
  fanetWeatherData.temp = temperature;
  fanetWeatherData.bHumidity = true;
  fanetWeatherData.Humidity = humidity;
  if(is_bmp280){
    fanetWeatherData.bBaro = true;
    fanetWeatherData.Baro = baro_pressure;    
  } else {
    fanetWeatherData.bBaro = true;
    fanetWeatherData.Baro = -1;  
  }
  fanetWeatherData.bStateOfCharge = true;
  fanetWeatherData.Charge = batt_perc;

  if(testmode){
    fanetWeatherData.bBaro = true;
    fanetWeatherData.Baro = -1;  
    fanetWeatherData.wHeading = 123;
    fanetWeatherData.wSpeed = 5;
    fanetWeatherData.wGust = 7;      
    fanetWeatherData.temp = 10;
    fanetWeatherData.Humidity = 15;
    log_i("\r\nTESTMODE - Fake values\r\n");
  }

  log_i("\r\nSending Weather\r\n");
  fanet.writeMsgType4(&fanetWeatherData);
  print_data();
  led_off();
}

// check if everything is ok to send the wather data now
bool allowed_to_send_weather(){
  bool ok = settings_ok;

  if (ok){
    if(is_bmp280){ok &= (next_baro_reading == 0);}
    if(is_ws80) { ok &= ((last_ws80_data && (time()- last_ws80_data < 1000)) || testmode); } // only send if weather data is up to date or testmode is enabled
    if(is_gps)  { ok &= (tinyGps.location.isValid()); } // only send if position is valid
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

// Setup ----------------------------------------------------------------------------------------------------------------------

void setup(){
  DEBUGSER.begin(115200); // on boot start with 48Mhz clock
  printf_init(DEBUGSER);
  log_i("\r\n--------------- RESET -------------------\r\n");
  log_i("Version: ");  log_i(VERSION); log_i("\r\n");
  log_i("FW Build Time: ");  log_i(__DATE__); log_i(" "); log_i(__TIME__); log_i("\r\n");
  Wire.begin();

  fanet.autobroadcast = false;
  fanet.setRFMode(rfMode);
  fanet.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS,PIN_LORA_RESET, PIN_LORA_DIO0,PIN_LORA_GPIO,frequency,14,radioChip);
  // pins from line above will be ignored, changed SPI.begin()
  printf("FANET ID: %02X%04X\r\n",fmac.myAddr.manufacturer,fmac.myAddr.id);

  if(setup_flash()){
    settings_ok = parse_file(SETTINGSFILE);
    if(!debug_enabled){
      DEBUGSER.println("Debug messages disabled");
    }
  }

  if(settings_ok){
    // Add altitude to station name, gets splittet by breezedude ogn parser
      if(altitude > -1){
        station_name+" (" + String(altitude) + "m)"; // Testation (1234m)
      }
      setup_PM(is_davis6410); // powermanagement add || other sensors using counter
      if(use_wdt) {
        wdt_enable(WDT_PERIOD,false);
      }
    if(is_bmp280){
      if(!bmp.begin()){
        is_bmp280 = false;
        log_e("BMP280 not found\r\n");
      }else{
        bmp.setOversampling(4);
        log_i("BMP280 setup ok\r\n");
      }
    }
    mcp4652_write(WRITE_WIPER_MPPT, calc_cn3791(mppt_voltage));
    print_settings();
    apply_mcp4652();
  }
  if(is_ws80){
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
  wakeup();
}

// loop ----------------------------------------------------------------------------------------------------------------------

void loop(){
static uint32_t send_active=0; // if > 0, time() last message was send to tx queue, reset to 0 if send is complete
static uint32_t last_settings_check = 0; // timee() ckecked if a settings file is present if settings not read yet sucessfully

// print millis as alive counter
static uint32_t last_call = 0;
if(time()-last_call > 1000){
  log_i("Time: ", time());
  last_call=time();
}

  if(is_heater){run_heater();}
  if(is_bmp280){read_bmp280();}
  if(is_gps){read_gps();}

  if(fanet_cooldown_ok() && broadcast_interval_name && ( (time()- last_msg_name) > broadcast_interval_name) ){ // once a hour
    if(station_name.length() > 1){
      led_on();
      fanet.setRFMode(rfMode);
      fmac.setRegion(pos_lat,pos_lon);
      fanet.sendName(station_name);
      log_i("Send name: "); log_i(station_name.c_str()); log_i("\r\n");
      last_fnet_send = time();
      last_msg_name = time();
      send_active = time();
      led_off();
    }
  }

  if(fanet_cooldown_ok() && broadcast_interval_info && ( (time()- last_msg_info) > broadcast_interval_info) ){
    if(station_name.length() > 1){ // replace with message length
      led_on();
      fanet.setRFMode(rfMode);
      fmac.setRegion(pos_lat,pos_lon);
      fanet.sendMSG("Testmessage");
      log_i("Send message: "); log_i("Testmessage");
      last_fnet_send = time();
      last_msg_info = time();
      send_active = time();
      led_off();
    }
  }

  if(fanet_cooldown_ok() && broadcast_interval_weather && ( (time()- last_msg_weather) > broadcast_interval_weather) ){
    if( allowed_to_send_weather() ){
      read_batt_perc();
      send_msg_weather();
      last_fnet_send = time();
      last_msg_weather = time();
      send_active = time();
    }
  }

  if(send_active){
    if( (time()- send_active > (3500))){
      log_i("Send timed out\r\n");
      send_active =0;
      sleep_allowed = time() + (20);
    }

    if(fanet.checkSendComplete()){
      log_i("Send complete\r\n");
      send_active = 0;
      sleep_allowed = time() + (20);
    }
  }
  fanet.run();

// Check if everything is done --> sleep
  if(sleep_allowed && (time() > sleep_allowed) && !usb_connected && (time() > 2500)){ // allow sleep after 2500 ms to get a change to detect usb connected
    sleep_allowed = 0;
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
        sleep_allowed = 0; 
        NVIC_SystemReset();      // processor software reset
        }
    }
  }

  // during Dev
  if(usb_connected){
    read_serial_cmd(); // read setting values from serial for testing
    if(time() > 15*60*1000){ usb_connected = false;} // keep usb alive for 15 min
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
  FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
     //log_i("Error, f_mkfs failed with error code: ", r);
    return false;
  }
  r = f_mount(&elmchamFatfs, "0:", 1);   // mount to set disk label
  if (r != FR_OK) {
     //log_i("Error, f_mount failed with error code: ", r);
    return false;
  }
   //log_i("Setting disk label to: "); log_i(DISK_LABEL); log_i("\r\n");
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
    log_i("USB Connected", time());
  }
  usb_connected = true;
  return true;
}

bool setup_flash(){
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

  //if(debug_enabled){printf("Internal flash with address %p and size %ld\r\n", my_internal_storage.get_flash_address(), my_internal_storage.get_flash_size());}
  if (flash.begin()){
    //log_i("Internal flash successfully set up\r\n");
  }else{
    log_e("Error: failed to set up the internal flash\r\n");
  }
    
  // The file system object from SdFat to read/write to the files in the internal flash
  if ( !fatfs.begin(&flash) ){
    if(!format_flash()){
      log_e("Error: failed to FAT format flash\r\n");
    }
    if(!fatfs.begin(&flash)){
      log_e("Error: file system not existing. failed to format. The internal flash drive should first be formated with Windows or fdisk on Linux\r\n");
    } else {
    return true;
  }
  } else {
    return true;
  }
  return false;
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