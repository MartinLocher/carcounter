#include "esp32/ulp.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "soc/soc_ulp.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

#define HAS_BATTERY_PROBE ADC1_GPIO35_CHANNEL // battery probe GPIO pin -> ADC1_CHANNEL_7
#define BATT_FACTOR 2 // voltage divider 100k/100k on board

#include "ulp_main.h"
#include "CayenneLPP.h"
#include "battery.h"

// LMIC
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRa Pins
#define LoRa_RST  14  // GPIO 14
#define LoRa_CS   18  // GPIO 18
#define LoRa_DIO0 26  // GPIO 26

#define TRANSMIT_POWER  14


//HELTECV1 & TTBEAM
#define HELTECV1
#ifdef  HELTECV1
#define LoRa_DIO1 33  // GPIO 33
#define LoRa_DIO2 32  // GPIO 32
#endif

//#define HELTECV2
#ifdef HELTECV2
#define LoRa_DIO1 34  // GPIO 33
#define LoRa_DIO2 35  // GPIO 32
#endif


#define LORA_MSG_PORT 67
#define NO_STARTUP_BLINKS 5
#define STARTUP_BLINK_PERIOD  300

#define SLEEP_MODE

//#define BIBERNODE1
#ifdef BIBERNODE1
static const PROGMEM u1_t NWKSKEY[16] = { 0x21, 0x89, 0xF5, 0x50, 0x64, 0x06, 0x3B, 0xA3, 0x67, 0xB8, 0x71, 0x80, 0x63, 0x6C, 0xB2, 0xA1 };
static const u1_t PROGMEM APPSKEY[16] = { 0x71, 0x8A, 0xD7, 0xD0, 0x17, 0x67, 0x5D, 0xF2, 0xC2, 0x11, 0xC4, 0x71, 0x3A, 0x97, 0x4D, 0x7E };
static const u4_t DEVADDR = 0x260118FF ;
#endif


//#define BIBERNODE2
#ifdef BIBERNODE2
static const u1_t NWKSKEY[16] = { 0xA1, 0x90, 0xE6, 0x58, 0xD0, 0x5A, 0x1E, 0x1B, 0x99, 0x84, 0x6C, 0xD0, 0xD1, 0x97, 0xFD, 0x1C };
static const u1_t APPSKEY[16] = { 0x81, 0xB1, 0x18, 0x2A, 0xA1, 0x90, 0x82, 0xD4, 0xD5, 0xDA, 0x3F, 0x48, 0xF3, 0x6C, 0xCF, 0x56 };
static const u4_t DEVADDR = 0x260115FF;
#endif

//#define BIBERNODE3
#ifdef BIBERNODE3
static const u1_t NWKSKEY[16] = { 0xB1, 0xCC, 0xDC, 0xB1, 0x65, 0x7A, 0x04, 0x90, 0x44, 0x42, 0x14, 0x6D, 0x47, 0xA1, 0xE5, 0xB6 };
static const u1_t APPSKEY[16] = { 0x51, 0x44, 0x94, 0xDC, 0x86, 0xEB, 0xB9, 0xF9, 0x2C, 0xF5, 0x69, 0xF3, 0x42, 0xA9, 0x5B, 0x13 };
static const u4_t DEVADDR = 0x26011FFF;
#endif

//#define BIBERNODE4
#ifdef BIBERNODE4
static const u1_t NWKSKEY[16] = { 0x31, 0xD4, 0xF6, 0xC7, 0x2D, 0x09, 0xBD, 0x5C, 0x4C, 0xAE, 0x79, 0x23, 0x01, 0x47, 0x21, 0xDB };
static const u1_t APPSKEY[16] = { 0x50, 0x5C, 0xB8, 0xE4, 0x75, 0x2D, 0x55, 0xC2, 0xF4, 0x93, 0x84, 0xDA, 0x86, 0x98, 0xEB, 0xF1 };
static const u4_t DEVADDR = 0x26011CFF;
#endif

//#define BIBERNODE5
#ifdef BIBERNODE5
static const u1_t NWKSKEY[16] = { 0x11, 0x68, 0x5D, 0x96, 0xDC, 0x5A, 0xA0, 0xAD, 0xEA, 0x81, 0x95, 0xFE, 0x12, 0x1F, 0xC0, 0xA0 };
static const u1_t APPSKEY[16] = { 0x61, 0x77, 0xFF, 0x09, 0xAF, 0xA2, 0x98, 0x89, 0x6A, 0xFD, 0xD7, 0x0C, 0xC2, 0x55, 0x39, 0xDC };
static const u4_t DEVADDR = 0x260116FF;
#endif


#define BIBERNODE6
#ifdef BIBERNODE6
//msb
static const u1_t NWKSKEY[16] = { 0x71, 0xB3, 0x0E, 0xE9, 0xC4, 0x14, 0xED, 0x27, 0x99, 0xF7, 0x1C, 0xCE, 0xA1, 0x64, 0x64, 0xB9 };
static const u1_t APPSKEY[16] = { 0x21, 0x2F, 0x85, 0x71, 0x0A, 0x34, 0xE2, 0x40, 0xA8, 0xCB, 0xFE, 0xA0, 0x46, 0xAA, 0x2A, 0x32 };
//just the number
static const u4_t DEVADDR = 0x26011FFF;
#endif

//#define OTAA
#ifdef OTAA
#define BIBERNODE6
#ifdef BIBERNODE6
// little endian
static const u1_t PROGMEM APPEUI[8] = { 0xF1, 0x3E, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
// little endian
static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x11, 0x32, 0x94, 0x22, 0x28, 0x54, 0x07 };
// msb
static const u1_t PROGMEM APPKEY[16] = { 0xFF, 0x11, 0x6F, 0x6D, 0x86, 0x16, 0xF6, 0xE4, 0x9F, 0x01, 0x52, 0x21, 0xF0, 0xAE, 0x14, 0xD8 };
#endif

void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
#else
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

#ifdef BIBERNODE1
RTC_DATA_ATTR byte dev_unique_id = 0x20;
#endif

#ifdef BIBERNODE2
RTC_DATA_ATTR byte dev_unique_id = 0x21;
#endif

#ifdef BIBERNODE3
RTC_DATA_ATTR byte dev_unique_id = 0x22;
#endif

#ifdef BIBERNODE4
RTC_DATA_ATTR byte dev_unique_id = 0x23;
#endif

#ifdef BIBERNODE5
RTC_DATA_ATTR byte dev_unique_id = 0x24;
#endif

#ifdef BIBERNODE6
RTC_DATA_ATTR byte dev_unique_id = 0x25;
#endif

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = LoRa_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LoRa_RST,
  .dio = { LoRa_DIO0, LoRa_DIO1, LoRa_DIO2 },
};

// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void init_run_ulp(uint32_t usec);
long pulse_count;
uint32_t pulse_count_from_ulp;
RTC_DATA_ATTR int boot_count = 0;
RTC_DATA_ATTR int sf  = 10;
#define SECS_PER_MIN  60

RTC_DATA_ATTR  int deep_sleep_sec = 10; //60 * 5; // Sleep every five minutes

const unsigned LINGER_TIME = 5000;

CayenneLPP lpp(51);

void do_deepsleep(osjob_t *arg)
{
  // Enter deep sleep
  printf("Entering deep sleep for %d seconds\n", deep_sleep_sec);
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
  delay (100);
  esp_deep_sleep(1000000LL * deep_sleep_sec);
}
void do_send(osjob_t *arg)
{
  lpp.reset();


  printf("do_send() called! ... Sending data!\n");
  printf("sf: %d\n", sf);
  printf("Voltage: %f\n", (float)read_voltage() / 1000);
  printf("Sum cars count: %f\n", (float)(pulse_count));
  printf("Cur. car count: %f\n", (float)(pulse_count_from_ulp));
  lpp.addAnalogInput(1, pulse_count);
  lpp.addAnalogInput(2, pulse_count_from_ulp);
  lpp.addAnalogInput(3, (float)read_voltage() / 1000);

  if (LMIC.opmode & OP_TXRXPEND)
  {
    printf("OP_TXRXPEND, not sending!\n");
  }
  else
  {
    LMIC_setTxData2(LORA_MSG_PORT, lpp.getBuffer(), lpp.getSize(), 0);
    printf("Packet queued\n");
  }
}

void set_car_count(byte cars)
{
  printf("!!!!!*****CarCount now:%d \n", cars);
  reset_puls_count(cars);
}

void set_sleep(byte length_min)
{
  printf("!!!!!*****Sleeping now for: %d mins\n", length_min);
  deep_sleep_sec = length_min * SECS_PER_MIN;
}

void set_sf (byte new_sf)
{
  printf("!!!!!*****SF change :%d \n", new_sf);
  sf = new_sf;
}
void  set_transmit_SF(byte spread_factor)
{
  printf("!!!!!*****SF set change :%d \n", spread_factor);
  switch (spread_factor)
  {
    case 7:
      LMIC_setDrTxpow(DR_SF7, TRANSMIT_POWER);
      break;

    case 8:
      LMIC_setDrTxpow(DR_SF8, TRANSMIT_POWER);
      break;

    case 9:
      LMIC_setDrTxpow(DR_SF9, TRANSMIT_POWER);
      break;

    case 10:
      LMIC_setDrTxpow(DR_SF10, TRANSMIT_POWER);
      break;

    case 11:
      LMIC_setDrTxpow(DR_SF11, TRANSMIT_POWER);
      break;

    case 12:
      LMIC_setDrTxpow(DR_SF12, TRANSMIT_POWER);
      break;
  }
}


// Callbacks from lmic, needs C linkage
void onEvent(ev_t ev)
{
  printf("Event Time: %lld, %d\n", os_getTime(), ev);
  switch (ev)
  {
    case EV_TXCOMPLETE:
      printf("EV_TXCOMPLETE (includes waiting for RX windows)\n");
      if (LMIC.txrxFlags & TXRX_ACK)
        printf("Received ack\n");
      if (LMIC.dataLen) {
        printf("Received Data\n");
        // on/off cmds should be coded bitwise in just one byte not by using a complete array  !!!!
        // future release
        // 0A 01 0A: SF10, 1 min sleep,
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        for (int i = 0; i < LMIC.dataLen; i++)
        {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
        }
        Serial.println();
        // transmit_sf, sleep_min, car_count
        switch (LMIC.dataLen)
        {
          case 1:
            set_sf(LMIC.frame[LMIC.dataBeg + 0]) ;
            break;

          case 2:
            set_sf(LMIC.frame[LMIC.dataBeg + 0]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 1]);
            break;

          case 3:
            set_sf(LMIC.frame[LMIC.dataBeg + 0]);
            set_sleep(LMIC.frame[LMIC.dataBeg + 1]);
            set_car_count(LMIC.frame[LMIC.dataBeg + 2]);
            break;
        }
      }

      //do_deepsleep(&LMIC.osjob);
      os_setTimedCallback(&LMIC.osjob, os_getTime() + ms2osticks(LINGER_TIME), FUNC_ADDR(do_deepsleep));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      printf("EV_RXCOMPLETE\n");
      break;
    default:
      printf("Unknown event: %d\n", ev);
      break;
  }
}


void setup()
{
  Serial.begin(115200);
  delay(1000);

  calibrate_voltage();


  // ulp variables data is the lower 16 bits
  if (boot_count == 0)
  {
    printf("Init ULP\n");
    init_ulp_program();
    printf("Nothing to send!!!\n");
    os_setTimedCallback(&LMIC.osjob, os_getTime() + ms2osticks(LINGER_TIME), FUNC_ADDR(do_deepsleep));

    boot_count ++;
  }
  else
  {
    printf("Timer wakeup, saving pulse count\n");
    update_pulse_count();


    if (pulse_count_from_ulp > 0 )
    {
      boot_count ++;

      os_init();

      LMIC_reset();

      uint8_t appskey[sizeof(APPSKEY)];
      uint8_t nwkskey[sizeof(NWKSKEY)];
      memcpy(appskey, APPSKEY, sizeof(APPSKEY));
      memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
      LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#define CFG_eu868

#if defined(CFG_eu868)
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
#elif defined(CFG_us915)
      LMIC_selectSubBand(0);
#endif

      // LMIC_setLinkCheckMode(0);
      LMIC.dn2Dr = DR_SF9;
      set_transmit_SF (sf);

      // Disable channel 1 to 8
      for (int i = 1; i <= 8; i++)
        LMIC_disableChannel(i);
      LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);
      delay(100);
      LMIC.seqnoUp = boot_count;

      printf("Sending TTN Data!!!\n");
      do_send(NULL);

      delay(100);
    }
    else
    {
      printf("Nothing to send!!!\n");
      os_setTimedCallback(&LMIC.osjob, os_getTime() + ms2osticks(LINGER_TIME), FUNC_ADDR(do_deepsleep));
    }
  }
}

void loop ()
{
  os_runloop_once();
}

void reset_puls_count(unsigned char init_val)
{
  const char *p_namespace = "pulsecnt";
  const char *count_key = "count";

  ESP_ERROR_CHECK(nvs_flash_init());
  nvs_handle handle;
  ESP_ERROR_CHECK(nvs_open(p_namespace, NVS_READWRITE, &handle));

  esp_err_t err = nvs_get_u32(handle, count_key, (uint32_t*)&pulse_count);
  assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);

  /* Save the new pulse count to NVS */
  pulse_count = init_val;
  printf("!!!!*****INIT pulse_count now: %5d\n", pulse_count);

  ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
  ESP_ERROR_CHECK(nvs_commit(handle));
  nvs_close(handle);
  printf("****Wrote updated pulse count to NVS: %5d ****\n", pulse_count);
}

void init_ulp_program()
{
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                  (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);


  /* GPIO used for pulse counting. */
  gpio_num_t gpio_num = GPIO_NUM_39;
  assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

  /* Initialize some variables used by ULP program.
     Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     These variables are declared in an auto generated header file,
     'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     These variables are located in RTC_SLOW_MEM and can be accessed both by the
     ULP and the main CPUs.

     Note that the ULP reads only the lower 16 bits of these variables.
  */
  ulp_debounce_counter = 3;
  ulp_debounce_max_count = 3;
  ulp_next_edge = 0;
  ulp_io_number = rtc_gpio_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */
  ulp_edge_count_to_wake_up = 10000;

  /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_num);
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_hold_en(gpio_num);

  /* Disconnect GPIO12 and GPIO15 to remove current drain through
     pullup/pulldown resistors.
     GPIO12 may be pulled high to select flash voltage.
  */
  rtc_gpio_isolate(GPIO_NUM_12);
  rtc_gpio_isolate(GPIO_NUM_15);
  //esp_deep_sleep_disable_rom_logging(); // suppress boot messages

  /* Set ULP wake up period to T = 20ms.
     Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
  */
  ulp_set_wakeup_period(0, 20000);

  /* Start the program */
  err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
  ESP_ERROR_CHECK(err);
}

void update_pulse_count()
{
  const char *p_namespace = "pulsecnt";
  const char *count_key = "count";

  ESP_ERROR_CHECK(nvs_flash_init());
  nvs_handle handle;
  ESP_ERROR_CHECK(nvs_open(p_namespace, NVS_READWRITE, &handle));

  nvs_get_u32(handle, count_key, (uint32_t*) &pulse_count);

  printf("Read pulse count from NVS: %5d\n", pulse_count);

  /* ULP program counts signal edges, convert that to the number of pulses */
  pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
  /* In case of an odd number of edges, keep one until next time */
  ulp_edge_count = ulp_edge_count % 2;
  printf("Pulse count from ULP: %5d\n", pulse_count_from_ulp);

  /* Save the new pulse count to NVS */
  pulse_count += pulse_count_from_ulp;
  printf("!!!!*****pulse_count now: %5d\n", pulse_count);

  ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
  ESP_ERROR_CHECK(nvs_commit(handle));
  nvs_close(handle);
  printf("****Wrote updated pulse count to NVS: %5d ****\n", pulse_count);
}
