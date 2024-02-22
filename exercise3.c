#include "contiki.h"  // Contiki core
#include "nrf52-ble.h"
#include "app_api.h"     
#include "arch/cpu/nrf52840/lib/packetcraft/wsf/include/util/bstream.h"
#include "ble-events.h"   
#include "nrf_radio.h"
#include "ll_api.h"

#include "gatt.h"
#include "dw1000.h"
#include "dw1000-arch.h"
#include "dev/radio/dw1000/dw1000-ranging.h"

#include "scanreport_buffer.h"

#include <math.h>
#include <stdlib.h>
#include "dev/leds.h"
#include "dev/button-hal.h"
#include "buzzer.h"
#include "node-id.h"

#include <string.h>

# define LOG_COLOR_DBG TC_GREEN
# include "sys/log.h"
# define LOG_MODULE "APP"
# define LOG_LEVEL LOG_LEVEL_INFO

/*---------------------------------------------------------------------------*/
#define ATT_UUID_CONTACT_TRACKER_SERVICE                     0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x23, 0x14, 0x00, 0x00
/*---------------------------------------------------------------------------*/
/* Declare global variable */


/*! advertising data, discoverable mode */
uint8_t myAppAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! service UUID list */
  17,                                      /*! length */
  DM_ADV_TYPE_128_UUID,                    /*! AD type */
  ATT_UUID_CONTACT_TRACKER_SERVICE,

  /*! device name */
  6,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                  /*! AD type */
  'D',
  'E',
  'V',
  'n',
  '0',

  2,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  0,                                      /*0 = neutral, 1 = contaminated*/   
};


// buttons
static button_hal_button_t *left_button;
static button_hal_button_t *right_button;

// timers
#define SCANNER_PERIOD  (CLOCK_SECOND * 15)
#define MINUTE          (CLOCK_SECOND * 60)
#define RANGING_TIMEOUT (CLOCK_SECOND / 8)
static struct etimer scanner_timer;
static struct etimer contamination_timer;
static struct etimer timeout_timer;

/* Declare ContikiNG process and set it to start on boot. */
/*---------------------------------------------------------------------------*/
PROCESS(contact_tracker_process, "Contact tracker process");
PROCESS(scanner_process, "Scanning process");
PROCESS(best_friends_process, "Best friends process");
PROCESS(contamination_process, "Contamination process");
AUTOSTART_PROCESSES(&contact_tracker_process); 

/* Declare ContikiNG process protothread. */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(contact_tracker_process, ev, data)
{

  PROCESS_EXITHANDLER()  // Cleanup code (if any)
  PROCESS_BEGIN();

  left_button = button_hal_get_by_id(1);

  // wait until BLE stack is initialized
  PROCESS_WAIT_EVENT_UNTIL(ble_reset_event);
  LOG_INFO("Contact tracker started!\n");
  
  dw1000_driver.init();
  dw1000_driver.on();

  leds_on(LEDS_GREEN);
  PROCESS_WAIT_EVENT_UNTIL(ev == button_hal_press_event && data == left_button);
  leds_off(LEDS_GREEN);

  // start advertisement
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
  LlSetAdvTxPower(0);
  AppAdvStart(APP_MODE_AUTO_INIT);

  process_start(&scanner_process, NULL);

  while(1) {

    etimer_set(&scanner_timer, SCANNER_PERIOD);
    while(1) {
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&scanner_timer) || (ev == button_hal_press_event && data == left_button));
      if(etimer_expired(&scanner_timer)) {
        break;
      }
      // when left button is pressed the scan process in suspended
      // wait until left button is pressed again
      leds_on(LEDS_GREEN);
      PROCESS_WAIT_EVENT_UNTIL(ev == button_hal_press_event && data == left_button);
      leds_off(LEDS_GREEN);
    }

    startScan();
    
  }

  PROCESS_END();

}



struct device_t {
  uint8_t name[6];
  int8_t rssi;
  double distance;
};

struct device_t table[15];
uint8_t device_scan_counter[15] = {0};
uint8_t device_num = 0;


int16_t fractional_part(float x){
  int16_t fp = (x - (int32_t) x) * 1000000;
  if(fp < 0) return -fp;
  return fp;
}

int8_t contaminated_id = -1;
uint8_t contamination_time;
static process_event_t contaminated_event;
static process_event_t table_update_event;

PROCESS_THREAD(scanner_process, ev, data) {
  
  PROCESS_BEGIN();

  contaminated_event = process_alloc_event();
  table_update_event = process_alloc_event();

  struct scanbuffer *buffer_head;
  static uint8_t *pData;
  static uint8_t *pData2;
  static uint8_t *name;
  static uint8_t index;
  static uint8_t in;
  static uint8_t first_scan = 1;
  static uint8_t *contaminated;
  static ranging_data_t *d;
  static bool status;
  
  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == ble_scan_start_event);
    leds_on(LEDS_BLUE);

    PROCESS_WAIT_EVENT_UNTIL(ev == ble_scan_stop_event);
    leds_off(LEDS_BLUE);

    // after first scan enable best friends log
    if(first_scan == 1) {

      process_start(&best_friends_process, NULL);
      process_start(&contamination_process, NULL);
      first_scan = 0;

    }


    // go through buffer and make a table
    while(1) {

      buffer_head = get_element();
      // if at the end of the buffer
      if(buffer_head == NULL) {
        break;
      }

      pData = DmFindAdType(DM_ADV_TYPE_128_UUID,  buffer_head->len, buffer_head->pData);
      pData2 = DmFindAdType(DM_ADV_TYPE_128_UUID,  sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
      if(memcmp(pData, pData2, sizeof(pData)) == 0) {
        
        name = DmFindAdType(DM_ADV_TYPE_LOCAL_NAME, buffer_head->len, buffer_head->pData);

        index = device_num;
        in = 0;
        for(int i = 0; i < device_num; i++) {

          //LOG_INFO("Name %s!\n", &name[2]);
          //LOG_INFO("Device %s!", table[i].name);

          if(memcmp(&name[2], &table[i].name, sizeof(table[device_num].name) - 1) == 0) {
            index = i;
            in = 1;
            break;
          }

        }

        if(index < 15) {
          memcpy(&table[index].name, &name[2], sizeof(table[index].name) - 1);
          table[index].name[6] = '\0';
          table[index].rssi = buffer_head->rssi;
          table[index].distance = 2. * exp(( - buffer_head->rssi - 84.2) / 4.9);
          device_scan_counter[index]++;

          if(in == 0) {
            device_num++;
          }

          // check if device is contaminated
          contaminated = DmFindAdType(DM_ADV_TYPE_MANUFACTURER,  buffer_head->len, buffer_head->pData);
          if(contaminated[2] == 1 && table[index].distance < 2 && index != contaminated_id) {
            contaminated_id = index;
            contamination_time = device_scan_counter[index] * 15;
            process_post(&contamination_process, contaminated_event, NULL);
          }

          // estimate distance using uwb
          for(int i = 0; i < 10; i++) {
            
            status = range_with((linkaddr_t*)buffer_head->addr, DW1000_RNG_DS);
            if(!status) {
                LOG_DBG("Failed ranging initialization\n");
            } else {

              etimer_set(&timeout_timer, RANGING_TIMEOUT);

              PROCESS_YIELD_UNTIL(ev== ranging_event || etimer_expired(&timeout_timer));

              if(etimer_expired(&timeout_timer)) {
                LOG_DBG("Ranging timeout\n");
              } else if(((ranging_data_t*)data)->status) {
                  
                d = data;
                table[index].distance = d->distance;
                LOG_INFO("Raging successful\n");
                break;
            
              } else {
                LOG_DBG("Ranging failed\n");
              }

            }

          }
        
        } 

      }

      remove_element();

    }

    // print table
    if(device_num > 0) {
      process_post(&contamination_process, table_update_event, NULL);
      LOG_INFO("Found following %d devices:\n", device_num);
    }
    for(int i = 0; i < device_num; i++) {

      LOG_INFO("Device %s has RSSI %d dB, estimated distance is %d.%04d m\n",
               table[i].name, (int)table[i].rssi, (int)table[i].distance, fractional_part(table[i].distance));

    }

  }
  
  PROCESS_END();
  
}



PROCESS_THREAD(best_friends_process, ev, data) {

  PROCESS_BEGIN();

  right_button = button_hal_get_by_id(0);

  static uint8_t max_count;
  static uint8_t index;
  static uint8_t best_friends[3] = {0};
  static uint8_t friend_counter;
  static uint8_t in;

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == button_hal_press_event && data == right_button);

    friend_counter = 0;

    if(device_num > 0) {
      
      LOG_INFO("List of best friends is: \n");
      for(int i = 0; i < 3 && i < device_num; i++) {

        index = 0;
        max_count = 0;
        for(int j = 0; j < device_num; j++) {
          in = 0;
          for(int k = 0; k < friend_counter; k++) {
            if(j == best_friends[k]) {
              in = 1;
              break;
            }
          }

          if(in == 0 && table[j].distance < 5. &&
             ((max_count < device_scan_counter[j]) ||
             (max_count == device_scan_counter[j] && table[j].distance > table[index].distance))) {
            max_count = device_scan_counter[j];
            index = j;
          }
        
        }

        best_friends[i] = index;
        LOG_INFO("%d: device %s (time = %d s, distance = %d.%04d m)\n", i + 1, table[index].name,
                 device_scan_counter[index] * 15, (int)table[index].distance, fractional_part(table[index].distance));

        friend_counter++;

      }

    }

  }

  PROCESS_END();

}


PROCESS_THREAD(contamination_process, ev, data) {

  PROCESS_BEGIN();

  buzzer_init();

  static uint8_t *contaminated;
  static uint8_t still_contaminated;

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == contaminated_event);
    leds_on(LEDS_ORANGE);

    etimer_set(&contamination_timer, MINUTE);
    while(1) {

      PROCESS_WAIT_EVENT_UNTIL(ev == table_update_event || etimer_expired(&contamination_timer));
      still_contaminated = 0;

      if (etimer_expired(&contamination_timer)) {

        still_contaminated = 1;
        break;

      } else if(table[contaminated_id].distance > 2.5) {
        leds_off(LEDS_ORANGE);
        contaminated_id = -1;
        break;
      }

    }

    if(still_contaminated == 1) {

      buzzer_start();

      etimer_set(&contamination_timer, MINUTE);
      while(1) {

        PROCESS_WAIT_EVENT_UNTIL(ev == table_update_event || etimer_expired(&contamination_timer));

        if (etimer_expired(&contamination_timer)) {

          leds_off(LEDS_ORANGE);
          // marked as infected
          leds_on(LEDS_RED);
          // stop old advertisement
          AppAdvStop();

          // change flag
          contaminated = DmFindAdType(DM_ADV_TYPE_MANUFACTURER,  sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
          contaminated[2] = 1;

          // start new advertisement
          AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
          AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(myAppAdvDataDisc), myAppAdvDataDisc);
          LlSetAdvTxPower(0);
          AppAdvStart(APP_MODE_AUTO_INIT);
          buzzer_stop();

          break;

        } else if(table[contaminated_id].distance > 2.5) {
          leds_off(LEDS_ORANGE);
          contaminated_id = -1;
          buzzer_stop();
          break;
        }
      
      }

    }



  }

  PROCESS_END();

}



/*---------------------------------------------------------------------------*/