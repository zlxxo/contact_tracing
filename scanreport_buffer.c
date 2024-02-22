/*---------------------------------------------------------------------------*/
/**
 * \file
 *  Ringbuffer for the scanreports
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "scanreport_buffer.h"

# define LOG_COLOR_DBG TC_GREEN
# include "sys/log.h"
# define LOG_MODULE "SCAN REP"
# define LOG_LEVEL LOG_LEVEL_DBG

static uint16_t tail = 0;                  //Points to the first free element -> modulo check to be performed 
static uint16_t head = 0;               //Points to the next element to be used 
static uint16_t number_of_elements = 0;    //Number of elements in the buffer

static struct scanbuffer scanreports [MAX_NUMBER_ENTRIES];

/*---------------------------------------------------------------------------*/
void add_element(uint8_t* pData, int8_t len, int8_t rssi, uint8_t* addr)
{
  if(number_of_elements > MAX_NUMBER_ENTRIES) {
    LOG_WARN("No space for new entries!\n");
    return;   //No space for new entries
  }

  if(pData == NULL) {
    LOG_WARN("Invalid pointer!\n");
    return;   //Invalid pointer
  }
    
  if(tail >= MAX_NUMBER_ENTRIES)
    tail = 0; 

  memset(scanreports[tail].pData, 0, MAX_PAYLOAD_ADV_PACKET);        //Init with 0
  len = len > MAX_PAYLOAD_ADV_PACKET ? MAX_PAYLOAD_ADV_PACKET : len;
  memcpy(scanreports[tail].pData, pData, len);
  
  scanreports[tail].len = len;
  scanreports[tail].rssi = rssi;
  memcpy(scanreports[tail].addr, addr, 6);
  tail++;     //For next cycle
  number_of_elements++;
}

/*---------------------------------------------------------------------------*/
void remove_element()
{
  if(number_of_elements == 0)
  {
    LOG_WARN("No elements to be removed!\n");
    return;
  }
  memset(scanreports[head].pData, 0, MAX_PAYLOAD_ADV_PACKET);
  scanreports[head].len = 0;
  scanreports[head].rssi = 0;
  memset(scanreports[head].addr,0,6);
  
  number_of_elements--;
  head++;
  if(head >= MAX_NUMBER_ENTRIES) 
    head = 0;
}
/*---------------------------------------------------------------------------*/
struct scanbuffer* get_element()
{
  if(number_of_elements == 0) {
    LOG_WARN("No elements in list!\n");
    return NULL;
  }
  return &scanreports[head];
}
/*---------------------------------------------------------------------------*/
/** @} */
