/**
 * \file
 * Header file for the ringbuffer.c
 */
/*---------------------------------------------------------------------------*/
// include <stdio.h>
/*---------------------------------------------------------------------------*/

#define MAX_NUMBER_ENTRIES 30
#define MAX_PAYLOAD_ADV_PACKET 31

struct scanbuffer {
  uint8_t             pData[MAX_PAYLOAD_ADV_PACKET];    /*!< \brief advertising or scan response data. */ //(max ADV) -> if some devices have larger adv data (extension) -> discard rest (copy only first 31byte - enough for our application)
  uint8_t             len;                              /*!< \brief length of advertising or scan response data. */
  int8_t              rssi;                             /*!< \brief RSSI. */
  uint8_t             addr[6];                          /*!< \brief Device address. */
} ;

/**
 * \brief Adds a new element to the buffer
 * \param pData advertising or scan response data
 * \param len length of advertising or scan response data.
 * \param rssi rssi value of the scan data
 * \param addr device address
 */
void add_element(uint8_t* pData, int8_t len, int8_t rssi, uint8_t* addr);

/**
 * \brief Remove the last element from buffer
 */
void remove_element(void);

/**
 * \brief Get the current element
 * \return the address of the current element or NULL in case buffer is empty
 */
struct scanbuffer* get_element(void);
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
