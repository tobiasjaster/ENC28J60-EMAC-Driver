/*
 * enc28j60_emac_config.h
 *
 *  Created on: 27.08.2019
 *      Author: tobias
 */

#ifndef ENC28J60_EMAC_CONFIG_H_
#define ENC28J60_EMAC_CONFIG_H_

#include "cmsis_os.h"

#define ENC28J60_HWADDR_SIZE                 6U
#define ENC28J60_BUFF_ALIGNMENT              4U

/*
 * Maximum Transfer Unit
 * The IEEE 802.3 specification limits the data portion of the 802.3 frame
 * to a minimum of 46 and a maximum of 1522 bytes, this is on L2 level.
 */
#define ENC28J60_ETH_MTU_SIZE                1500U
#define ENC28J60_ETH_IF_NAME                 "enc28j60"

/** \brief Defines for receiver thread */
#define FLAG_RX                              1U
#define RX_THREAD_PRIORITY          		 (osPriorityNormal)
#define RX_THREAD_STACKSIZE         		 2048U
#define LINK_STATUS_TASK_PERIOD_MS           200U
#define FLAG_IRQ                             2U
#define IRQ_THREAD_PRIORITY          		 (osPriorityISR)
#define IRQ_THREAD_STACKSIZE         		 2048U
#define PHY_STATE_LINK_DOWN                  false
#define PHY_STATE_LINK_UP                    true
#define CRC_LENGTH_BYTES                     4U

#endif /* ENC28J60_EMAC_CONFIG_H_ */
