/*
 * Copyright (c) 2019 Tobias Jaster
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ENC28J60_ETH_DRV_H_
#define ENC28J60_ETH_DRV_H_

#include <stdint.h>
//#include "stdbool.h"
//#include "EMAC.h"
#include "mbed.h"
#include "rtos.h"
#include "rtos/Mutex.h"
#include "enc28j60.h"

#define ENC28J60_READWRITE

/**
 * \brief Error code definitions
 *
 */
typedef struct {
	bool packet_handled = true;
	uint16_t packet_state = 0;
	uint32_t packet_pointer = 0;
	uint32_t packet_data_pointer = 0;
	uint16_t packet_len = 0;
	uint32_t next_packet_pointer = RXSTART_INIT;
} enc28j60_rx_handle_t;

typedef struct {
	bool packet_data_handled = true;
	uint32_t packet_data_pointer = 0;
	uint8_t packet_data_control = 0;
	uint16_t packet_data_len = 0;
	uint8_t packet_data_state[7] = {0};
	bool next_fifo_filled = false;
	uint32_t next_packet_data_pointer = TXSTART_INIT;
	uint8_t next_packet_data_control = 0;
	uint16_t next_packet_data_len = 0;
} enc28j60_tx_handle_t;

/**
 * \brief Error code definitions
 *
 */
typedef enum {
    ENC28J60_ERROR_NONE     		= 0U, /*!< no error */
    ENC28J60_ERROR_TIMEOUT  		= 1U, /*!< timeout */
    ENC28J60_ERROR_BUSY     		= 2U, /*!< no error */
    ENC28J60_ERROR_PARAM    		= 3U, /*!< invalid parameter */
    ENC28J60_ERROR_INTERNAL 		= 4U,  /*!< internal error */
    ENC28J60_ERROR_WRONG_ID 		= 5U,  /*!< internal error */
	ENC28J60_ERROR_NOPACKET 		= 10U,
	ENC28J60_ERROR_RECEIVE			= 11U,
	ENC28J60_ERROR_LASTPACKET		= 12U,
    ENC28J60_ERROR_POSITIONLENGTH 	= 13U,  /*!< internal error */
    ENC28J60_ERROR_SIZE			 	= 20U,  /*!< internal error */
    ENC28J60_ERROR_FIFOFULL		 	= 21U,  /*!< internal error */
    ENC28J60_ERROR_NEXTPACKET	 	= 22U,  /*!< internal error */

} enc28j60_error_t;

/**
 * \brief Interrupt source definitions
 *
 */
typedef enum {
	ENC28J60_INTERRUPT_ENABLE				  = EIE_INTIE,
	ENC28J60_INTERRUPT_RX_PENDING_ENABLE	  = EIE_PKTIE,
	ENC28J60_INTERRUPT_DMA_ENABLE	  		  = EIE_DMAIE,
	ENC28J60_INTERRUPT_LINK_STATE_ENABLE	  = EIE_LINKIE,
	ENC28J60_INTERRUPT_TX_ENABLE			  = EIE_TXIE,
	ENC28J60_INTERRUPT_TX_ERROR_ENABLE		  = EIE_TXERIE,
	ENC28J60_INTERRUPT_RX_ERROR_ENABLE		  = EIE_RXERIE
}enc28j60_interrupt_source;


class ENC28J60_ETH_DRV {

public:

	ENC28J60_ETH_DRV(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName reset);

	ENC28J60_ETH_DRV(mbed::SPI *spi, PinName cs, PinName reset);

	/**
	 * \brief Initializes ENC28J60 Ethernet controller to a known default state:
	 *          - device ID is checked
	 *          - global interrupt is enabled, but all irq sources are disabled
	 *          - Establish link enabled
	 *          - Rx enabled
	 *          - Tx enabled
	 *        Init should be called prior to any other process and
	 *        it's the caller's responsibility to follow proper call order.
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	enc28j60_error_t init(void);



	/** This returns a unique 6-byte MAC address, based on the device UID
	*  This function overrides hal/common/mbed_interface.c function
	*  @param mac A 6-byte array to write the MAC address
	*/

	void mbed_mac_address(char *mac);
	MBED_WEAK uint8_t mbed_otp_mac_address(char *mac);
	void mbed_default_mac_address(char *mac);
	/**
	 * \brief Read ENC28J60 ID.
	 *
	 * \return ID number
	 */
	uint32_t read_id(void);

	/**
	 * \brief Initiates a hard reset.
	 */
	void hard_reset(bool state);

	/**
	 * \brief Initiates a soft reset, returns failure or success.
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	enc28j60_error_t soft_reset(void);

	/**
	 * \brief Set maximum transition unit by Rx fifo size.
	 *        Note: The MTU will be smaller by 512 bytes,
	 *        because the status uses this fixed space.
	 *
	 * \param[in] val Size of the fifo in kbytes
	 */
	void set_rxfifo(uint32_t val);

	/**
	 * \brief Initialise irqs by clearing and disabling all interrupt sources
	 *        and enable interrupts. Since all interrupt sources are disabled,
	 *        interrupt won't be triggered, until interrupt sources won't be
	 *        enabled by \ref enc28j60_enable_interrupt
	 */
	void init_irqs(void);

	/**
	 * \brief Check PHY ID registers.
	 *
	 * \param[in] dev Ethernet device structure \ref enc28j60_eth_dev_t
	 */
	enc28j60_error_t check_phy(void);

	/**
	 * \brief Reset PHY
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	enc28j60_error_t reset_phy(void);

	/**
	 * \brief Enable transmission
	 */
	void enable_xmit(void);

	/**
	 * \brief Disable transmission
	 */
	void disable_xmit(void);

	/**
	 * \brief Enable receive
	 */
	void enable_mac_recv(void);

	/**
	 * \brief Disable receive
	 */
	void disable_mac_recv(void);

	/**
	 * \brief Enable the given interrupt source.
	 *
	 * \param[in] source Enum of the interrupt source.
	 */
	void enable_interrupt(enc28j60_interrupt_source source);

	/**
	 * \brief Disable the given interrupt source.
	 *
	 * \param[in] source Enum of the interrupt source.
	 */
	void disable_interrupt(enc28j60_interrupt_source source);

	/**
	 * \brief Disable all of the interrupt sources.
	 */
	void disable_all_interrupts(void);

	/**
	 * \brief Clear the given interrupt source.
	 *
	 * \param[in] source Enum of the interrupt source.
	 */
	void clear_interrupt(enc28j60_interrupt_source source);

	/**
	 * \brief Clear all of the interrupt sources.
	 */
	void clear_all_interrupts(void);

	/**
	 * \brief Get the status of all interrupt sources.
	 *
	 * \return non-zero if the given interrupt source is triggered, zero otherwise
	 */
	uint8_t get_interrupts(void);
	/**
	 * \brief Get the status of the given interrupt source.
	 *
	 * \param[in] source Enum of the interrupt source.
	 *
	 * \return non-zero if the given interrupt source is triggered, zero otherwise
	 */
	bool get_interrupt(enc28j60_interrupt_source source);

	/**
	 * \brief Read MAC address from EEPROM.
	 *
	 * \param[in,out] mac array will include the read MAC address in
	 *                6 bytes hexadecimal format.
	 *                It should be allocated by the caller to 6 bytes.
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	enc28j60_error_t read_mac_address(char *mac);

	/**
	 * \brief Write MAC address to EEPROM.
	 *
	 * \param[in,out] mac array will include the write MAC address in
	 *                6 bytes hexadecimal format.
	 *                It should be allocated by the caller to 6 bytes.
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	enc28j60_error_t write_mac_address(char *mac);

	/**
	 * \brief Check device ID.
	 *
	 * \return error code /ref enc28j60_error_t
	 */
	bool check_id(void);

	/**
	 * \brief Get the data size of the Rx buffer, aka Maximum Transition Unit
	 *
	 * \return Fifo data size in bytes
	 */
	uint32_t get_rx_data_fifo_size(void);

	enc28j60_error_t tx_setWritePtr(uint16_t position, uint16_t offset);
	uint32_t tx_get_fifo_data_used_space(void);
	uint32_t tx_get_fifo_data_free_space(void);
	enc28j60_error_t tx_send_next_packet(void);
	enc28j60_error_t tx_write_data_fifo(uint8_t *data, uint16_t position, uint16_t size_bytes);
	void tx_packetHandled(void);
	/**
	 * \brief Get the used space of Rx fifo in bytes.
	 *
	 * \param[in] dev Ethernet device structure \ref enc28j60_eth_dev_t
	 *
	 * \return Data received and waiting for read in bytes
	 */
	uint32_t rx_get_fifo_data_used_space(void);

	uint32_t rx_get_fifo_data_free_space(void);
	/**
	 * \brief Get the size of next unread packet in Rx buffer, using the peak
	 *        register, which is not destructive so can be read asynchronously.
	 *        Warning: In case of heavy receiving load, it's possible this register
	 *        is not perfectly in sync.
	 *
	 * \param[in] dev Ethernet device structure \ref enc28j60_eth_dev_t
	 *
	 * \return Size in bytes of the next packet can be read from Rx fifo, according
	 *         to the peek register.
	 */
	enc28j60_error_t rx_setReadPtr(uint16_t position, uint16_t offset);//(uint16_t offset, uint16_t *len);

	enc28j60_error_t rx_peek_next_packet(enc28j60_rx_handle_t *handle);

	enc28j60_error_t rx_read_data_packet(uint8_t *data, uint16_t offset, uint16_t size_bytes);

	void rx_packetHandled(void);

	void rx_freeBuffer(void);

	uint16_t rx_getReadPointer(void);

	uint16_t rx_getWritePointer(void);

	void readBuffer(uint16_t len, uint8_t* data);
	void writeBuffer(uint16_t len, uint8_t* data);
	uint8_t readReg(uint8_t address);
	uint16_t readRegPair(uint8_t address);
	void writeReg(uint8_t address, uint8_t data);
	void writeRegPair(uint8_t address, uint16_t data);
	enc28j60_error_t phyRead(uint8_t address, uint16_t *data);
	enc28j60_error_t phyWrite(uint8_t address, uint16_t data);
	bool linkStatus(void);
	uint8_t readOp(uint8_t op, uint8_t address);
	void writeOp(uint8_t op, uint8_t address, uint8_t data);

private:
	void _setBank(uint8_t address);
	void _read(uint8_t cmd, uint8_t *buf, uint16_t len, bool blocking);
	void _write(uint8_t cmd, uint8_t *buf, uint8_t len, bool blocking);
#ifdef ENC28J60_READWRITE
	void _readwrite(uint8_t cmd, uint8_t *readbuf, uint8_t *writebuf, uint8_t len, bool blocking);
#endif
	mbed::SPI* _spi;
	rtos::Mutex _SPIMutex;
	DigitalOut _cs;
	DigitalOut _reset;
	bool _blocking;
    uint8_t _bank;
    enc28j60_rx_handle_t _rx_handle;
    enc28j60_tx_handle_t _tx_handle;
};


#endif /* ENC28J60_ETH_DRV_H_ */
