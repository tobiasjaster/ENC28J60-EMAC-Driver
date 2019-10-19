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

#include "cmsis.h"
#include "enc28j60_eth_drv.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#define ENC28J60_DEBUG 0

#if ENC28J60_DEBUG
#define ENC28J60_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define ENC28J60_DEBUG_PRINTF(...)
#endif

/** Millisec timeout macros */
#define RESET_TIME_OUT_MS               10U
#define REG_WRITE_TIME_OUT_MS           50U
#define PHY_RESET_TIME_OUT_MS           100U
#define INIT_FINISH_DELAY               2000U

/**
 * \brief TX FIFO Size definitions
 *
 */
#define TX_STATUS_FIFO_SIZE_BYTES       512U /*< fixed allocation in bytes */
#define TX_DATA_FIFO_SIZE_KBYTES_POS    8U
#define TX_DATA_FIFO_SIZE_KBYTES_MASK   0x0FU
#define KBYTES_TO_BYTES_MULTIPLIER      1024U

/**
 * \brief FIFO Info definitions
 *
 */
#define FIFO_USED_SPACE_MASK            0xFFFFU
#define DATA_FIFO_USED_SPACE_POS        0U
#define STATUS_FIFO_USED_SPACE_POS      16U

#define HW_CFG_REG_RX_FIFO_POS     		10U
#define HW_CFG_REG_RX_FIFO_SIZE_ALL	    8U
#define HW_CFG_REG_RX_FIFO_SIZE_MIN     2U  /*< Min Rx fifo size in KB */
#define HW_CFG_REG_RX_FIFO_SIZE_MAX     6U /*< Max Rx fifo size in KB */
#define HW_CFG_REG_RX_FIFO_SIZE         5U  /*< Rx fifo size in KB */

/**
 * \brief Chip ID definitions
 *
 */
#define CHIP_ID         0x20C5U


ENC28J60_ETH_DRV::ENC28J60_ETH_DRV(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName reset) :
			_cs(cs), _reset(reset) {
	_spi = new mbed::SPI(mosi, miso, sclk);
	_spi->format(8, 0);          // 8bit, mode 0
    _spi->frequency(7000000);    // 7MHz
	 _bank = 0;
	_blocking = false;
}

ENC28J60_ETH_DRV::ENC28J60_ETH_DRV(mbed::SPI *spi, PinName cs, PinName reset) :
					_cs(cs), _reset(reset) {
	_spi = spi;
	_spi->format(8, 0);          // 8bit, mode 0
    _spi->frequency(7000000);    // 7MHz
	 _bank = 0;
	_blocking = false;
}

void ENC28J60_ETH_DRV::hard_reset(bool state) {

	if (_reset.is_connected()){
		_reset = !state;
	}
}

uint32_t ENC28J60_ETH_DRV::read_id(void) {
	uint16_t phid1;
	uint16_t phid2;

	phyRead(PHHID1, &phid1);
	phyRead(PHHID2, &phid2);

    return ((((uint32_t)phid1) << 6) | (phid2 >> 10));
}

enc28j60_error_t  ENC28J60_ETH_DRV::soft_reset(void) {
    writeOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);
//    wait_ms(1);
    ThisThread::sleep_for(1);
    return ENC28J60_ERROR_NONE;
}

void ENC28J60_ETH_DRV::set_rxfifo(uint32_t val)
{
    if(val >= HW_CFG_REG_RX_FIFO_SIZE_MIN &&
       val <= HW_CFG_REG_RX_FIFO_SIZE_MAX) {
        writeRegPair(ERXSTL, RXSTART_INIT);
        // set receive pointer address
        writeRegPair(ERXRDPTL, RXSTART_INIT);
        // RX end
        writeRegPair(ERXNDL, 0x1FFF-(val << HW_CFG_REG_RX_FIFO_POS));
    }
}

void ENC28J60_ETH_DRV::init_irqs(void) {
    disable_all_interrupts();
    clear_all_interrupts();

    enable_interrupt(ENC28J60_INTERRUPT_RX_PENDING_ENABLE);
    enable_interrupt(ENC28J60_INTERRUPT_LINK_STATE_ENABLE);
    phyWrite(PHIE, PHIE_PLNKIE | PHIE_PGEIE);
    enable_interrupt(ENC28J60_INTERRUPT_TX_ENABLE);
    enable_interrupt(ENC28J60_INTERRUPT_TX_ERROR_ENABLE);
    enable_interrupt(ENC28J60_INTERRUPT_RX_ERROR_ENABLE);

}

enc28j60_error_t ENC28J60_ETH_DRV::check_phy(void) {
	uint16_t phcon1 = 0;
    uint16_t phcon2 = 0;
    uint32_t id = 0;

    if (phyRead(PHCON1, &phcon1)) {
        return ENC28J60_ERROR_INTERNAL;
    }
    if (phyRead(PHCON2, &phcon2)) {
        return ENC28J60_ERROR_INTERNAL;
    }
    id = read_id();
    if (((phcon1 == 0xFFFF && phcon2 == 0xFFFF) ||
            (phcon1 == 0x0 && phcon2 == 0x0)) && (id == 0)) {
        return ENC28J60_ERROR_INTERNAL;
    }
    return ENC28J60_ERROR_NONE;
}

enc28j60_error_t ENC28J60_ETH_DRV::reset_phy(void) {
	enc28j60_error_t error = ENC28J60_ERROR_NONE;
	uint16_t phcon1 = 0;
	error = phyRead(PHCON1, &phcon1);
	if (error)
	    return ENC28J60_ERROR_TIMEOUT;
	error = phyWrite(PHCON1, (phcon1 | PHCON1_PRST));
	if (error)
	    return ENC28J60_ERROR_TIMEOUT;
    ThisThread::sleep_for(PHY_RESET_TIME_OUT_MS);
    error = phyRead(PHCON1, &phcon1);
    if (error || (phcon1 & PHCON1_PRST) != 0) {
		return ENC28J60_ERROR_TIMEOUT;
    }
    return ENC28J60_ERROR_NONE;
}

void ENC28J60_ETH_DRV::enable_xmit(void) {
	writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

void ENC28J60_ETH_DRV::disable_xmit(void) {
	writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
}

void ENC28J60_ETH_DRV::enable_mac_recv(void) {
    writeOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

void ENC28J60_ETH_DRV::disable_mac_recv(void) {
    writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
}

bool ENC28J60_ETH_DRV::check_id(void) {
    uint32_t id = read_id();

    return ((id == CHIP_ID) ? true : false);
}


void ENC28J60_ETH_DRV::enable_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_SET, EIE, source);
}

void ENC28J60_ETH_DRV::disable_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_CLR, EIE, source);
}

void ENC28J60_ETH_DRV::disable_all_interrupts(void) {
	writeReg(EIE, 0x00);
}

void ENC28J60_ETH_DRV::clear_interrupt(enc28j60_interrupt_source source) {
	writeOp(ENC28J60_BIT_FIELD_CLR, EIR, source);
}

void ENC28J60_ETH_DRV::clear_all_interrupts(void) {
    writeReg(EIR, 0x00);
}

uint8_t ENC28J60_ETH_DRV::get_interrupts(void) {
	return readOp(ENC28J60_READ_CTRL_REG, EIR);
}

bool ENC28J60_ETH_DRV::get_interrupt(enc28j60_interrupt_source source) {
    return (get_interrupts()&source)>0 ? true : false;
}

enc28j60_error_t ENC28J60_ETH_DRV::read_mac_address(char *mac) {
    if(!mac) {
        return ENC28J60_ERROR_PARAM;
    }

	_setBank(MAADR0);

	mac[0] = readReg(MAADR5);
	mac[1] = readReg(MAADR4);
	mac[2] = readReg(MAADR3);
	mac[3] = readReg(MAADR2);
	mac[4] = readReg(MAADR1);
	mac[5] = readReg(MAADR0);

    return ENC28J60_ERROR_NONE;
}

enc28j60_error_t ENC28J60_ETH_DRV::write_mac_address(char *mac) {
    if(!mac) {
        return ENC28J60_ERROR_PARAM;
    }

	_setBank(MAADR0);

	writeReg(MAADR5, mac[0]);
	writeReg(MAADR4, mac[1]);
	writeReg(MAADR3, mac[2]);
	writeReg(MAADR2, mac[3]);
	writeReg(MAADR1, mac[4]);
	writeReg(MAADR0, mac[5]);

    return ENC28J60_ERROR_NONE;
}

uint32_t ENC28J60_ETH_DRV::get_rx_data_fifo_size(void) {
    return (uint32_t)(RXSTOP_INIT - RXSTART_INIT);
}

enc28j60_error_t ENC28J60_ETH_DRV::init(void) {
    enc28j60_error_t error = ENC28J60_ERROR_NONE;
    char MACAddr[6] = {0};

    hard_reset(false);

    if(!check_id()) {
        return ENC28J60_ERROR_WRONG_ID;
    }

    error = soft_reset();
    if(error != ENC28J60_ERROR_NONE) {
        return error;
    }

    set_rxfifo(HW_CFG_REG_RX_FIFO_SIZE);

    init_irqs();

    /* Configure MAC addresses here if needed. */
#if (MBED_MAC_ADDRESS_SUM != MBED_MAC_ADDR_INTERFACE)
    MACAddr[0] = MBED_MAC_ADDR_0;
    MACAddr[1] = MBED_MAC_ADDR_1;
    MACAddr[2] = MBED_MAC_ADDR_2;
    MACAddr[3] = MBED_MAC_ADDR_3;
    MACAddr[4] = MBED_MAC_ADDR_4;
    MACAddr[5] = MBED_MAC_ADDR_5;
#else
    mbed_mac_address(MACAddr);
#endif
    error = write_mac_address(MACAddr);
    if(error != ENC28J60_ERROR_NONE) {
        return error;
    }

    error = check_phy();
    if(error != ENC28J60_ERROR_NONE) {
        return error;
    }

    error = reset_phy();
    if(error != ENC28J60_ERROR_NONE) {
        return error;
    }

    writeReg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN);
    writeRegPair(EPMM0, 0x303f);
    writeRegPair(EPMCSL, 0xf7f9);
    // enable MAC receive
    // and bring MAC out of reset (writes 0x00 to MACON2)
    writeRegPair(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
    // enable automatic padding to 60bytes and CRC operations
    writeOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);
    // set inter-frame gap (non-back-to-back)
    writeRegPair(MAIPGL, 0x0C12);
    // set inter-frame gap (back-to-back)
    writeReg(MABBIPG, 0x12);
    // Set the maximum packet size which the controller will accept
    // Do not send packets longer than MAX_FRAMELEN:
    writeRegPair(MAMXFLL, MAX_FRAMELEN);
    // no loopback of transmitted frames
    phyWrite(PHCON2, PHCON2_HDLDIS);

    enable_mac_recv();

    wait_us(60);    // 60us

    //Configure leds
    phyWrite(PHLCON, 0x476);

    return ENC28J60_ERROR_NONE;
}

/** This returns a unique 6-byte MAC address, based on the device UID
*  This function overrides hal/common/mbed_interface.c function
*  @param mac A 6-byte array to write the MAC address
*/

void ENC28J60_ETH_DRV::mbed_mac_address(char *mac)
{
    if (mbed_otp_mac_address(mac)) {
        return;
    } else {
        mbed_default_mac_address(mac);
    }
    return;
}

MBED_WEAK uint8_t ENC28J60_ETH_DRV::mbed_otp_mac_address(char *mac)
{
    return 0;
}

void ENC28J60_ETH_DRV::mbed_default_mac_address(char *mac)
{
	char ST_mac_addr[3] = {0x00, 0x80, 0xe1}; // default STMicro mac address

    // Read unic id
#if defined (TARGET_STM32F2)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F4)
    uint32_t word0 = *(uint32_t *)0x1FFF7A10;
#elif defined (TARGET_STM32F7)
    uint32_t word0 = *(uint32_t *)0x1FF0F420;
#elif defined (TARGET_STM32H7)
    uint32_t word0 = *(uint32_t *)0x1FF1E800;
#else
#error MAC address can not be derived from target unique Id
#endif

    mac[0] = ST_mac_addr[0];
    mac[1] = ST_mac_addr[1];
    mac[2] = ST_mac_addr[2];
    mac[3] = (word0 & 0x00ff0000) >> 16;
    mac[4] = (word0 & 0x0000ff00) >> 8;
    mac[5] = (word0 & 0x000000ff);

    return;
}

enc28j60_error_t ENC28J60_ETH_DRV::tx_setWritePtr(uint16_t position, uint16_t offset) {

	uint32_t start = position + offset > TXSTOP_INIT ? position +
			offset - TXSTOP_INIT + TXSTART_INIT : position + offset;

	writeRegPair(EWRPTL, start);

	return ENC28J60_ERROR_NONE;
}

uint32_t ENC28J60_ETH_DRV::tx_get_fifo_data_used_space(void) {
	return (TXSTOP_INIT-TXSTART_INIT)-tx_get_fifo_data_free_space();
}

uint32_t ENC28J60_ETH_DRV::tx_get_fifo_data_free_space(void) {

	uint32_t freeSpace = 0;
	if (!_tx_handle.packet_data_handled){
		if (_tx_handle.next_packet_data_pointer > _tx_handle.packet_data_pointer) {
			freeSpace = (uint32_t)(TXSTOP_INIT-TXSTART_INIT) - (_tx_handle.next_packet_data_pointer - _tx_handle.packet_data_pointer);
		}
		else if (_tx_handle.next_packet_data_pointer == _tx_handle.packet_data_pointer) {
			freeSpace = (uint32_t)(TXSTOP_INIT-TXSTART_INIT);
		}
		else {
			freeSpace = _tx_handle.packet_data_pointer - _tx_handle.next_packet_data_pointer - 1;
		}
		return freeSpace;
	}
	return (uint32_t)(TXSTOP_INIT-TXSTART_INIT);
}

enc28j60_error_t ENC28J60_ETH_DRV::tx_send_next_packet(void) {
	if (_tx_handle.packet_data_handled && _tx_handle.next_fifo_filled) {
		_tx_handle.packet_data_pointer = _tx_handle.next_packet_data_pointer;
		_tx_handle.packet_data_len = _tx_handle.next_packet_data_len;
		_tx_handle.packet_data_control = _tx_handle.next_packet_data_control;
		uint16_t stop = _tx_handle.packet_data_pointer + _tx_handle.packet_data_len + TX_CONFIG + TX_STATE > TXSTOP_INIT ? _tx_handle.packet_data_pointer +
				_tx_handle.packet_data_len + TX_CONFIG + TX_STATE - TXSTOP_INIT + TXSTART_INIT : _tx_handle.packet_data_pointer + _tx_handle.packet_data_len + TX_CONFIG + TX_STATE;
		_tx_handle.next_packet_data_pointer = stop%2 == 0 ? stop + 2 : stop + 1;
		_tx_handle.next_fifo_filled = false;

#if ENC28J60_DEBUG
		ENC28J60_DEBUG_PRINTF ("sendPacket [%u-%u]: ", (unsigned int)_tx_handle.packet_data_pointer, (unsigned int)stop);
		uint16_t erdptl = readRegPair(ERDPTL);
		//std::vector<uint8_t> data_vector;
		uint8_t *data = new uint8_t[_tx_handle.packet_data_len + TX_CONFIG + TX_STATE]();
		writeRegPair(ERDPTL, _tx_handle.packet_data_pointer);
		readBuffer((uint16_t)(_tx_handle.packet_data_len + TX_CONFIG + TX_STATE), (uint8_t *)data);
		for (uint16_t i = 0; i < _tx_handle.packet_data_len + TX_CONFIG + TX_STATE; i++) {
			ENC28J60_DEBUG_PRINTF("%d ", data[i]);
		}
		ENC28J60_DEBUG_PRINTF ("\r\n");
		writeRegPair(ERDPTL, erdptl);
#endif
		writeRegPair(ETXSTL, _tx_handle.packet_data_pointer);
		writeRegPair(ETXNDL, stop);
		enable_xmit();
		_tx_handle.packet_data_handled = false;
		return ENC28J60_ERROR_NONE;
	}
	return ENC28J60_ERROR_NEXTPACKET;
}

enc28j60_error_t ENC28J60_ETH_DRV::tx_write_data_fifo(uint8_t *data, uint16_t offset, uint16_t size_bytes) {
	uint16_t full_size = size_bytes+TX_CONFIG+TX_STATE;
	if(tx_get_fifo_data_free_space() < full_size){
		return ENC28J60_ERROR_SIZE;
	}
	if(_tx_handle.next_fifo_filled!=false){
		return ENC28J60_ERROR_FIFOFULL;
	}
	_tx_handle.next_packet_data_control = 0;
	enc28j60_error_t error = tx_setWritePtr(_tx_handle.next_packet_data_pointer, offset);
	if(offset==0){
		_tx_handle.next_packet_data_len = size_bytes;
		writeBuffer(1, &_tx_handle.next_packet_data_control);
	}
	if (error) {
		return error;
	}
	uint16_t writeBufferSizeCounter = size_bytes;
	uint8_t *writeBufferPointer = data;
	uint16_t writeBufferOffset = 1;
	while (writeBufferSizeCounter != 0) {
		if (writeBufferSizeCounter >= 64){
			error = tx_setWritePtr(_tx_handle.next_packet_data_pointer, writeBufferOffset);
			if (error) {
				return error;
			}
			writeBuffer(64, writeBufferPointer);
			writeBufferPointer += 64;
			writeBufferOffset += 64;
			writeBufferSizeCounter -= 64;
		}else {
			error = tx_setWritePtr(_tx_handle.next_packet_data_pointer, writeBufferOffset);
			if (error) {
				return error;
			}
			writeBuffer(writeBufferSizeCounter, writeBufferPointer);
			writeBufferSizeCounter = 0;
		}
	}
	_tx_handle.next_fifo_filled = true;
	return error;
}

void ENC28J60_ETH_DRV::tx_packetHandled(void) {
	_tx_handle.packet_data_handled = true;
}



uint32_t ENC28J60_ETH_DRV::rx_get_fifo_data_used_space(void) {
	return (RXSTOP_INIT - RXSTART_INIT)-rx_get_fifo_data_free_space();
}

uint32_t ENC28J60_ETH_DRV::rx_get_fifo_data_free_space(void) {

	uint16_t readPointer = rx_getReadPointer();
	uint16_t writePointer = rx_getWritePointer();
	uint32_t freeSpace = 0;
	if (writePointer > readPointer) {
		freeSpace = (uint32_t)(RXSTOP_INIT - RXSTART_INIT) - (writePointer - readPointer);
	}
	else if (writePointer == readPointer) {
		freeSpace = (RXSTOP_INIT - RXSTART_INIT);
	}
	else {
		freeSpace = readPointer - writePointer - 1;
	}
	return freeSpace;
}

enc28j60_error_t ENC28J60_ETH_DRV::rx_setReadPtr(uint16_t position, uint16_t offset) { //(uint16_t offset, uint16_t *len)
//	if (offset > _rx_handle.packet_len) {
//		return ENC28J60_ERROR_POSITIONLENGTH;
//	}
//	uint32_t start = _rx_handle.packet_data_pointer + offset > RXSTOP_INIT ? _rx_handle.packet_data_pointer +
//			offset - RXSTOP_INIT + RXSTART_INIT : _rx_handle.packet_pointer + offset;

	uint32_t start = position + offset > RXSTOP_INIT ? position +
			offset - RXSTOP_INIT + RXSTART_INIT : position + offset;

	writeRegPair(ERDPTL, start);

//	*len = _rx_handle.packet_len - offset;

	return ENC28J60_ERROR_NONE;
}

enc28j60_error_t ENC28J60_ETH_DRV::rx_peek_next_packet(enc28j60_rx_handle_t *handle) {

	if (_rx_handle.packet_handled) {
	    if (readReg(EPKTCNT) != 0) {
	    	_rx_handle.packet_pointer = _rx_handle.next_packet_pointer;
	    	_rx_handle.packet_data_pointer = _rx_handle.packet_pointer + 6 > RXSTOP_INIT ? _rx_handle.packet_pointer +
	            6 - RXSTOP_INIT + RXSTART_INIT : _rx_handle.packet_pointer + 6;
	        // Set the read pointer to the start of the received packet

	        writeRegPair(ERDPTL, _rx_handle.packet_pointer);

	        // read the next packet pointer
	        _rx_handle.next_packet_pointer = readOp(ENC28J60_READ_BUF_MEM, 0);
	        _rx_handle.next_packet_pointer |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;

	        // read the packet length (see datasheet page 43)
	        _rx_handle.packet_len = readOp(ENC28J60_READ_BUF_MEM, 0);
	        _rx_handle.packet_len |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;
	        _rx_handle.packet_len -= 4;   //remove the CRC count

	        // read the receive status (see datasheet page 43)
	        _rx_handle.packet_state = readOp(ENC28J60_READ_BUF_MEM, 0);
	        _rx_handle.packet_state |= readOp(ENC28J60_READ_BUF_MEM, 0) << 8;

	        _rx_handle.packet_handled = false;

#if ENC28J60_DEBUG
	        ENC28J60_DEBUG_PRINTF (
	            "[ENC28J60] rx_peek_next_packet: receivePacket [%u-%u], next: %u, stat: %u, count: %d -> ",
				(unsigned int)_rx_handle.packet_pointer,
				(unsigned int)((_rx_handle.packet_pointer + _rx_handle.packet_len) % (RXSTOP_INIT + 1)),
				(unsigned int)_rx_handle.next_packet_pointer,
				(unsigned int)_rx_handle.packet_state,
	            readReg(EPKTCNT)
	        );
	        (_rx_handle.packet_state & 0x80) != 0 ? ENC28J60_DEBUG_PRINTF("OK") : ENC28J60_DEBUG_PRINTF("failed");
	        ENC28J60_DEBUG_PRINTF("\r\n");
#endif
	        // decrement the packet counter indicate we are done with this packet

	        writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

	        // check CRC and symbol errors (see datasheet page 44, table 7-3):
	        // The ERXFCON.CRCEN is set by default. Normally we should not
	        // need to check this.
	        if ((_rx_handle.packet_state & 0x80) != 0) {
	        	*handle = _rx_handle;
	            return ENC28J60_ERROR_NONE;
	        }

	        // Move the RX read pointer to the start of the next received packet
	        // This frees the memory we just read out
	        _rx_handle.packet_handled = true;
	        return ENC28J60_ERROR_RECEIVE;
	    }
	    return ENC28J60_ERROR_NOPACKET;
	}
	return ENC28J60_ERROR_LASTPACKET;
}

enc28j60_error_t ENC28J60_ETH_DRV::rx_read_data_packet(uint8_t *data, uint16_t offset, uint16_t size_bytes) {
	uint16_t len = _rx_handle.packet_len - offset;
	if (size_bytes > len) {
		ENC28J60_DEBUG_PRINTF ("[ENC28J60] rx_read_data_packet: length = %u", len);
		return ENC28J60_ERROR_POSITIONLENGTH;
	}
	enc28j60_error_t error = ENC28J60_ERROR_NONE;

	uint16_t readBufferSizeCounter = size_bytes;
	uint8_t *readBufferPointer = data;
	uint16_t readBufferOffset = offset;
	while (readBufferSizeCounter != 0) {
		if (readBufferSizeCounter >= 64){
			error = rx_setReadPtr(_rx_handle.packet_data_pointer, readBufferOffset);
			if (error) {
				ENC28J60_DEBUG_PRINTF ("[ENC28J60] rx_read_data_packet: %d", error);
				return ENC28J60_ERROR_POSITIONLENGTH;
			}
			readBuffer(64, readBufferPointer);
			readBufferPointer += 64;
			readBufferOffset += 64;
			readBufferSizeCounter -= 64;
		}else {
			error = rx_setReadPtr(_rx_handle.packet_data_pointer, readBufferOffset);
			if (error) {
				ENC28J60_DEBUG_PRINTF ("[ENC28J60] rx_read_data_packet: %d", error);
				return ENC28J60_ERROR_POSITIONLENGTH;
			}
			readBuffer(readBufferSizeCounter, readBufferPointer);
			readBufferSizeCounter = 0;
		}
	}
	return error;
}

void ENC28J60_ETH_DRV::rx_packetHandled(void) {
	_rx_handle.packet_handled = true;
}

void ENC28J60_ETH_DRV::rx_freeBuffer(void) {
	if (_rx_handle.packet_handled == true) {
	    writeRegPair(ERXRDPTL, _rx_handle.next_packet_pointer == RXSTART_INIT ? RXSTOP_INIT : _rx_handle.next_packet_pointer - 1);
	}
	else {
		writeRegPair(ERXRDPTL, _rx_handle.packet_pointer == RXSTART_INIT ? RXSTOP_INIT : _rx_handle.packet_pointer - 1);
	}
}

uint16_t ENC28J60_ETH_DRV::rx_getReadPointer(void) {
	return readRegPair(ERXRDPTL);
}
uint16_t ENC28J60_ETH_DRV::rx_getWritePointer(void) {
	uint16_t count_pre = readReg(EPKTCNT);
	uint16_t writePointer = readRegPair(ERXWRPTL);
	uint16_t count_post = readReg(EPKTCNT);
	while (count_pre != count_post){
		count_pre = count_post;
		writePointer = readRegPair(ERXWRPTL);
		count_post = readReg(EPKTCNT);
	}
	ENC28J60_DEBUG_PRINTF ("[ENC28J60] rx_getWritePointer: %d", writePointer);
	return writePointer;
}

void ENC28J60_ETH_DRV::readBuffer(uint16_t len, uint8_t* data) {

	_read(ENC28J60_READ_BUF_MEM, data, len, false);
	data[len] = '\0';
}

void ENC28J60_ETH_DRV::writeBuffer(uint16_t len, uint8_t* data) {

	_write(ENC28J60_WRITE_BUF_MEM, data, len, false);
}

uint8_t ENC28J60_ETH_DRV::readReg(uint8_t address) {

    // set the bank
    _setBank(address);

    // do the read
    return readOp(ENC28J60_READ_CTRL_REG, address);
}

uint16_t ENC28J60_ETH_DRV::readRegPair(uint8_t address) {

	uint16_t temp;
    // set the bank
    _setBank(address);

    // do the read
    temp = (uint16_t)(readOp(ENC28J60_READ_CTRL_REG, address + 1)) << 8;
    temp |= readOp(ENC28J60_READ_CTRL_REG, address);

    return temp;
}

void ENC28J60_ETH_DRV::writeReg(uint8_t address, uint8_t data) {
    // set the bank
    _setBank(address);
    // do the write
    writeOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void ENC28J60_ETH_DRV::writeRegPair(uint8_t address, uint16_t data) {
    // set the bank
    _setBank(address);

    // do the write
    writeOp(ENC28J60_WRITE_CTRL_REG, address, (data & 0xFF));
    writeOp(ENC28J60_WRITE_CTRL_REG, address + 1, (data) >> 8);
}

enc28j60_error_t ENC28J60_ETH_DRV::phyRead(uint8_t address, uint16_t *data) {
	uint8_t timeout = 0;
	writeReg(MIREGADR, address);
    writeReg(MICMD, MICMD_MIIRD);

    // wait until the PHY read completes
    while (readReg(MISTAT) & MISTAT_BUSY) {
        wait_us(15);
        timeout++;
        if (timeout > 10)
        	return ENC28J60_ERROR_TIMEOUT;
    }   //and MIRDH

    writeReg(MICMD, 0);
    *data = (readReg(MIRDL) | readReg(MIRDH) << 8);
    return ENC28J60_ERROR_NONE;
}

enc28j60_error_t ENC28J60_ETH_DRV::phyWrite(uint8_t address, uint16_t data) {
	uint8_t timeout = 0;
    // set the PHY register address
    writeReg(MIREGADR, address);

    // write the PHY data
    writeRegPair(MIWRL, data);

    // wait until the PHY write completes
    while (readReg(MISTAT) & MISTAT_BUSY) {
        wait_us(15);
        timeout++;
        if (timeout > 10)
        	return ENC28J60_ERROR_TIMEOUT;
    }
    return ENC28J60_ERROR_NONE;
}

bool ENC28J60_ETH_DRV::linkStatus(void) {
	uint16_t data;
	phyRead(PHSTAT2, &data);
    return(data & 0x0400) > 0;
}

uint8_t ENC28J60_ETH_DRV::readOp(uint8_t op, uint8_t address) {
    uint8_t result;
	uint8_t data[2];

    // issue read command
    if (address & 0x80) {
    	_read((op | (address & ADDR_MASK)), &data[0], 2, false);
    	result = data[1];
    	return result;
    }
    else{
    	_read((op | (address & ADDR_MASK)), &data[0], 1, false);
    }
    result = data[0];
    return result;
}

void ENC28J60_ETH_DRV::writeOp(uint8_t op, uint8_t address, uint8_t data) {
    // issue write command
    _write(op | (address & ADDR_MASK), &data, 1, false);
}


void ENC28J60_ETH_DRV::_setBank(uint8_t address) {
    // set the bank (if needed)
    if ((address & BANK_MASK) != _bank) {
        // set the bank
        writeOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1 | ECON1_BSEL0));
        writeOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK) >> 5);
        _bank = (address & BANK_MASK);
    }
}

void ENC28J60_ETH_DRV::_read(uint8_t cmd, uint8_t *buf, uint16_t len, bool blocking) {
#ifndef ENC28J60_READWRITE
	_SPIMutex.lock();
	_cs = 0;
    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        // read data
        *buf = _spi->write(0x00);
        buf++;
    }
    _cs = 1;
	_SPIMutex.unlock();
#else
	uint8_t *dummy = NULL;
	_readwrite(cmd, buf, dummy, len, blocking);
#endif
}

void ENC28J60_ETH_DRV::_write(uint8_t cmd, uint8_t *buf, uint8_t len, bool blocking) {
#ifndef ENC28J60_READWRITE
	_SPIMutex.lock();
    _cs = 0;
    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        // read data
        _spi->write((int)*buf);
        buf++;
    }
    _cs = 1;
	_SPIMutex.unlock();
#else
	uint8_t *dummy = NULL;
	_readwrite(cmd, dummy, buf, len, blocking);
#endif
}

#ifdef ENC28J60_READWRITE

void ENC28J60_ETH_DRV::_readwrite(uint8_t cmd, uint8_t *readbuf, uint8_t *writebuf, uint8_t len, bool blocking) {
	_SPIMutex.lock();
	_cs = 0;
    // issue read command
    _spi->write((int)cmd);
    while (len) {
        len--;

        if(readbuf == NULL) {
        	_spi->write((int)*writebuf);
            writebuf++;
        }else if (writebuf == NULL){
        	*readbuf =_spi->write(0x00);
            readbuf++;
        }else {
        	break;
        }
    }
    _cs = 1;
	_SPIMutex.unlock();
}
#endif


