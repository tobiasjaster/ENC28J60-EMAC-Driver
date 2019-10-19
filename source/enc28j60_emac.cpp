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

#include "mbed_interface.h"
#include "mbed_wait_api.h"
#include "mbed_assert.h"
#include "netsocket/nsapi_types.h"
#include "mbed_shared_queues.h"

#include "enc28j60.h"
#include "enc28j60_emac.h"
#include "enc28j60_eth_drv.h"

#ifndef DEVICE_ENC28J60
#error "ENC28J60_ETH should be defined, check device_cfg.h!"
#endif

#ifndef NULL
#define NULL ((void *) 0)
#endif

static ENC28J60_EMAC *board_emac_pointer = NULL;

ENC28J60_EMAC::ENC28J60_EMAC() : _int(ETH_INT) {

	_irq_thread = new Thread(IRQ_THREAD_PRIORITY,(uint32_t)IRQ_THREAD_STACKSIZE);
	_rx_thread = new Thread(RX_THREAD_PRIORITY,(uint32_t)RX_THREAD_STACKSIZE);
	_enc28j60 = new ENC28J60_ETH_DRV(ETH_MOSI, ETH_MISO, ETH_SCK, ETH_CS, ETH_RESET);
	_enc28j60->hard_reset(false);
	power_down();
	#ifdef ETH_INT
	_int.fall(callback(this, &ENC28J60_EMAC::Interrupt_Handler));
	#endif

	_prev_link_status_up = PHY_STATE_LINK_DOWN;
	_link_status_task_handle = 0;
	_memory_manager = NULL;
}


/** \brief Ethernet receive interrupt handler
 *
 *  This function handles the receive interrupt.
 */
void ENC28J60_EMAC::Interrupt_Handler(void) {
	_irq_thread->flags_set(FLAG_IRQ);
}

/** \brief  Allocates a emac_mem_buf_t and returns the data from the incoming
 * packet.
 *
 *  \return a emac_mem_buf_t filled with the received packet
 * (including MAC header)
 */
emac_mem_buf_t *ENC28J60_EMAC::low_level_input() {
    emac_mem_buf_t *p = NULL;
    uint32_t message_length = 0;
    enc28j60_rx_handle_t handle;
    uint32_t length = 0;

    _ETHLockMutex.lock();
    if (_enc28j60->rx_peek_next_packet(&handle)) {
        _ETHLockMutex.unlock();
        return p;
    }
    _ETHLockMutex.unlock();
    message_length = handle.packet_len;
    if (message_length == 0) {
        return p;
    }

    p = _memory_manager->alloc_heap(message_length, ENC28J60_BUFF_ALIGNMENT);

    if (p != NULL) {
        _ETHLockMutex.lock();
        length = _memory_manager->get_len(p);
        MBED_ASSERT(length < 0xFFFF);
        _enc28j60->rx_read_data_packet(reinterpret_cast<uint8_t*>(_memory_manager->get_ptr(p)),
        		0,
				(uint16_t)(_memory_manager->get_len(p)));
        _ETHLockMutex.unlock();
    }
    return p;
}

/** \brief  Receiver thread.
 *
 * Woken by thread flags to receive packets or clean up transmit
 *
 *  \param[in] params pointer to the interface data
 */
void ENC28J60_EMAC::receiver_thread_function(void* params) {
    ENC28J60_EMAC *enc28j60_enet = static_cast<ENC28J60_EMAC *>(params);

    while(1) {
        uint32_t flags = ThisThread::flags_wait_any(FLAG_RX);

        if (flags & FLAG_RX) {
        	enc28j60_enet->packet_rx();
        }
        enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
    }
}

void ENC28J60_EMAC::interrupt_thread_function(void* params) {

	ENC28J60_EMAC *enc28j60_enet = static_cast<ENC28J60_EMAC *>(params);
	volatile uint32_t flags;
	uint8_t estat = 0;
	uint8_t eir = 0;

    while(1) {
        flags = ThisThread::flags_wait_any(FLAG_IRQ);

        if (flags & FLAG_IRQ) {
        	flags = 0;
        	estat = enc28j60_enet->_enc28j60->readOp(ENC28J60_READ_CTRL_REG, ESTAT);
        	if ((estat & 0x80) > 0) {
        		enc28j60_enet->_enc28j60->disable_interrupt(ENC28J60_INTERRUPT_ENABLE);
    			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_ENABLE);
        		eir = enc28j60_enet->_enc28j60->get_interrupts();
        		if ((eir & 0x40) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_RX_PENDING_ENABLE);
        			enc28j60_enet->_rx_thread->flags_set(FLAG_RX);
            		printf("ENC28J60_INTERRUPT_RX_PENDING_ENABLE");
            		//break;
        		}
        		else if ((eir & 0x20) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_DMA_ENABLE);
            		printf("ENC28J60_INTERRUPT_DMA_ENABLE");
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
            		//break;
        		}
        		else if ((eir & 0x10) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_LINK_STATE_ENABLE);
        			enc28j60_enet->link_status_task();
        			printf("ENC28J60_INTERRUPT_LINK_STATE_ENABLE");
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
            		//break;
        		}
        		else if ((eir & 0x08) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_TX_ENABLE);
        			enc28j60_enet->_enc28j60->tx_packetHandled();
        			printf("ENC28J60_INTERRUPT_TX_ENABLE");
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
            		//break;
        		}
        		else if ((eir & 0x02) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_TX_ERROR_ENABLE);
        			printf("ENC28J60_INTERRUPT_TX_ERROR_ENABLE");
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
            		//break;
        		}
        		else if ((eir & 0x01) > 0){
        			enc28j60_enet->_enc28j60->clear_interrupt(ENC28J60_INTERRUPT_RX_ERROR_ENABLE);
        			printf("ENC28J60_INTERRUPT_RX_ERROR_ENABLE");
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
            		//break;
        		}
        		else {
        			enc28j60_enet->_enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);
        			//break;
        		}
        	}
        }
    }
}


/** \brief  Packet reception task
 *
 * This task is called when a packet is received. It will
 * pass the packet to the Network Stack.
 */
void ENC28J60_EMAC::packet_rx()
{
    emac_mem_buf_t *p;
    p = low_level_input();
    if(p != NULL) {
    	if (_emac_link_input_cb){
            _emac_link_input_cb(p);
            _enc28j60->rx_packetHandled();
    	}
    }
}

bool ENC28J60_EMAC::link_out(emac_mem_buf_t *buf)
{
	uint8_t *data_ptr;
    if(buf == NULL) {
        return false;
    } else {
        //uint32_t buffer_chain_length = 0;
        enc28j60_error_t error = ENC28J60_ERROR_NONE;
        /* If buffer is chained or not aligned then
         * make a contiguous aligned copy of it */
        if (_memory_manager->get_next(buf) ||
            reinterpret_cast<uint32_t>(_memory_manager->get_ptr(buf))) {
            emac_mem_buf_t *copy_buf;
            copy_buf = _memory_manager->alloc_heap(_memory_manager->get_total_len(buf),
                                             ENC28J60_BUFF_ALIGNMENT);
            if (copy_buf == NULL) {
                _memory_manager->free(buf);
                return false;
            }

            /* Copy to new buffer and free original */
            _memory_manager->copy(copy_buf, buf);
            _memory_manager->free(buf);
            buf = copy_buf;
        }

        //buffer_chain_length = _memory_manager->get_total_len(buf);

        _ETHLockMutex.lock();
        data_ptr = (uint8_t*)(_memory_manager->get_ptr(buf));
        error = _enc28j60->tx_write_data_fifo(data_ptr,
        							  0,
                                      _memory_manager->get_len(buf));
        if (error != ENC28J60_ERROR_NONE) {
        	_ETHLockMutex.unlock();
            return false;
        }
        error = _enc28j60->tx_send_next_packet();
        if (error != ENC28J60_ERROR_NONE) {
        	_ETHLockMutex.unlock();
            return false;
        }
        _ETHLockMutex.unlock();
        return true;
    }
}

void ENC28J60_EMAC::link_status_task() {
    uint16_t phy_basic_status_reg_value = 0;
    bool current_link_status_up = false;

    /* Get current status */
    _ETHLockMutex.lock();
    _enc28j60->phyRead(PHSTAT2, &phy_basic_status_reg_value);
    _ETHLockMutex.unlock();

    current_link_status_up = (bool)((phy_basic_status_reg_value & PHSTAT2_LSTAT) > 0);

    /* Compare with previous state */
    if (current_link_status_up != _prev_link_status_up) {
        if(_emac_link_state_cb){
        	_emac_link_state_cb(current_link_status_up);
        }
        _prev_link_status_up = current_link_status_up;
    }
}

bool ENC28J60_EMAC::power_up() {
	board_emac_pointer = this;
	volatile uint32_t timeout = 500;
	_enc28j60->writeOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
	while (_enc28j60->readReg(ESTAT_CLKRDY) == 0) {
		ThisThread::sleep_for(1);
		timeout--;
		if (timeout == 0) {
			return false;
		}
	}
	_rx_thread = new Thread(RX_THREAD_PRIORITY,(uint32_t)RX_THREAD_STACKSIZE);
	_rx_thread->start(callback(&ENC28J60_EMAC::receiver_thread_function, this));

    /* Initialize the hardware */
    enc28j60_error_t init_successful = _enc28j60->init();
    if (init_successful != ENC28J60_ERROR_NONE) {
        return false;
    }
    /* enable interrupts */
    _enc28j60->enable_interrupt(ENC28J60_INTERRUPT_ENABLE);

    /* Trigger thread to deal with any RX packets that arrived
     * before receiver_thread was started */
	_rx_thread->flags_set(FLAG_RX);
    _prev_link_status_up = PHY_STATE_LINK_DOWN;
    mbed::mbed_event_queue()->call(mbed::callback(this, &ENC28J60_EMAC::link_status_task));

    /* Allow the Link Status task to detect the initial link state */
    ThisThread::sleep_for(10);
    _link_status_task_handle = mbed::mbed_event_queue()->call_every(LINK_STATUS_TASK_PERIOD_MS,
                              mbed::callback(this, &ENC28J60_EMAC::link_status_task));

	_irq_thread = new Thread(IRQ_THREAD_PRIORITY,(uint32_t)IRQ_THREAD_STACKSIZE);
	_irq_thread->start(callback(&ENC28J60_EMAC::interrupt_thread_function, this));
//	irq_thread->flags_set(FLAG_IRQ);
	return true;
}

uint32_t ENC28J60_EMAC::get_mtu_size() const
{
    return ENC28J60_ETH_MTU_SIZE;
}

uint32_t ENC28J60_EMAC::get_align_preference() const
{
    return ENC28J60_BUFF_ALIGNMENT;
}

void ENC28J60_EMAC::get_ifname(char *name, uint8_t size) const
{
    memcpy(name, ENC28J60_ETH_IF_NAME, (size < sizeof(ENC28J60_ETH_IF_NAME)) ?
                                           size : sizeof(ENC28J60_ETH_IF_NAME));
}

uint8_t ENC28J60_EMAC::get_hwaddr_size() const
{
    return ENC28J60_HWADDR_SIZE;
}

bool ENC28J60_EMAC::get_hwaddr(uint8_t *addr) const
{
	enc28j60_error_t error = _enc28j60->read_mac_address((char*)addr);
    if(error == ENC28J60_ERROR_NONE) {
        return true;
    } else {
        return false;
    }
}

void ENC28J60_EMAC::set_hwaddr(const uint8_t *addr)
{
    if (!addr) {
        return;
    }

    memcpy(_hwaddr, addr, sizeof _hwaddr);
    _ETHLockMutex.lock();
	enc28j60_error_t error = _enc28j60->write_mac_address((char*)addr);
    _ETHLockMutex.unlock();
    if (error) {
        return;
    }
}

void ENC28J60_EMAC::set_link_input_cb(emac_link_input_cb_t input_cb)
{
    _emac_link_input_cb = input_cb;
}

void ENC28J60_EMAC::set_link_state_cb(emac_link_state_change_cb_t state_cb)
{
    _emac_link_state_cb = state_cb;
}

void ENC28J60_EMAC::add_multicast_group(const uint8_t *addr)
{
    // No action for now
}

void ENC28J60_EMAC::remove_multicast_group(const uint8_t *addr)
{
    // No action for now
}

void ENC28J60_EMAC::set_all_multicast(bool all)
{
    // No action for now
}

void ENC28J60_EMAC::power_down() {
	_irq_thread->terminate();
	_rx_thread->terminate();
	_enc28j60->disable_mac_recv();
	if (_enc28j60->readReg(ESTAT_RXBUSY) != 0) {
		_enc28j60->enable_mac_recv();
		return;
	}
	if (_enc28j60->readReg(ECON1_TXRTS) != 0) {
		_enc28j60->enable_mac_recv();
		return;
	}
	_enc28j60->writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_VRPS);
	_enc28j60->writeOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
	_irq_thread->join();
	_rx_thread->join();
	delete _irq_thread;
	delete _rx_thread;
}

void ENC28J60_EMAC::set_memory_manager(EMACMemoryManager &mem_mngr)
{
    _memory_manager = &mem_mngr;
}


ENC28J60_EMAC &ENC28J60_EMAC::get_instance() {
    static ENC28J60_EMAC emac;
    return emac;
}

/* Weak so a module can override */
MBED_WEAK EMAC &EMAC::get_default_instance() {
    return ENC28J60_EMAC::get_instance();
}

