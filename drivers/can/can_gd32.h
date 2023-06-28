/*
 * Copyright (c) 2023 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_GD32_H_
#define ZEPHYR_DRIVERS_CAN_GD32_H_

#include <zephyr/drivers/can.h>
//#include "gd32e50x_can.h"

#define CAN_GD32_NUM_FILTER_BANKS (14)
#define CAN_GD32_MAX_FILTER_ID \
	(CONFIG_CAN_MAX_EXT_ID_FILTER + CONFIG_CAN_MAX_STD_ID_FILTER * 2)

#define CAN_GD32_FIRX_STD_IDE_POS    (3U)
#define CAN_GD32_FIRX_STD_RTR_POS    (4U)
#define CAN_GD32_FIRX_STD_ID_POS     (5U)

#define CAN_GD32_FIRX_EXT_IDE_POS    (2U)
#define CAN_GD32_FIRX_EXT_RTR_POS    (1U)
#define CAN_GD32_FIRX_EXT_STD_ID_POS (21U)
#define CAN_GD32_FIRX_EXT_EXT_ID_POS (3U)

/*******************  Definition for CAN frame format  *******************/
#define CAN_RFIFOMI_RTR_OFFSET       (1U)
#define CAN_RFIFOMIRTR_Msk       	 CAN_RFIFOMI_FT                                    /*!< 0x00000002 */
#define CAN_RFIFOMI_RTR           	 CAN_RFIFOMIRTR_Msk                                /*!<Remote Transmission Request */
#define CAN_RFIFOMI_IDE_OFFSET       (2U)
#define CAN_RFIFOMI_IDE_Msk          CAN_RFIFOMI_FF                                     /*!< 0x00000004 */
#define CAN_RFIFOMI_IDE              CAN_RFIFOMI_IDE_Msk                                /*!<Identifier Extension */
#define CAN_RFIFOMI_EXID_OFFSET      (3U)
#define CAN_RFIFOMI_EXID_Msk         (0x3FFFFUL << CAN_RFIFOMI_EXID_OFFSET)                 /*!< 0x001FFFF8 */
#define CAN_RFIFOMI_EXID             CAN_RFIFOMI_EXID_Msk                               /*!<Extended Identifier */
#define CAN_RFIFOMI_STID_POFFSET     (21U)
#define CAN_RFIFOMI_STID_Msk         (0x7FFUL << CAN_RFIFOMI_STID_POFFSET)                   /*!< 0xFFE00000 */
#define CAN_RFIFOMI_STID             CAN_RFIFOMI_STID_Msk                               /*!<Standard Identifier or Extended Identifier */

#define CAN_RFIFOMP_TS_OFFSET		 (16U)
#define CAN_RFIFOMP_FI_OFFSET		 (8U)
#define CAN_ERR_TECNT_OFFSET         (16U)
#define CAN_ERR_RECNT_OFFSET         (24U)

/**
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
volatile uint32_t CANX_TMI;  /*!< CAN TX mailbox identifier register */
volatile uint32_t CANX_TMP; /*!< CAN mailbox data length control and time stamp register */
volatile uint32_t CANX_TMDATA0; /*!< CAN mailbox data low register */
volatile uint32_t CANX_TMDATA1; /*!< CAN mailbox data high register */
} CAN_TxMail_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
volatile uint32_t CANX_RFIFOMI;  /*!< CAN receive FIFO mailbox identifier register */
volatile uint32_t CANX_RFIFOMP; /*!< CAN receive FIFO mailbox data length control and time stamp register */
volatile uint32_t CANX_RFIFODATA0; /*!< CAN receive FIFO mailbox data low register */
volatile uint32_t CANX_RFIFODATA1; /*!< CAN receive FIFO mailbox data high register */
} CAN_RxFIFOMail_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
volatile uint32_t CANX_FxDATA0; 		/*!< CAN Filter bank register 0 */
volatile uint32_t CANX_FxDATA1; 		/*!< CAN Filter bank register 1 */
} CAN_FilterReg_TypeDef;


typedef struct {
volatile uint32_t              CANX_CTL;                   /*!< CAN master control register,         Address offset: 0x00          */
volatile uint32_t              CANX_STAT;                  /*!< CAN master status register,          Address offset: 0x04          */
volatile uint32_t              CANX_TSTAT;                 /*!< CAN transmit status register,        Address offset: 0x08          */
volatile uint32_t              CANX_RFIFO0;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
volatile uint32_t              CANX_RFIFO1;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
volatile uint32_t              CANX_INTEN;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
volatile uint32_t              CANX_ERR;                   /*!< CAN error status register,           Address offset: 0x18          */
volatile uint32_t              CANX_BT;                    /*!< CAN bit timing register,             Address offset: 0x1C          */
 #ifdef CONFIG_CAN_FD_MODE 
volatile uint32_t              CANX_FDCTL;
volatile uint32_t              CANX_FDSTAT;
volatile uint32_t              CANX_FDTDC; 
volatile uint32_t              CANX_DBT; 
uint32_t                       RESERVED0[84];       		 /*!< Reserved, 0x030 - 0x17F                                            */
 #else
  uint32_t                     RESERVED0[88];       		 /*!< Reserved, 0x020 - 0x17F                                            */
 #endif
  CAN_TxMail_TypeDef           TXMAIL[3];                /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_RxFIFOMail_TypeDef       RXFIFOMAIL[2];                /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                     RESERVED1[12];       		    /*!< Reserved, 0x1D0 - 0x1FF                                            */
volatile uint32_t              CANX_FCTL;                     /*!< CAN filter master register,          Address offset: 0x200         */
volatile uint32_t              CANX_FMCFG;                	/*!< CAN filter mode register,            Address offset: 0x204         */
uint32_t                       RESERVED2;           			/*!< Reserved, 0x208                                                    */
volatile uint32_t              CANX_FSCFG;                	/*!< CAN filter scale register,           Address offset: 0x20C         */
uint32_t                       RESERVED3;           			/*!< Reserved, 0x210                                                    */
volatile uint32_t              CANX_FAFIFO;               	/*!< CAN filter FIFO assignment register, Address offset: 0x214         */
uint32_t                       RESERVED4;           			/*!< Reserved, 0x218                                                    */
volatile uint32_t              CANX_FW;                		/*!< CAN filter activation register,      Address offset: 0x21C         */
uint32_t                       RESERVED5[8];        			/*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterReg_TypeDef        CANX_FilterReg[28];			/*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} GD_CAN_TypeDef;

struct can_gd32_mailbox {
	can_tx_callback_t tx_callback;
	void *callback_arg;
};

struct can_gd32_data {
	struct k_mutex inst_mutex;
	struct k_sem tx_int_sem;
	struct can_gd32_mailbox mb0;
	struct can_gd32_mailbox mb1;
	struct can_gd32_mailbox mb2;
	can_rx_callback_t rx_cb_std[CONFIG_CAN_MAX_STD_ID_FILTER];
	can_rx_callback_t rx_cb_ext[CONFIG_CAN_MAX_EXT_ID_FILTER];
	void *cb_arg_std[CONFIG_CAN_MAX_STD_ID_FILTER];
	void *cb_arg_ext[CONFIG_CAN_MAX_EXT_ID_FILTER];
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	enum can_state state;
	bool started;
};

struct can_gd32_config {
	GD_CAN_TypeDef *can;   /*!< CAN Registers*/
	GD_CAN_TypeDef *master_can;   /*!< CAN Registers for shared filter */
	uint32_t bus_speed;
	uint16_t sample_point;
	uint8_t sjw;
	uint8_t prop_ts1;
	uint8_t ts2;
    uint32_t bus_speed_data;
	uint16_t sample_point_data;
	uint8_t sjw_data;
	uint8_t prop_ts1_data;
	uint8_t ts2_data;
	struct gd32_pclken pclken;
	void (*config_irq)(GD_CAN_TypeDef *can);
	const struct pinctrl_dev_config *pcfg;
	const struct device *phy;
	uint32_t max_bitrate;
	uint8_t tx_delay_comp_offset;
};

#endif /* ZEPHYR_DRIVERS_CAN_GD32_H_ */
