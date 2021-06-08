

#ifndef __PLAT_UART_H__
#define __PLAT_UART_H__
/*****************************************************************************
  1 Include other Head file
*****************************************************************************/
#include <linux/serial_core.h>
#include "plat_type.h"
/*****************************************************************************
  2 Define macro
*****************************************************************************/
typedef  enum {
    STATE_TTY_TX = 0,
    STATE_TTY_RX = 1,
    STATE_UART_TX = 2,
    STATE_UART_RX = 3,
}UART_STATE_INDEX;
/*****************************************************************************
  3 STRUCT DEFINE
*****************************************************************************/
struct ps_uart_state_s {
    uint32_t tty_tx_cnt;
    uint32_t tty_rx_cnt;
    uint32_t tty_stopped;     /* tty 软件流控标志位 */
    uint32_t tty_hw_stopped;  /* tty 硬件流控标志位 */
    struct uart_icount uart_cnt;
};

/*****************************************************************************
  4 EXTERN VARIABLE
*****************************************************************************/

/*****************************************************************************
  5 EXTERN FUNCTION
*****************************************************************************/
extern int32_t plat_uart_init(void);
extern int32_t plat_uart_exit(void);
extern int32_t open_tty_drv(void *pm_data);
extern int32_t release_tty_drv(void *pm_data);
extern int32_t ps_change_uart_baud_rate(int64_t baud_rate, uint8_t enable_flowctl);
extern void ps_uart_tty_tx_add(uint32_t cnt);
extern void ps_uart_state_pre(struct tty_struct *tty);
extern void ps_uart_state_dump(struct tty_struct *tty);
extern uint32_t ps_uart_state_cur(uint32_t index);
#endif

