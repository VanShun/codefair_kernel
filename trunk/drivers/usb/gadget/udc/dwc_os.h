#ifndef DWC_OS_H_
#define DWC_OS_H_

#include <linux/types.h>

/** @name Primitive Types and Values */

/** We define a boolean type for consistency.  Can be either YES or NO */
typedef uint8_t dwc_bool_t;
#define YES  1
#define NO   0

typedef uint32_t dwc_dma_t;

/** @todo make them positive and return the negative error code */
/** @name Error Codes */
#define DWC_E_INVALID         1001
#define DWC_E_NO_MEMORY       1002
#define DWC_E_NO_DEVICE       1003
#define DWC_E_NOT_SUPPORTED   1004
#define DWC_E_TIMEOUT         1005
#define DWC_E_BUSY            1006
#define DWC_E_AGAIN           1007
#define DWC_E_RESTART         1008
#define DWC_E_ABORT           1009
#define DWC_E_SHUTDOWN        1010
#define DWC_E_NO_DATA         1011
#define DWC_E_DISCONNECT      2000
#define DWC_E_UNKNOWN         3000
#define DWC_E_NO_STREAM_RES   4001
#define DWC_E_COMMUNICATION   4002
#define DWC_E_OVERFLOW        4003
#define DWC_E_PROTOCOL        4004
#define DWC_E_IN_PROGRESS     4005
#define DWC_E_PIPE            4006
#define DWC_E_IO              4007
#define DWC_E_NO_SPACE        4008

/** @name Timer
 *
 * Callbacks must be small and atomic.
 */
struct dwc_timer;
typedef struct dwc_timer dwc_timer_t;

/** Reads the content of a 32-bit register. */
extern uint32_t DWC_READ_REG32(uint32_t volatile *reg);
#define dwc_read_reg32 DWC_READ_REG32
/** Reads the content of a 64-bit register. */
extern uint64_t DWC_READ_REG64(uint64_t volatile *reg);
#define dwc_read_reg64 DWC_READ_REG64
/** Writes to a 32-bit register. */
extern void DWC_WRITE_REG32(uint32_t volatile *reg, uint32_t value);
#define dwc_write_reg32 DWC_WRITE_REG32
/** Writes to a 64-bit register. */
extern void DWC_WRITE_REG64(uint64_t volatile *reg, uint64_t value);
#define dwc_write_reg64 DWC_WRITE_REG64
/**
 * Modify bit values in a register.  Using the
 * algorithm: (reg_contents & ~clear_mask) | set_mask.
 */
extern void DWC_MODIFY_REG32(uint32_t volatile *reg, uint32_t clear_mask, uint32_t set_mask);
#define dwc_modify_reg32 DWC_MODIFY_REG32


/** Microsecond delay.
 *
 * @param usecs  Microseconds to delay.
 */
extern void DWC_UDELAY(uint32_t usecs);
#define dwc_udelay DWC_UDELAY

/** Millisecond delay.
 *
 * @param msecs  Milliseconds to delay.
 */
extern void DWC_MDELAY(uint32_t msecs);
#define dwc_mdelay DWC_MDELAY


extern void __DWC_ERROR(char *format, ...);


#define dwc_free DWC_FREE



#define DWC_FREE(_addr_) __DWC_FREE(_addr_)


#endif

