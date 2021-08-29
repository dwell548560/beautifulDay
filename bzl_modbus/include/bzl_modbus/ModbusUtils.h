#ifndef __MODBUS_UTILS_H__
#define __MODBUS_UTILS_H__

#include <stdint.h>

enum _cpu_endian {
	M_BIG_ENDIAN = 0x01,
	M_LITTLE_ENDIAN = 0x02,
};

typedef struct _registers_info {
	uint16_t index;
	uint16_t bytes;
} registers_info;

void modbus_utils_init();
void modbus_utils_deinit();
int cpu_endian_check(void);
int modbus_adjust_bigendian(uint16_t addr, uint16_t num, const uint16_t *src, uint16_t *dest);

#endif
