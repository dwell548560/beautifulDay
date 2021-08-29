#include "bzl_modbus/ModbusUtils.h"

#include "string.h"
#include <vector>
#include <map>
#include <iostream>
#include <stdio.h>

using namespace std;

std::map<uint16_t, uint16_t> registers_info_map;

registers_info registers_arr[] = {
	// common input_registers
	{0x4000, 2},
	{0x4001, 4},
	{0x4003, 4},
	{0x4005, 2},
	{0x4006, 2},
	{0x4007, 4},
	{0x4009, 4},
	{0x400B, 4},
	{0x400D, 4},
	{0x400F, 2},
	{0x4010, 4},
	{0x4012, 2},
	{0x4013, 2},

	// power-board holding-registers
	{0x0300, 2},
	{0x0301, 2},
	{0x0302, 2},
	{0x0303, 2},
	{0x0304, 2},
	{0x0305, 2},
	{0x0306, 2},
	{0x0307, 2},
	{0x0308, 2},
	{0x0309, 2},
	{0x030A, 2},
	{0x030B, 2},
	{0x030C, 2},
	{0x030D, 2},
	{0x030E, 2},
	{0x030F, 2},

	// guawang
	{0x4400, 2},

	// dzpt_1
	{0x4800, 4},
	{0x4802, 4},
	{0x4804, 2},
	{0x4805, 2},
	{0x4806, 4},
	{0x4808, 4},
	{0x480A, 4},
	{0x480C, 4},
	{0x480E, 4},
	{0x4810, 4},
	{0x4812, 2},
	{0x4813, 2},
	{0x4814, 2},
	{0x4815, 2},

	// paint
	{0x4C00, 2},
	{0x4C01, 2},
	{0x4C02, 2},
	{0x4C03, 4},
	{0x4C05, 4},
	{0x4C07, 2},
	{0x4C08, 2},
	{0x4C09, 4},
	{0x4C0B, 2},

	// wall install
	{0x5400, 2},
	{0x5401, 2},
	{0x5402, 2},

	// wall handling
	{0x1000, 2},
	{0x1001, 2},
	{0x1002, 2},
	{0x1003, 2},
	{0x1004, 2},
	{0x1005, 2},
	{0x1006, 2},
	{0x1007, 2},
	{0x1008, 2},

	// 天花打磨
	{0x5801, 2},
	{0x5802, 2},
	{0x5803, 2},
	{0x5804, 2},
	{0x5806, 2},
	{0x5806, 2},
	{0x5820, 2},
	{0x5821, 2},
	{0x5822, 2},
	{0x5823, 2},
	{0x5824, 2},
	{0x5825, 2},
	{0x5826, 2},
	{0x5827, 2},
	{0x5828, 2},
	{0x5829, 2},
	{0x5830, 2},
	{0x5831, 2},
	{0x5832, 2},
	{0x5833, 2},
	{0x5834, 2},
	{0x5835, 2},
	{0x5836, 2},
	{0x5837, 2},
	{0x5838, 2},
	{0x5839, 2},
	{0x5840, 2},
	{0x5841, 2},
	{0x5842, 2},
	{0x5843, 2},
	{0x5844, 2},
	{0x5845, 2},
	{0x5846, 2},
	{0x5847, 2},
	{0x5848, 2},
	{0x5849, 2},
	{0x5850, 2},
	{0x5851, 2},
};

void modbus_utils_init(void)
{
	for(int i=0; i<(sizeof(registers_arr)/sizeof(registers_info)); i++)
	{
		registers_info_map.insert(std::pair<uint16_t, uint16_t>(registers_arr[i].index, i));
	}
}

void modbus_utils_deinit(void)
{
	registers_info_map.clear();
}

int modbus_register_info_get(uint16_t addr)
{
	if(registers_info_map.find(addr) != registers_info_map.end())
	{
		return registers_info_map.at(addr);
	}

	return -1;
}

int cpu_endian_check(void)
{
	int x = 1;

	if(*(char *)&x == 1)
	{
		return M_LITTLE_ENDIAN;
	}
	else
	{
		return M_BIG_ENDIAN;
	}
}

/**
 * @brief	modbus_adjust_to_bigendian
 * @addr	start addr
 * @num		bytes to adjust, must be multiple of 2
 * @src		source data
 * @dest	dest buffer
 * @desc	adjust to big-endian bytes-orders
 */
int modbus_adjust_bigendian(uint16_t addr, uint16_t num, const uint16_t *src, uint16_t *dest)
{
	int index = 0;
	registers_info *info = NULL;

	index = modbus_register_info_get(addr);
	if(index == -1) {
		memcpy(dest, src, num);
		return 0;
		//return -1;
	}

	info = &registers_arr[index];
	for(int i=num; (i>=info->bytes) && (index<(sizeof(registers_arr)/sizeof(registers_info))); index++)
	{
		if(info->bytes == 2)
		{
			*dest = *src;
		}
		else if(info->bytes == 4)
		{
			dest[0] = src[1];
			dest[1] = src[0];
		}
		else
		{
			memcpy(dest, src, info->bytes/2);
		}

		i -= info->bytes;
		dest += info->bytes/2;
		src += info->bytes/2;
	}

	return 0;
}
