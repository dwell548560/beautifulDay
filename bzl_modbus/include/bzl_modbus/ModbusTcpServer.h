#ifndef MODBUS_TCP_SERVER_H
#define MODBUS_TCP_SERVER_H

#include <stdio.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <stdlib.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

extern "C"{
#include <modbus/modbus.h>
}

#include "bzl_modbus/modbus_common.h"

#include <vector>
#include <map>

#define NB_CONNECTION    5

// modbus帧消息转化结构体
typedef struct
{
	uint8_t  cmd;
    uint8_t  func_code;
    uint16_t reg_addr;
    uint16_t reg_num;
    std::vector<uint16_t> data;
}S_ModbusMessage;


class ModbusTcpServer;

typedef void (*FUNC_RECIVE_CALLBACK)(ModbusTcpServer* server, S_ModbusMessage modbus_mess);

class ModbusTcpServer
{

public:
    ~ModbusTcpServer();

    void setup(int port);

    // 创建一个新的数据存储区
    bool add_new_mapping(int mapping_index,
        unsigned int start_bits, unsigned int nb_bits,
        unsigned int start_input_bits, unsigned int nb_input_bits,
        unsigned int start_registers, unsigned int nb_registers,
        unsigned int start_input_registers, unsigned int nb_input_registers);

    // 获取数据存储区
    modbus_mapping_t* get_mapping(int mapping_index);

    // 设置当前接收数据存储区
    void set_recive_mapping(int mapping_index);

    // 填充数据到数据存储区
    void fill_read_bits_memory(int mapping_index, int addr, int num, const uint8_t* buf);
    void fill_write_bits_memory(int mapping_index, int addr, int num, const uint8_t* buf);
    void fill_read_reg_memory(int mapping_index, int addr, int num, const uint16_t* buf);
    void fill_write_reg_memory(int mapping_index, int addr, int num, const uint16_t* buf);

	// 从数据存储区获取数据
    void get_write_reg_memory(int mapping_index, int addr, int num, uint16_t* buf);

    // 注册接收回调
    void register_recive_callback(FUNC_RECIVE_CALLBACK func);
    
    // 开始服务
    int start_server();

private:
    // 将接收到的数据转化为S_ModbusMessage格式
    S_ModbusMessage parse_modbus_mess(unsigned char* buf, int len);

    // 数据接收回调
    FUNC_RECIVE_CALLBACK m_recive_callback;

    // 当前使用的数据存储区
    modbus_mapping_t *mb_mapping;

    // 端口号
    int m_port;

    // socket
    int m_server_socket;

    // modbus_t
    modbus_t *m_ctx;

	// modbus_hdrlen
	int hdrlen;

    // 数据存储区字典
    std::map<int, modbus_mapping_t*> m_map_all;

};


#endif

