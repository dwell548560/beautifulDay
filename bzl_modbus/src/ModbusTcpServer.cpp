#include "bzl_modbus/ModbusTcpServer.h"
#include "string.h"

#define PROTO_TYPE_INDEX 2
#define PACK_LEN_INDEX 4
#define MODBUS_CMD_INDEX 7
#define RES_BYTE_NUM_INDEX 8
#define REG_ADDR_INDEX 8
#define REG_NUM_INDEX 10

// 大端模式解析字数据
uint16_t get_word_big(unsigned char* buf)
{
	return (uint16_t)((buf[0] << 8) | buf[1]);
}

void ModbusTcpServer::setup(int port)
{
	m_port = port;
}

ModbusTcpServer::~ModbusTcpServer()
{
	if (m_server_socket != -1) {
		close(m_server_socket);
	}
	modbus_free(m_ctx);

	std::map<int, modbus_mapping_t*>::iterator i;

	for(i=m_map_all.begin(); i != m_map_all.end(); ++i)
	{
		modbus_mapping_free((*i).second);
	}
}

void ModbusTcpServer::register_recive_callback(FUNC_RECIVE_CALLBACK func)
{
	m_recive_callback = func;
}

bool ModbusTcpServer::add_new_mapping(int mapping_index,
		unsigned int start_bits, unsigned int nb_bits,
		unsigned int start_input_bits, unsigned int nb_input_bits,
		unsigned int start_registers, unsigned int nb_registers,
		unsigned int start_input_registers, unsigned int nb_input_registers)
{

	printf("[%s:%d]: mapping = %d, start_bits = 0x%02x, nb_bits = %d, start_input_bits = 0x%02x, nb_input_bits = %d\n", \
			__func__, __LINE__, mapping_index, start_bits, nb_bits, start_input_bits, nb_input_bits);
	printf("\t\tstart_registers = 0x%02x, nb_registers = %d, start_input_registers = 0x%02x, nb_input_registers = %d\n", \
			start_registers, nb_registers, start_input_registers, nb_input_registers);
	printf("---------------------------------------------------------------------------------------------------------\n");

	modbus_mapping_t * new_mapping = modbus_mapping_new_start_address(start_bits, nb_bits,
			start_input_bits, nb_input_bits, start_registers, nb_registers, start_input_registers, nb_input_registers);

	if(new_mapping == NULL)
	{
		return false;
	}

	m_map_all.insert(std::pair<int, modbus_mapping_t*>(mapping_index, new_mapping));
	return true;
}

modbus_mapping_t* ModbusTcpServer::get_mapping(int mapping_index)
{
	if(m_map_all.find(mapping_index) != m_map_all.end())
	{
		return m_map_all.at(mapping_index);
	}
	return NULL;
}

void ModbusTcpServer::set_recive_mapping(int mapping_index)
{
	mb_mapping = get_mapping(mapping_index);
}

// 线圈地址 线圈数量 buf每一位为一个线圈状态
void ModbusTcpServer::fill_read_bits_memory(int mapping_index, int addr, int num, const uint8_t* buf)
{
	modbus_mapping_t* map = get_mapping(mapping_index);
	if(map)
	{
		modbus_set_bits_from_bytes(map->tab_input_bits, addr, num, buf);
	}
}

void ModbusTcpServer::fill_write_bits_memory(int mapping_index, int addr, int num, const uint8_t* buf)
{
	modbus_mapping_t* map = get_mapping(mapping_index);
	if(map)
	{
		modbus_set_bits_from_bytes(map->tab_bits, addr, num, buf);
	}
}

void ModbusTcpServer::fill_read_reg_memory(int mapping_index, int addr, int num, const uint16_t* buf)
{
	modbus_mapping_t* map = get_mapping(mapping_index);
	if(map)
	{
		memcpy(map->tab_input_registers + addr, buf, 2*num);
	}
}

void ModbusTcpServer::fill_write_reg_memory(int mapping_index, int addr, int num, const uint16_t* buf)
{
	modbus_mapping_t* map = get_mapping(mapping_index);
	if(map)
	{
		memcpy(map->tab_registers + addr, buf, 2*num);
	}
}

void ModbusTcpServer::get_write_reg_memory(int mapping_index, int addr, int num, uint16_t* buf)
{
	modbus_mapping_t* map = get_mapping(mapping_index);
	if(map)
	{
		memcpy(buf, map->tab_registers + addr, 2*num);
	}
	else
	{
		memset(buf, 0x00, num*2);
	}
}

S_ModbusMessage ModbusTcpServer::parse_modbus_mess(unsigned char* buf, int len)
{
	S_ModbusMessage mess;
	unsigned char modbus_cmd = buf[MODBUS_CMD_INDEX];
	mess.func_code = modbus_cmd;
	mess.reg_addr = get_word_big(&buf[REG_ADDR_INDEX]);
	mess.reg_num = get_word_big(&buf[REG_NUM_INDEX]);
	mess.data.clear();

	if(modbus_cmd == MODBUS_FC_WRITE_SINGLE_REGISTER)
	{
		mess.reg_num = 1;
		mess.data.push_back(get_word_big(&buf[hdrlen + 3]));
	}
	else if(modbus_cmd == MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
	{
		std::vector<uint16_t> vec;
		//printf("value: ");
		for(int i=0; i<mess.reg_num; ++i)
		{
			uint16_t val = get_word_big(&buf[hdrlen + 6] + 2*i);
		//	printf("0x%02x ", val);
			vec.push_back(val);
		}
		//printf("\n");
		mess.data = vec;
	}

	return mess;
}

int ModbusTcpServer::start_server()
{
	uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
	int master_socket;
	int rc;
	fd_set refset;
	fd_set rdset;
	/* Maximum file descriptor number */
	int fdmax;

	m_ctx = modbus_new_tcp("0.0.0.0", m_port);

	//modbus_set_debug(m_ctx, 1);

	hdrlen = modbus_get_header_length(m_ctx);

	m_server_socket = modbus_tcp_listen(m_ctx, NB_CONNECTION);

	if (m_server_socket == -1) {
		fprintf(stderr, "Unable to listen TCP connection\n");
		modbus_free(m_ctx);
		return -1;
	}

	/* Clear the reference set of socket */
	FD_ZERO(&refset);
	/* Add the server socket */
	FD_SET(m_server_socket, &refset);

	/* Keep track of the max file descriptor */
	fdmax = m_server_socket;

	for (;;) {
		rdset = refset;
		if (select(fdmax+1, &rdset, NULL, NULL, NULL) == -1) {
			perror("Server select() failure.");
		}

		/* Run through the existing connections looking for data to be
		 * read */
		for (master_socket = 0; master_socket <= fdmax; master_socket++) {

			if (!FD_ISSET(master_socket, &rdset)) {
				continue;
			}

			if (master_socket == m_server_socket) {
				/* A client is asking a new connection */
				socklen_t addrlen;
				struct sockaddr_in clientaddr;
				int newfd;

				/* Handle new connections */
				addrlen = sizeof(clientaddr);
				memset(&clientaddr, 0, sizeof(clientaddr));
				newfd = accept(m_server_socket, (struct sockaddr *)&clientaddr, &addrlen);
				if (newfd == -1) {
					perror("Server accept() error");
				} else {
					FD_SET(newfd, &refset);

					if (newfd > fdmax) {
						/* Keep track of the maximum */
						fdmax = newfd;
					}
					printf("New connection from %s:%d on socket %d\n",
							inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
				}
			} else {
				modbus_set_socket(m_ctx, master_socket);
				rc = modbus_receive(m_ctx, query);
				if (rc > 0) {

					// fan add
					if(m_recive_callback != NULL)
					{
						(*m_recive_callback)(this, parse_modbus_mess(query, rc));
					}
					// fan add end

					if(mb_mapping == NULL)
					{
						continue;
					}

					modbus_reply(m_ctx, query, rc, mb_mapping);

				} else if (rc == -1) {
					/* This example server in ended on connection closing or
					 * any errors. */
					printf("Connection closed on socket %d\n", master_socket);
					close(master_socket);

					/* Remove from reference set */
					FD_CLR(master_socket, &refset);

					if (master_socket == fdmax) {
						fdmax--;
					}
				}
			}
		}
	}
}
