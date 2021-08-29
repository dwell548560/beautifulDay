#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <vector>
#include <map>

#include <strings.h>

#include <pthread.h>
#include <signal.h>

#include <modbus/modbus.h>
#include <sys/socket.h>

#include "bzl_modbus/modbus_common.h"
#include "bzl_modbus/Modbus.h"
#include "bzl_modbus/ModbusStatus.h"
#include "bzl_modbus/ModbusUtils.h"

static const uint32_t DEFAULT_MODBUS_INTERVAL = 100 * 1000; //100ms

enum {
	TCP = 0,
    RTU = 1
};

//using namespace std;

//global variables
modbus_t *modbus_ctx = NULL;
pthread_t g_tid = -1;
bool g_terminal = false;

int g_cpu_endian = M_LITTLE_ENDIAN;

ros::Publisher g_pub_modbus;
ros::Subscriber g_sub_modbus;
ros::Publisher g_pub_modbus_status;

std::string g_ip_str;
static int g_interval = DEFAULT_MODBUS_INTERVAL;
static int g_backend = TCP;
static int g_debug = FALSE;

std::string g_rtu_port("/dev/ttyTHS1");
int g_rtu_baudrate = 115200;
int g_slave_id = 0x10;
int g_tcp_port = 1502;

pthread_mutex_t g_mutex;

class RegInfo {
private:
	uint16_t holding_addr;
	uint16_t holding_cnts;
	uint16_t input_addr;
	uint16_t input_cnts;
	uint16_t *holding_data;
	uint16_t *input_data;
	uint32_t reg_masks; // only for holding regs
	uint16_t *masks_ext;

public:
	RegInfo(uint16_t h_addr, uint16_t h_cnts, uint16_t i_addr, uint16_t i_cnts)
		:holding_addr(h_addr), holding_cnts(h_cnts),
		 input_addr(i_addr), input_cnts(i_cnts)
	{
		holding_data = new uint16_t[holding_cnts];
		memset(holding_data, 0x0, sizeof(uint16_t) * holding_cnts);

		masks_ext = new uint16_t[holding_cnts/16 + 1];
		memset(masks_ext, 0x0, sizeof(uint16_t) * (holding_cnts/16 + 1));

		reg_masks = 0;

		input_data = new uint16_t[input_cnts];
		memset(input_data, 0x0, sizeof(uint16_t) * input_cnts);
	};

	~RegInfo() {
		delete[] holding_data;
		delete[] input_data;
	};

	void mask_set(uint16_t index, uint16_t cnts = 1)
	{
		if(index > holding_cnts || index + cnts > holding_cnts)
			return;

		uint16_t bits = index >> 4;
		uint16_t offs = index & 0x0F;

		while(cnts != 0)
		{
			reg_masks |= (1U << bits);
			for(uint16_t i = offs; cnts != 0 && i < 16; i++, cnts--)
			{
				masks_ext[bits] |= (1U << i);
			}

			offs = 0;
			bits++;
		}
	}

	void mask_clear(uint16_t index, uint16_t cnts = 1)
	{
		if(index > holding_cnts || index + cnts > holding_cnts)
			return;

		uint16_t bits = index >> 4;
		uint16_t offs = index & 0x0F;
		while(cnts != 0)
		{
			for(uint16_t i = offs; cnts != 0 && i < 16; i++, cnts--)
			{
				masks_ext[bits] &= ~(1U << i);
			}

			if(masks_ext[bits] == 0x00) {
				reg_masks &= ~(1U << bits);
			}

			offs = 0;
			bits++;
		}
	}

	uint32_t get_reg_masks(void)
	{
		return reg_masks;
	}

	uint16_t get_masks_ext(uint16_t index)
	{
		return masks_ext[index];
	}

	uint16_t get_holding_addr()
	{
		return holding_addr;
	};

	uint16_t get_input_addr()
	{
		return input_addr;
	};

	uint16_t *get_holding_data()
	{
		return holding_data;
	};

	uint16_t *get_input_data()
	{
		return input_data;
	};

	inline uint16_t get_holding_cnts()
	{
		return holding_cnts;
	};

	inline uint16_t get_input_cnts()
	{
		return input_cnts;
	};

	int holding_index(uint16_t addr)
	{
		if(addr < holding_addr || addr > holding_addr + holding_cnts)
			return -1;

		return addr - holding_addr;
	};

	int input_index(uint16_t addr)
	{
		if(addr < input_addr || addr > input_addr + input_cnts)
			return -1;

		return addr - input_addr;
	};
};

std::map<uint16_t, RegInfo *> g_regs_map;
RegInfo *g_reginfo;

void *modbus_client(void *arg)
{
	bzl_modbus::ModbusStatus st;
	uint16_t failed_cnts = 0;

	while(!g_terminal)
	{
		// modbus_slave off-line
		if(failed_cnts > 20)
		{
			ROS_INFO("Disconnected!");
			st.status = 0xFFFF;
            g_pub_modbus_status.publish(st);
			failed_cnts--;
			usleep(g_interval);
			continue;
		}
#if 1
		pthread_mutex_lock(&g_mutex);
		uint32_t masks = g_reginfo->get_reg_masks();
		uint16_t holding_addr = g_reginfo->get_holding_addr();
		uint16_t *holding_data = g_reginfo->get_holding_data();
		for(uint16_t i = 0; masks != 0; i++, masks >>= 1)
		{
			if((masks & 0x01) == 0)
				continue;

			uint16_t mask16 = g_reginfo->get_masks_ext(i);
			if(mask16 == 0)
				continue;

			for(uint16_t j = 0; mask16 != 0; j++, mask16 >>= 1)
			{
				if((mask16 & 0x01) == 0)
					continue;

				uint16_t offset = 16 * i + j;
				if(modbus_write_registers(modbus_ctx, holding_addr + offset,
							1, holding_data + offset) == -1)
				{
					pthread_mutex_unlock(&g_mutex);
					failed_cnts++;
					modbus_close(modbus_ctx);
					modbus_connect(modbus_ctx);
					usleep(g_interval);
					continue;
				}

				ROS_INFO_NAMED("ros_to_modbus", "modbus_write_registers: addr: 0x%02x, data: 0x%02x",
						holding_addr + offset, holding_data[offset]);

				usleep(100);

				g_reginfo->mask_clear(offset);
			}
		}

		if(modbus_read_input_registers(modbus_ctx, g_reginfo->get_input_addr(),
					g_reginfo->get_input_cnts(), g_reginfo->get_input_data()) < 0)
		{
			pthread_mutex_unlock(&g_mutex);
			failed_cnts++;
			modbus_close(modbus_ctx);
			modbus_connect(modbus_ctx);
			usleep(g_interval);
			continue;
		}
		pthread_mutex_unlock(&g_mutex);

		failed_cnts = 0;

		st.status = 0x0000;
        g_pub_modbus_status.publish(st);
#else
		uint16_t holding_cnts = g_cur_reginfo->get_holding_cnts();
		uint16_t input_cnts = g_cur_reginfo->get_input_cnts();
		uint16_t *holding_data = g_cur_reginfo->get_holding_data();
		uint16_t *input_data = g_cur_reginfo->get_input_data();

		ROS_INFO("holding_data: ");
		for(int i = 0; i < holding_cnts; i++)
		{
			ROS_INFO("data[%d]: 0x%02x", i, holding_data[i]);
		}

		ROS_INFO("input_data: ");
		for(int i = 0; i < input_cnts; i++)
		{
			ROS_INFO("data[%d]: 0x%02x", i, input_data[i]);
		}
#endif
		usleep(g_interval);
	}
}

void handle_ros_to_modbus(const bzl_modbus::Modbus::ConstPtr& modbus)
{
	if(g_debug) {
		ROS_INFO_NAMED("ros_to_modbus", "cmd: 0x%02x, addr: 0x%02x, num: %d",
				modbus->cmd, modbus->addr, modbus->num);
		for(int i=0; i<modbus->data.size(); i++)
		{
			ROS_INFO_NAMED("ros_to_modbus", "[%d] = 0x%02x", i, modbus->data[i]);
		}
		}

	switch(modbus->cmd)
	{
		case ROS_TO_MODBUS_CMD_WRITE:
			{
				int index = g_reginfo->holding_index(modbus->addr);
				if(index == -1)
					break;

				int regs = modbus->num;
				if(modbus->num != modbus->data.size() && modbus->data.size() != 0)
				{
					regs = modbus->data.size();
				}
				else if(!modbus->num || !modbus->data.size())
				{
					break;
				}

				int bytes = regs * sizeof(uint16_t);

				// translate to big_endian
				if(g_cpu_endian == M_LITTLE_ENDIAN)
				{
					uint16_t *data = (uint16_t *)malloc(bytes);
					if(data == NULL) {
						ROS_WARN("[%s:%d] malloc failed", __func__, __LINE__);
						break;
					}

					memset(data, 0x0000, bytes);
					if(!modbus_adjust_bigendian(modbus->addr, bytes, &modbus->data[0], data))
					{
						pthread_mutex_lock(&g_mutex);
						memcpy(&g_reginfo->get_holding_data()[index], data, regs * sizeof(uint16_t));
						g_reginfo->mask_set(index, regs);
						pthread_mutex_unlock(&g_mutex);
					}

					free(data);
				}
				else
				{
					pthread_mutex_lock(&g_mutex);
					memcpy(&g_reginfo->get_holding_data()[index], (uint16_t *)&modbus->data[0], regs * sizeof(uint16_t));
					g_reginfo->mask_set(index, regs);
					pthread_mutex_unlock(&g_mutex);
					//if(modbus_write_registers(modbus_ctx, modbus->addr, regs, (uint16_t *)&modbus->data[0]) != -1)
					//	break;
				}

				// send request as response
				// g_pub_modbus.publish(modbus);
			}
			break;
		case ROS_TO_MODBUS_CMD_READ:
			{
				if(!modbus->num)
					break;

				uint16_t *data = NULL;
				int index = g_reginfo->input_index(modbus->addr);
				if(index == -1)
				{
					index = g_reginfo->holding_index(modbus->addr);
					if(index == -1)
						break;

					data = &g_reginfo->get_holding_data()[index];
				}
				else
				{
					data = &g_reginfo->get_input_data()[index];
				}

				bzl_modbus::Modbus modbus_msg;

				modbus_msg.cmd = ROS_TO_MODBUS_CMD_READ;
				modbus_msg.addr = modbus->addr;
				modbus_msg.num = modbus->num;
				modbus_msg.data.clear();

				pthread_mutex_lock(&g_mutex);

				for(int i=0; i<modbus->num; i++) {
					modbus_msg.data.push_back(data[i]);
				}
				pthread_mutex_unlock(&g_mutex);

				g_pub_modbus.publish(modbus_msg);
			}
			break;
	}
}

void regs_init(void)
{
	g_regs_map.insert(std::pair<uint16_t, RegInfo*>(0x2000, new RegInfo(0x2000, 48, 0x6000, 32)));
	g_reginfo = g_regs_map[0x2000];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modbus_client");

	modbus_utils_init();
	g_cpu_endian = cpu_endian_check();

	ros::NodeHandle nh;
	ros::NodeHandle nh_local("~");

	std::string s_backend;
	nh_local.getParam("backend", s_backend);
	nh_local.getParam("slave_id", g_slave_id);
	nh_local.getParam("debug", g_debug);
	nh_local.getParam("update_interval", g_interval);

	ROS_INFO("backend: %s", s_backend.c_str());
	ROS_INFO("slave_id: %d", g_slave_id);
	if(strcasecmp(s_backend.c_str(), "RTU") == 0)
	{
		g_backend = RTU;
		nh_local.getParam("rtu_port", g_rtu_port);
		nh_local.getParam("rtu_baudrate", g_rtu_baudrate);
        // modbus_ctx = modbus_new_rtu(g_rtu_port.c_str(), g_rtu_baudrate, 'N', 8, 1);
        modbus_ctx = modbus_new_rtu("/dev/ttyTHS1", 115200, 'N', 8, 1);
		ROS_INFO("rtu_port: %s", g_rtu_port.c_str());
		ROS_INFO("rtu_baudrate: %d", g_rtu_baudrate);
	}
	else
	{
		g_backend = TCP;
		nh_local.getParam("server_ip", g_ip_str);
		nh_local.getParam("port", g_tcp_port);
		modbus_ctx = modbus_new_tcp(g_ip_str.c_str(), g_tcp_port);
		ROS_INFO("server_ip: %s", g_ip_str.c_str());
		ROS_INFO("port: %d", g_tcp_port);
	}

	ROS_INFO("debug: %s", g_debug ? "ON" : "OFF");
	ROS_INFO("update_interval: %dus", g_interval);

	if(modbus_ctx == NULL)
	{
		ROS_WARN("modbus init failed");
		return -1;
	}

	modbus_set_debug(modbus_ctx, g_debug);
	modbus_set_error_recovery(modbus_ctx, 
			(modbus_error_recovery_mode)(MODBUS_ERROR_RECOVERY_LINK
				| MODBUS_ERROR_RECOVERY_PROTOCOL));

	modbus_set_slave(modbus_ctx, g_slave_id);
	modbus_set_response_timeout(modbus_ctx, 0, 100*1000);

	regs_init();
	pthread_mutex_init(&g_mutex, NULL);

	uint32_t try_cnts = 0;

	// Trying connecting to modbus_server 
	ROS_INFO("Trying connecting to modbus_server %s!", g_ip_str.c_str());

	while(ros::ok())
	{
        if(modbus_connect(modbus_ctx) != -1)
		{
			ROS_INFO("Trying connecting to modbus_server succed!");
			break;
		}
		else
		{
			ROS_WARN("Trying connecting to modbus_server timeout, retry %d times", try_cnts);
		}
		try_cnts++;
        sleep(1);
	}

#if 0
	std::vector<uint16_t> data;
	for(int i=0; i<15; i+=2)
	{
		data.push_back(0xFFFF);
		data.push_back(0x0000);
	}

	//start test
	int rc = modbus_write_registers(modbus_ctx,
			0x0300,
			15,
			(uint16_t *)&data[0]);
	//perror("modbus_write");

	usleep(1000 * 1000);

	uint16_t rbuf[32];
	memset(rbuf, 0, 32);
	rc = modbus_read_input_registers(modbus_ctx, 0x4300, 3, rbuf);
	//perror("modbus_write");
	//end test
#endif

    g_pub_modbus_status = nh.advertise <bzl_modbus::ModbusStatus> ("modbus_status", 100);
	g_pub_modbus = nh.advertise <bzl_modbus::Modbus> ("modbus_to_ros", 100);
	g_sub_modbus = nh.subscribe <bzl_modbus::Modbus> ("ros_to_modbus", 100, handle_ros_to_modbus);

	pthread_create(&g_tid, NULL, modbus_client, NULL);

	ros::spin();

#if 0
	ros::Rate loop_rate(1000);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	g_terminal = true;
	pthread_join(g_tid, NULL);

	g_sub_modbus.shutdown();
	g_pub_modbus.shutdown();
	g_pub_modstatus.shutdown();

	modbus_utils_deinit();
#endif

	return 0;
}
