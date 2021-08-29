#include "ros/ros.h"
#include "bzl_modbus/ModbusUtils.h"
#include "bzl_modbus/ModbusTcpServer.h"
#include <bzl_modbus/Modbus.h>

//cpu endian
int g_cpu_endian = M_LITTLE_ENDIAN;

//global variables
pthread_t g_tid = -1;

//debug flags
int g_debug = 0;

ros::Publisher pub_modbus;
ros::Subscriber sub_modbus;

ModbusTcpServer * g_modbus_tcp_server = NULL;

void on_tcp_recived(ModbusTcpServer* server, S_ModbusMessage modbus_mess)
{
	// 发布topic
	if(modbus_mess.func_code == MODBUS_FC_WRITE_SINGLE_REGISTER
			|| modbus_mess.func_code == MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
	{
        bzl_modbus::Modbus modbus_msg;
		modbus_msg.addr = modbus_mess.reg_addr;
		modbus_msg.num = 2 * modbus_mess.reg_num;
		modbus_msg.data = modbus_mess.data;

		pub_modbus.publish(modbus_msg);
	}

	if(modbus_mess.reg_addr >= MODBUS_INPUT_REG_ADDRESS_OFFSET)
	{ 
		server->set_recive_mapping(MODBUS_INPUT_REG_ADDRESS_MAP_TYPE(modbus_mess.reg_addr));
	}
	else
	{
		server->set_recive_mapping(MODBUS_REG_ADDRESS_MAP_TYPE(modbus_mess.reg_addr));
	}
}

void *modbus_tcp_server(void *arg)
{
	ModbusTcpServer modbus_tcp_server;

	g_modbus_tcp_server = &modbus_tcp_server;

	modbus_tcp_server.setup(1502);

	// for common reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_COMMON_REG_INDEX, 0, 0, 0, 0,
			MODBUS_COMMON_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_COMMON_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for guawang reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_GUAWANG_REG_INDEX, 0, 0, 0, 0,
			MODBUS_GUAWANG_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_GUAWANG_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for guawang reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_DIZHUAN1_REG_INDEX, 0, 0, 0, 0,
			MODBUS_DIZHUAN1_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_DIZHUAN1_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for paint reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_PAINT_REG_INDEX, 0, 0, 0, 0,
			MODBUS_PAINT_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_PAINT_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for wallhandling reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_WALLHANDING_REG_INDEX, 0, 0, 0, 0,
			MODBUS_WALLHANDING_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_WALLHANDING_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for wall reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_WALLINSTALL_REG_INDEX, 0, 0, 0, 0,
			MODBUS_WALLINSTALL_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_WALLINSTALL_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for wallpaper reg mapping
	modbus_tcp_server.add_new_mapping(MODBUS_WALLPAPER_REG_INDEX, 0, 0, 0, 0,
			MODBUS_WALLPAPER_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_WALLPAPER_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	// for ceiling polish
	modbus_tcp_server.add_new_mapping(MODBUS_CEILINGPOLISH_REG_INDEX, 0, 0, 0, 0,
			MODBUS_CEILINGPOLISH_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_CEILINGPOLISH_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);
	
	// for inside wallpolish
	modbus_tcp_server.add_new_mapping(MODBUS_INSIDE_WALLPOLISH_REG_INDEX, 0, 0, 0, 0,
			MODBUS_INSIDE_WALLPOLISH_REG_ADDRESS, MODBUS_REG_ADDRESS_FRAG_SIZ,
			MODBUS_INSIDE_WALLPOLISH_REG_ADDRESS + MODBUS_INPUT_REG_ADDRESS_OFFSET, MODBUS_REG_ADDRESS_FRAG_SIZ);

	modbus_tcp_server.register_recive_callback(on_tcp_recived);

	/*
	// 测试代码
	int16_t d[20];

	for(int i=0; i<20; ++i)
	{
	d[i] = -(1000 + i);
	}

	g_modbus_tcp_server->fill_read_reg_memory(
	MODBUS_REG_ADDRESS_MAP_TYPE(MODBUS_COMMON_REG_ADDRESS),
	0,
	20,
	(uint16_t *)&d[0]);
	// 测试代码 end
	*/

	modbus_tcp_server.start_server();
}

void handle_ros_to_modbus(const bzl_modbus::Modbus::ConstPtr& modbus)
{
	if(g_modbus_tcp_server == NULL)
	{
		return;
	}

	if(g_debug) {
		ROS_INFO_NAMED("ros_to_modbus", "cmd: 0x%02x, addr: 0x%02x, num: %d",
				modbus->cmd, modbus->addr, modbus->num);
		for(int i=0; i<modbus->data.size(); i++)
		{
			ROS_INFO_NAMED("ros_to_modbus", "[%d] = 0x%02x", i, modbus->data[i]);
		}
	}

	if(modbus->cmd == ROS_TO_MODBUS_CMD_READ)
	{
		if(!modbus->num)
		{
			return;
		}

		//ROS_INFO("recvd modbus read request!");
		uint16_t *data = (uint16_t *)malloc(modbus->num * sizeof(uint16_t));
		if(data == NULL) {
			ROS_WARN("[%s:%d] malloc failed", __func__, __LINE__);
			return;
		}

		memset(data, 0x0000, modbus->num * sizeof(uint16_t));

		g_modbus_tcp_server->get_write_reg_memory(
				MODBUS_REG_ADDRESS_MAP_TYPE(modbus->addr),
				MODBUS_REG_ADDRESS_INDEX(modbus->addr),
				modbus->num,
				data);

        bzl_modbus::Modbus modbus_msg;

		modbus_msg.cmd = ROS_TO_MODBUS_CMD_READ;
		modbus_msg.addr = modbus->addr;
		modbus_msg.num = modbus->num;
		modbus_msg.data.clear();
		for(int i=0; i<modbus->num; i++) {
			modbus_msg.data.push_back(data[i]);
		}
		pub_modbus.publish(modbus_msg);

		free(data);
	}
	else if(modbus->cmd == ROS_TO_MODBUS_CMD_WRITE)
	{
		int regs = modbus->num;
		if(modbus->num != modbus->data.size() && modbus->data.size() != 0)
		{
			regs = modbus->data.size();
		}
		else if(!modbus->num || !modbus->data.size())
		{
			return;
		}

		int bytes = regs * sizeof(uint16_t);
		// translate to big_endian
		if(g_cpu_endian == M_LITTLE_ENDIAN)
		{
            // fan add
            if(modbus->addr < 0x4000)
            {
                printf("g_modbus_tcp_server->fill_write_reg_memory\n");
                g_modbus_tcp_server->fill_write_reg_memory(
                        MODBUS_REG_ADDRESS_MAP_TYPE(modbus->addr),
                        MODBUS_REG_ADDRESS_INDEX(modbus->addr),
                        regs,
                        &modbus->data[0]);
                return;
            }
            // fan add end

			uint16_t *data = (uint16_t *)malloc(bytes);
			if(data == NULL) {
				ROS_WARN("[%s:%d] malloc failed", __func__, __LINE__);
				return;
			}

			memset(data, 0x0000, bytes);

			if(!modbus_adjust_bigendian(modbus->addr, bytes, &modbus->data[0], data))
			{
				g_modbus_tcp_server->fill_read_reg_memory(
						MODBUS_INPUT_REG_ADDRESS_MAP_TYPE(modbus->addr),
						MODBUS_REG_ADDRESS_INDEX(modbus->addr),
						regs,
						&data[0]);
			}

			free(data);
		}
		else
		{
            // fan add
            if(modbus->addr < 0x4000)
            {
                printf("g_modbus_tcp_server->fill_write_reg_memory\n");
                g_modbus_tcp_server->fill_write_reg_memory(
                        MODBUS_REG_ADDRESS_MAP_TYPE(modbus->addr),
                        MODBUS_REG_ADDRESS_INDEX(modbus->addr),
                        regs,
                        &modbus->data[0]);
                return;
            }
            // fan add end

			g_modbus_tcp_server->fill_read_reg_memory(
					MODBUS_INPUT_REG_ADDRESS_MAP_TYPE(modbus->addr),
					MODBUS_REG_ADDRESS_INDEX(modbus->addr),
					regs,
					&modbus->data[0]);


		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "modbus_server", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;
	ros::NodeHandle nh_local("~");

	nh_local.getParam("debug", g_debug);

	modbus_utils_init();

	g_cpu_endian = cpu_endian_check();

	pthread_create(&g_tid, NULL, modbus_tcp_server, NULL);

    pub_modbus = nh.advertise <bzl_modbus::Modbus> ("modbus_to_ros", 100);
    sub_modbus = nh.subscribe <bzl_modbus::Modbus> ("ros_to_modbus", 100, handle_ros_to_modbus);

	ros::spin();

#if 0
	ros::Rate loop_rate(g_callback_rate);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	pub_modbus.shutdown();
	sub_modbus.shutdown();

	pthread_join(g_tid, NULL);
	modbus_utils_deinit();
#endif

	return 0;
}
