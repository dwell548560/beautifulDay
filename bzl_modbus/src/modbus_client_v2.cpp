#include <ros/ros.h>

#include <modbus/modbus.h>
#include "bzl_modbus/Modbus.h"
#include "bzl_modbus/ModbusStatus.h"
#include "bzl_modbus/ModbusStatusV2.h"
#include <queue>
#define MAX_FARAM_SIZE 100

enum AGVModbusRosFuncCode {
    ROS_TO_MODBUS_CMD_WRITE = 0x05,
    ROS_TO_MODBUS_CMD_READ = 0x06,
    ROS_MODBUS_CMD_STATUS = 0x07
};


//global variables
static std::string g_connect_name = "PLC";
static int g_interval = 100 * 1000; //100ms

static ros::Publisher g_pub_modbus;
static ros::Subscriber g_sub_modbus;
static ros::Publisher g_pub_modbus_status;
static ros::Publisher g_pub_modbus_status_v2;

static modbus_t *g_modbus_ctx = NULL;
static pthread_mutex_t g_mutex;
static pthread_mutex_t g_mutex2;

static bool g_debug = false;
static bool g_sync = false;
static bool g_terminal = false;
static bool g_connected = false;
static bool g_read_hold_reg = false;

std::vector<bzl_modbus::Modbus> write_vector;

void re_connect()
{
    modbus_close(g_modbus_ctx);
    while(modbus_connect(g_modbus_ctx) == -1)
    {
        modbus_close(g_modbus_ctx);

        if(g_connected)
        {
            g_connected = false;
            ROS_INFO("%s connect lost, reconnecting...", g_connect_name.c_str());
        }
        sleep(1);
    }

    if(!g_connected)
    {
        g_connected = true;
        ROS_INFO("%s connected.", g_connect_name.c_str());
    }
}

class RegInfo2
{
private:
    uint16_t m_hold_addr;
    uint16_t m_hold_cnts;
    uint16_t m_input_addr;
    uint16_t m_input_cnts;

    uint16_t *m_read_hold_data;
    uint16_t *m_write_hold_data;
    uint16_t *m_read_input_data;

public:
    RegInfo2(uint16_t h_addr, uint16_t h_cnts, uint16_t i_addr, uint16_t i_cnts)
        :m_hold_addr(h_addr), m_hold_cnts(h_cnts), m_input_addr(i_addr), m_input_cnts(i_cnts)
    {
        m_read_hold_data = new uint16_t[m_hold_cnts];
        memset(m_read_hold_data, 0x0, sizeof(uint16_t) * m_hold_cnts);

        m_write_hold_data = new uint16_t[m_hold_cnts];
        memset(m_write_hold_data, 0x0, sizeof(uint16_t) * m_hold_cnts);

        m_read_input_data = new uint16_t[m_input_cnts];
        memset(m_read_input_data, 0x0, sizeof(uint16_t) * m_input_cnts);

        ROS_INFO("********************************");
        ROS_INFO("***** m_hold_addr : 0x%04x *****", m_hold_addr);
        ROS_INFO("***** m_hold_cnts : %6d *****", m_hold_cnts);
        ROS_INFO("***** m_input_addr: 0x%04x *****", m_input_addr);
        ROS_INFO("***** m_input_cnts: %6d *****", m_input_cnts);
        ROS_INFO("********************************");
    }

    uint16_t* get_read_mem_addr(uint16_t addr, uint16_t num)
    {
        uint16_t* mem_addr = NULL;

        if((addr >= m_input_addr) && ((addr + num) <= (m_input_addr + m_input_cnts)))
        {
            mem_addr =  addr - m_input_addr + m_read_input_data;
        }
        else if((addr >= m_hold_addr) && ((addr + num) <= (m_hold_addr + m_hold_cnts)))
        {
            mem_addr = addr - m_hold_addr + m_read_hold_data;
        }

        return mem_addr;
    }

    uint16_t* get_write_mem_addr(uint16_t addr, uint16_t num)
    {
        uint16_t* mem_addr = NULL;

        if((addr >= m_hold_addr) && ((addr + num) <= (m_hold_addr + m_hold_cnts)))
        {
            mem_addr = addr - m_hold_addr + m_write_hold_data;
        }

        return mem_addr;
    }

    ~RegInfo2()
    {
        delete[] m_read_hold_data;
        delete[] m_write_hold_data;
        delete[] m_read_input_data;
    }

    void deal_write_hold_reg()
    {
        if(g_sync)
        {
            if(memcmp(m_read_hold_data, m_write_hold_data, sizeof(uint16_t)*m_hold_cnts) != 0)
            {
                for(int i=0; i<m_hold_cnts; ++i)
                {
                    if(m_read_hold_data[i] != m_write_hold_data[i])
                    {
                        pthread_mutex_lock(&g_mutex);
                        if(modbus_write_registers(g_modbus_ctx, m_hold_addr + i, 1, m_write_hold_data + i) == -1)
                        {
                            ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, data: 0x%02x failed",
                                    g_connect_name.c_str(), m_hold_addr + i, m_write_hold_data[i]);
                            re_connect();
                        }
                        else
                        {
                            ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, data: 0x%02x scuessed",
                                    g_connect_name.c_str(), m_hold_addr + i, m_write_hold_data[i]);
                        }
                        pthread_mutex_unlock(&g_mutex);
                        usleep(g_interval);
                    }
                }
            }
        }
        else
        {
            while(write_vector.size() > 0)
            {
                pthread_mutex_lock(&g_mutex2);
                bzl_modbus::Modbus info = write_vector.front();
                write_vector.erase(write_vector.begin());
                pthread_mutex_unlock(&g_mutex2);

                static int MAX_WRITE_NUM = 96;

                int left_reg_num = info.data.size(); //剩余要写的寄存器数量

                uint16_t write_reg_addr = info.addr;
                uint16_t* write_reg_data = info.data.data();

                while(left_reg_num > MAX_WRITE_NUM)
                {
                    pthread_mutex_lock(&g_mutex);
                    if(modbus_write_registers(g_modbus_ctx, write_reg_addr, MAX_WRITE_NUM, write_reg_data) == -1)
                    {
                        ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, num: %02d failed",
                                g_connect_name.c_str(), write_reg_addr, MAX_WRITE_NUM);
                        re_connect();
                    }
                    else
                    {
                        ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, num: %02d scuessed",
                                g_connect_name.c_str(), write_reg_addr, MAX_WRITE_NUM);
                    }
                    pthread_mutex_unlock(&g_mutex);

                    left_reg_num  -= MAX_WRITE_NUM;
                    write_reg_addr += MAX_WRITE_NUM;
                    write_reg_data += MAX_WRITE_NUM;
                }

                if(left_reg_num > 0)
                {
                    pthread_mutex_lock(&g_mutex);
                    if(modbus_write_registers(g_modbus_ctx, write_reg_addr, left_reg_num, write_reg_data) == -1)
                    {
                        ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, num: %02d failed",
                                g_connect_name.c_str(), write_reg_addr, left_reg_num);
                        re_connect();
                    }
                    else
                    {
                        ROS_INFO_NAMED("ros_to_modbus", "%s modbus_write_registers: addr: 0x%02x, num: %02d scuessed",
                                g_connect_name.c_str(), write_reg_addr, left_reg_num);


                    }
                    pthread_mutex_unlock(&g_mutex);
                }
            }
        }
    }

    void deal_read_hold_reg()
    {
        uint16_t times = m_hold_cnts / MAX_FARAM_SIZE + 1;

        for(uint16_t i=0; i<times; ++i)
        {
            uint16_t read_num = m_hold_cnts - i*MAX_FARAM_SIZE;
            if(read_num > MAX_FARAM_SIZE)
            {
                read_num = MAX_FARAM_SIZE;
            }
            else if(read_num <= 0)
            {
                break;
            }

            uint16_t read_addr = m_hold_addr + i*MAX_FARAM_SIZE;
            uint16_t* read_fill_data = m_read_hold_data + i*MAX_FARAM_SIZE;

            pthread_mutex_lock(&g_mutex);
            if(modbus_read_registers(g_modbus_ctx, read_addr, read_num, read_fill_data) == -1)
            {
                ROS_INFO("%s modbus_read_registers: addr:0x%02x, num:%d failed", g_connect_name.c_str(), read_addr, read_num);
                re_connect();
            }

            pthread_mutex_unlock(&g_mutex);
            usleep(g_interval);
        }
    }

    void deal_read_input_reg()
    {
        uint16_t times = m_input_cnts / MAX_FARAM_SIZE + 1;

        for(uint16_t i=0; i<times; ++i)
        {
            uint16_t read_num = m_input_cnts - i*MAX_FARAM_SIZE;
            if(read_num > MAX_FARAM_SIZE)
            {
                read_num = MAX_FARAM_SIZE;
            }
            else if(read_num <= 0)
            {
                break;
            }

            uint16_t read_addr = m_input_addr + i*MAX_FARAM_SIZE;
            uint16_t* read_fill_data = m_read_input_data + i*MAX_FARAM_SIZE;

            pthread_mutex_lock(&g_mutex);
            if(modbus_read_input_registers(g_modbus_ctx, read_addr, read_num, read_fill_data) == -1)
            {
                ROS_INFO("%s modbus_read_registers: addr:0x%02x, num:%d failed", g_connect_name.c_str(), read_addr, read_num);
                re_connect();
            }

            pthread_mutex_unlock(&g_mutex);
            usleep(g_interval);
        }
    }
};

static RegInfo2 *g_reginfo;

void *modbus_client(void *arg)
{
    while(!g_terminal && ros::ok())
    {
        static uint32_t try_cnts = 0;
        if(modbus_connect(g_modbus_ctx) != -1)
        {
            g_connected = true;
            ROS_INFO("connect to modbus_server %s succed!", g_connect_name.c_str());
            break;
        }
        else
        {
            g_connected = false;
            ROS_WARN("connect to modbus_server %s timeout, retry %d times", g_connect_name.c_str(), ++try_cnts);
        }
        sleep(1);
    }

    while(!g_terminal)
    {
        g_reginfo->deal_read_input_reg();

        if(g_read_hold_reg)
        {
            g_reginfo->deal_read_hold_reg();
        }

        g_reginfo->deal_write_hold_reg();

        usleep(g_interval);
    }
}

void handle_ros_to_modbus(const bzl_modbus::Modbus::ConstPtr& modbus)
{
    if(g_debug)
    {
        ROS_INFO_NAMED("ros_to_modbus", "cmd: 0x%02x, addr: 0x%02x, num: %d", modbus->cmd, modbus->addr, modbus->num);
        for(int i=0; i<modbus->data.size(); i++)
        {
            ROS_INFO_NAMED("ros_to_modbus", "[%d] = 0x%02x", i, modbus->data[i]);
        }
    }

    switch(modbus->cmd)
    {
        case ROS_TO_MODBUS_CMD_WRITE:
            {
                uint16_t write_num = modbus->data.size();
                if(write_num == 0)
                {
                    break;
                }
                uint16_t write_reg_addr = modbus->addr;
                uint16_t * write_data = (uint16_t *)&modbus->data[0];

                uint16_t * write_mem_addr = g_reginfo->get_write_mem_addr(write_reg_addr, write_num);
                uint16_t * read_mem_addr = g_reginfo->get_read_mem_addr(write_reg_addr, write_num);

                if(write_mem_addr == NULL)
                {
                    break;
                }

                if(g_sync)
                {
                    memcpy(write_mem_addr, write_data, write_num * sizeof(uint16_t));
                }
                else if(!g_read_hold_reg || (g_read_hold_reg && memcmp(read_mem_addr, write_data, write_num* sizeof(uint16_t)) != 0))
                {
                    pthread_mutex_lock(&g_mutex2);
                    for(std::vector<bzl_modbus::Modbus>::iterator it = write_vector.begin(); it != write_vector.end(); ++it)
                    {
                        if((((*it).addr == (*modbus).addr)) && ((*it).num == (*modbus).num))
                        {
                            if(memcmp(&(*it).data, &(*modbus).data, (*it).num) == 0)
                            {
                                pthread_mutex_unlock(&g_mutex2);
                                return;
                            }
                        }
                    }
                    write_vector.push_back(*modbus);
                    pthread_mutex_unlock(&g_mutex2);
                }
            }
            break;
        case ROS_TO_MODBUS_CMD_READ:
            {
                uint16_t read_num = modbus->num/2;
                if(read_num == 0)
                {
                    break;
                }
                uint16_t read_reg_addr = modbus->addr;
                uint16_t * read_mem_addr = g_reginfo->get_read_mem_addr(read_reg_addr, read_num);

                if(read_mem_addr == NULL)
                {
                    break;
                }

                bzl_modbus::Modbus modbus_msg;

                modbus_msg.cmd = ROS_TO_MODBUS_CMD_READ;
                modbus_msg.addr = modbus->addr;
                modbus_msg.num = modbus->num;
                modbus_msg.data.clear();

                for(int i=0; i<read_num; i++)
                {
                    modbus_msg.data.push_back(read_mem_addr[i]);
                }

                g_pub_modbus.publish(modbus_msg);
            }
            break;
    }
}

void regs_init(void)
{
    bool b_getParamScuess = true;

    ros::NodeHandle nh_local("~");
    int hold_reg_start, hold_reg_num, input_reg_start, input_reg_num;

    b_getParamScuess &= nh_local.getParam("hold_reg_start", hold_reg_start);
    b_getParamScuess &= nh_local.getParam("hold_reg_num", hold_reg_num);
    b_getParamScuess &= nh_local.getParam("input_reg_start", input_reg_start);
    b_getParamScuess &= nh_local.getParam("input_reg_num", input_reg_num);

    if(!b_getParamScuess)
    {
        ROS_INFO("get param failed. use test param.");
        hold_reg_start = 0;
        hold_reg_num = 256;
        input_reg_start = 256;
        input_reg_num = 256;
    }

    g_reginfo = new RegInfo2(hold_reg_start, hold_reg_num, input_reg_start, input_reg_num);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "modbus_client_v2", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    std::string param_backend = "TCP";
    int param_slave_id = 1;

    nh_local.getParam("connect_name", g_connect_name);
    nh_local.getParam("backend", param_backend);
    nh_local.getParam("slave_id", param_slave_id);
    nh_local.getParam("debug", g_debug);
    nh_local.getParam("sync", g_sync);
    nh_local.getParam("update_interval", g_interval);

    ROS_INFO("backend: %s", param_backend.c_str());
    ROS_INFO("slave_id: %d", param_slave_id);
    ROS_INFO("debug: %s", g_debug ? "ON" : "OFF");
    ROS_INFO("sync: %s", g_sync ? "ON" : "OFF");
    ROS_INFO("update_interval: %dus", g_interval);

    if(strcasecmp(param_backend.c_str(), "RTU") == 0)
    {
        std::string param_rtu_port("/dev/ttyTHS1");
        int param_rtu_baudrate = 115200;

        nh_local.getParam("rtu_port", param_rtu_port);
        nh_local.getParam("rtu_baudrate", param_rtu_baudrate);
        g_modbus_ctx = modbus_new_rtu(param_rtu_port.c_str(), param_rtu_baudrate, 'N', 8, 1);

        ROS_INFO("rtu_port: %s", param_rtu_port.c_str());
        ROS_INFO("rtu_baudrate: %d", param_rtu_baudrate);
    }
    else
    {
        std::string param_tcp_ip = "192.168.1.109";
        int param_tcp_port = 1502;

        nh_local.getParam("server_ip", param_tcp_ip);
        nh_local.getParam("port", param_tcp_port);
        g_modbus_ctx = modbus_new_tcp(param_tcp_ip.c_str(), param_tcp_port);

        ROS_INFO("tcp_ip: %s", param_tcp_ip.c_str());
        ROS_INFO("tcp_port: %d", param_tcp_port);
    }
    regs_init();

    if(g_modbus_ctx == NULL)
    {
        ROS_WARN("modbus init failed");
        return -1;
    }

    modbus_set_debug(g_modbus_ctx, g_debug);
    modbus_set_error_recovery(g_modbus_ctx, (modbus_error_recovery_mode)(MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL));
    modbus_set_slave(g_modbus_ctx, param_slave_id);
    modbus_set_response_timeout(g_modbus_ctx, 0, 500*1000);

    pthread_mutex_init(&g_mutex, NULL);
    pthread_mutex_init(&g_mutex2, NULL);

    g_pub_modbus_status = nh.advertise <bzl_modbus::ModbusStatus> ("modbus_status", 100);
    g_pub_modbus_status_v2 = nh.advertise <bzl_modbus::ModbusStatusV2> ("modbus_status_v2", 100);
    g_pub_modbus = nh.advertise <bzl_modbus::Modbus> ("modbus_to_ros", 100);
    g_sub_modbus = nh.subscribe <bzl_modbus::Modbus> ("ros_to_modbus", 100, handle_ros_to_modbus);

    pthread_t thread_id = -1;
    pthread_create(&thread_id, NULL, modbus_client, NULL);


    bzl_modbus::ModbusStatus st;
    bzl_modbus::ModbusStatusV2 st_v2;
    st_v2.connect_name = g_connect_name;

    int ticks = 0;
    bool last_connected = false;
    ros::Rate rate(100);
    while(ros::ok())
    {
        if(((ticks++ % 200) == 0) || (last_connected != g_connected))
        {
            st.status = g_connected ? 0 : 0xFFFF;
            st_v2.is_connected = g_connected;
            g_pub_modbus_status.publish(st);
            g_pub_modbus_status_v2.publish(st_v2);

            last_connected = g_connected;
        }


        rate.sleep();
        ros::spinOnce();
    }

    g_terminal = true;
    pthread_join(thread_id, NULL);

    return 0;
}
