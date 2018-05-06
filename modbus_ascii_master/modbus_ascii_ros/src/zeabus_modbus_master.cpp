    /*
 * zeabus_modbus_master.cpp
 *
 *  Created on: Jun 15, 2015
 *      Author: mahisorn
 */
#include <ros/ros.h>
#include <modbus_ascii_ros/DeviceDiscovered.h>
#include <modbus_ascii_ros/MBBitRegisterStatus.h>
#include <modbus_ascii_ros/MBWordRegisterStatus.h>
#include <modbus_ascii_ros/MBBitRegisterCommand.h>
#include <modbus_ascii_ros/MBWordRegisterCommand.h>

#include <mblib/MBDevice.h>
#include <termios.h>
#include <vector>

#include <boost/thread.hpp>

#define SCAN_DEVICE_START_ADDRESS 1
#define SCAN_DEVICE_END_ADDRESS 48
//#define SIM

extern int show_debug_info;

boost::mutex g_mutex;
ros::NodeHandle* nhPtr;
MBSlave* mb_slave_gpio;
int fd = 0;


typedef struct _register_bank
{
	bool coil[16];
	bool discrete_input[16];
	int holding_register[16];
	int input_register[16];
} RegisterBank;

RegisterBank register_bank[16];


std::vector<MBSlave> DiscoveredDeviceList;
std::vector<int> DiscoveredAddressList;
std::map<int,int> DiscoveredDeviceAddress;
std::map<int,int> DiscoveredDeviceDiscreteInputCount;
std::map<int,int> DiscoveredDeviceCoilCount;
std::map<int,int> DiscoveredDeviceInputRegisterCount;
std::map<int,int> DiscoveredDeviceHoldingRegisterCount;





bool MBDeviceScan (modbus_ascii_ros::DeviceDiscovered::Request  &req,
                     modbus_ascii_ros::DeviceDiscovered::Response &res)
{
#ifdef SIM
  res.address.resize(5);
  res.name.resize(5);
  res.reg_count_bank_1.resize(5);
  res.reg_count_bank_2.resize(5);
  res.reg_count_bank_3.resize(5);
  res.reg_count_bank_4.resize(5);
  res.register_name.resize(5);

  int i, j;
  for (i=0;i<5;i++)
  {
	  res.register_name[i].discrete_input_name.resize(16);
	  res.register_name[i].coil_name.resize(16);
	  res.register_name[i].input_register_name.resize(16);
	  res.register_name[i].holding_register_name.resize(16);

  }
  std::string name, temp;
  for (i=0;i<5;i++)
  {
	  name = "deviceName" + boost::to_string(i);
	  res.address[i] = i;
	  nhPtr->param<std::string>(name, res.name[i], "<NoName>");
	  res.reg_count_bank_1[i] = 16;
	  res.reg_count_bank_2[i] = 16;
	  res.reg_count_bank_3[i] = 16;
	  res.reg_count_bank_4[i] = 16;
	  for (j=0;j<16;j++)
	  {
		  name = "address"+boost::to_string(i)+"DiscreteInputName"+boost::to_string(j);
		  nhPtr->param<std::string>(name, res.register_name[i].discrete_input_name[j] , "<NoName>");
		  name = "address"+boost::to_string(i)+"CoilName"+boost::to_string(j);
		  nhPtr->param<std::string>(name, res.register_name[i].coil_name[j] , "<NoName>");
		  name = "address"+boost::to_string(i)+"InputRegisterName"+boost::to_string(j);
		  nhPtr->param<std::string>(name, res.register_name[i].input_register_name[j] , "<NoName>");
		  name = "address"+boost::to_string(i)+"HoldingRegisterName"+boost::to_string(j);
		  nhPtr->param<std::string>(name, res.register_name[i].holding_register_name[j] , "<NoName>");
		  //res.register_name[i].discrete_input_name.push_back("XX");

		  //ROS_ERROR("kk%s\n", typeid(res.register_name[i]).name());
	  }
  }
  return true;

#else
  	int address_to_test = 0;
  	DiscoveredDeviceList.clear();
  	DiscoveredAddressList.clear();
  	DiscoveredDeviceDiscreteInputCount.clear();
  	DiscoveredDeviceCoilCount.clear();
  	DiscoveredDeviceInputRegisterCount.clear();
  	DiscoveredDeviceHoldingRegisterCount.clear();
  	DiscoveredDeviceAddress.clear();
  	bool is_device_connected = false;
  	bool is_register_exist = false;
  	int discovered_count = 0;
	for (address_to_test=SCAN_DEVICE_START_ADDRESS;address_to_test<=SCAN_DEVICE_END_ADDRESS;address_to_test++)
	{
	  MBSlave DUT(address_to_test);
	  DUT.Init(fd);
	  {
		  boost::mutex::scoped_lock(g_mutex);
		  is_device_connected = DUT.ConnectionTest();
	  }
	  if (is_device_connected)
	  {
		  ROS_INFO("Device address 0x%02X (%d) Found", address_to_test, address_to_test);
	  	  DiscoveredDeviceList.push_back(DUT);
	  	  DiscoveredAddressList.push_back(address_to_test);
	  	  DiscoveredDeviceAddress[address_to_test] = discovered_count;
	  	  discovered_count++;
	  }
	  else{
		  //ROS_INFO("Device address 0x%02X (%d) Not found", address_to_test, address_to_test);
	  }
	}
	ROS_INFO("Finished Scanning");
	std::string name;
	int i, p;
	int register_count = 0;
	int register_address = 0;
	int device_count = DiscoveredDeviceList.size();
	res.name.resize(device_count);
	res.address.resize(device_count);
	res.reg_count_bank_1.resize(device_count);
	res.reg_count_bank_2.resize(device_count);
	res.reg_count_bank_3.resize(device_count);
	res.reg_count_bank_4.resize(device_count);
	res.register_name.resize(device_count);
	uint8_t bit_stream_dummy[1];
	uint16_t word_stream_dummy[1];
	for (i=0;i<device_count;i++)
	{
		name = "deviceName" + boost::to_string(DiscoveredAddressList[i]);
		res.address[i] = DiscoveredAddressList[i];
		nhPtr->param<std::string>(name, res.name[i], "<NoName>");
		register_count = 0;
		register_address = 0;
		while (ros::ok)
		{
			{
				boost::mutex::scoped_lock(g_mutex);
				is_register_exist = DiscoveredDeviceList[i].Read_discrete_inputs(register_address, 1, bit_stream_dummy);
			}
			if (!is_register_exist) break;
			register_count++;
			register_address++;
		}
		res.reg_count_bank_1[i] = register_count;
		res.register_name[i].discrete_input_name.resize(register_count);
		DiscoveredDeviceDiscreteInputCount.insert(std::pair<int,int>(DiscoveredAddressList[i], register_count));
		for (p=0;p<register_count;p++)
		{
		  name = "address"+boost::to_string(DiscoveredAddressList[i])+"DiscreteInputName"+boost::to_string(p);
		  nhPtr->param<std::string>(name, res.register_name[i].discrete_input_name[p] , "<NoName>");
		}


		register_count = 0;
		register_address = 0;
		while (ros::ok)
	  	{
			{
				boost::mutex::scoped_lock(g_mutex);
				is_register_exist = DiscoveredDeviceList[i].Read_coils(register_address, 1, bit_stream_dummy);
			}
			if (!is_register_exist) break;
	  		register_count++;
	  		register_address++;
	  	}
		res.reg_count_bank_2[i] = register_count;
		res.register_name[i].coil_name.resize(register_count);
		DiscoveredDeviceCoilCount[DiscoveredAddressList[i]] = register_count;
		ROS_INFO("Address %d order %d CoilCount=%d", DiscoveredAddressList[i], i, DiscoveredDeviceCoilCount[DiscoveredAddressList[i]]);
		for (p=0;p<register_count;p++)
			{
			name = "address"+boost::to_string(DiscoveredAddressList[i])+"CoilName"+boost::to_string(p);
			nhPtr->param<std::string>(name, res.register_name[i].coil_name[p] , "<NoName>");
		}


		register_count = 0;
		register_address = 0;
		while (ros::ok)
		{
			{
				boost::mutex::scoped_lock(g_mutex);
				is_register_exist =  DiscoveredDeviceList[i].Read_input_registers(register_address, 1, word_stream_dummy);
			}
			if (!is_register_exist) break;
			register_count++;
			register_address++;
		}
		res.reg_count_bank_3[i] = register_count;
		res.register_name[i].input_register_name.resize(register_count);
		DiscoveredDeviceInputRegisterCount.insert(std::pair<int,int>(DiscoveredAddressList[i], register_count));
		for (p=0;p<register_count;p++)
			{
			name = "address"+boost::to_string(DiscoveredAddressList[i])+"InputRegisterName"+boost::to_string(p);
			nhPtr->param<std::string>(name, res.register_name[i].input_register_name[p] , "<NoName>");
		}


		register_count = 0;
		register_address = 0;
		while (ros::ok)
		{
			{
				boost::mutex::scoped_lock(g_mutex);
				is_register_exist = DiscoveredDeviceList[i].Read_holding_registers(register_address, 1, word_stream_dummy);
			}
			if (!is_register_exist) break;
			register_count++;
			register_address++;
		}
		res.reg_count_bank_4[i] = register_count;
		res.register_name[i].holding_register_name.resize(register_count);
		DiscoveredDeviceHoldingRegisterCount.insert(std::pair<int,int>(DiscoveredAddressList[i], register_count));
		for (p=0;p<register_count;p++)
			{
			name = "address"+boost::to_string(DiscoveredAddressList[i])+"HoldingRegisterName"+boost::to_string(p);
			nhPtr->param<std::string>(name, res.register_name[i].holding_register_name[p] , "<NoName>");
		}
	}

	//Register Count
	return true;


#endif

}

bool MBReadAllCoil(modbus_ascii_ros::MBBitRegisterStatus::Request  &req,
        modbus_ascii_ros::MBBitRegisterStatus::Response &res)
{
#ifdef SIM
	res.status.resize(16);
	int i;
	for (i=0;i<16;i++)
	{
		res.status[i] = register_bank[req.device_address].coil[i];
	}
	return true;
#else
	int register_count = DiscoveredDeviceCoilCount[req.device_address];
	res.status.resize(register_count);
	int i, is_success;
	uint8_t bit_stream[(register_count+7)/8];
	{
		boost::mutex::scoped_lock(g_mutex);
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Read_coils(0, (uint16_t)register_count, bit_stream);
	}
	ROS_INFO("Read coils - Requested Address = %d count = %d", req.device_address, register_count);
	if (!is_success) return false;
	for (i=0;i<register_count;i++)
	{
		res.status[i] = (bit_stream[i/8] & (1<<(i%8)))?true:false;
	}
	return true;
#endif
}

bool MBReadAllDiscreteInput(modbus_ascii_ros::MBBitRegisterStatus::Request  &req,
        modbus_ascii_ros::MBBitRegisterStatus::Response &res)
{
#ifdef SIM
	res.status.resize(16);
	int i;
	for (i=0;i<16;i++)
	{
		res.status[i] = register_bank[req.device_address].discrete_input[i];

	}
	return true;
#else
	int register_count = DiscoveredDeviceDiscreteInputCount[req.device_address];
	res.status.resize(register_count);
	int i, is_success;
	uint8_t bit_stream[(register_count+7)/8];
	{
		boost::mutex::scoped_lock(g_mutex);
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Read_discrete_inputs(0, (uint16_t)register_count, bit_stream);
	}
	ROS_INFO("Read discrete inputs - Requested Address = %d count = %d", req.device_address, register_count);
	if (!is_success) return false;
	for (i=0;i<register_count;i++)
	{
		res.status[i] = (bit_stream[i/8] & (1<<(i%8)))?true:false;
	}
	return true;

#endif
}

bool MBReadAllHoldingRegister(modbus_ascii_ros::MBWordRegisterStatus::Request  &req,
        modbus_ascii_ros::MBWordRegisterStatus::Response &res)
{
#ifdef SIM
	res.status.resize(16);
	int i;
	for (i=0;i<16;i++)
	{
		res.status[i] = register_bank[req.device_address].holding_register[i];
	}
	return true;
#else
	int register_count = DiscoveredDeviceHoldingRegisterCount[req.device_address];
	res.status.resize(register_count);
	int i, is_success;
	uint16_t word_stream[register_count];
	{
		boost::mutex::scoped_lock(g_mutex);
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Read_holding_registers(0, (uint16_t)register_count, word_stream);
	}
	ROS_INFO("Read Holding registers - Requested Address = %d count = %d", req.device_address, register_count);
	if (!is_success) return false;
	for (i=0;i<register_count;i++)
	{
		res.status[i] = word_stream[i];
	}
	return true;
#endif
}

bool MBReadAllInputRegister(modbus_ascii_ros::MBWordRegisterStatus::Request  &req,
        modbus_ascii_ros::MBWordRegisterStatus::Response &res)
{
#ifdef SIM
	res.status.resize(16);
	int i;
	for (i=0;i<16;i++)
	{
		res.status[i] = register_bank[req.device_address].input_register[i];
	}
	return true;
#else
	int register_count = DiscoveredDeviceInputRegisterCount[req.device_address];
	res.status.resize(register_count);
	int i, is_success;
	uint16_t word_stream[register_count];
	{
		boost::mutex::scoped_lock(g_mutex);
		//show_debug_info = 1;
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Read_input_registers(0, (uint16_t)register_count, word_stream);
		//show_debug_info = 0;
	}
	ROS_INFO("Read Input registers - Requested Address = %d count = %d", req.device_address, register_count);
	if (!is_success) return false;
	for (i=0;i<register_count;i++)
	{
		res.status[i] = word_stream[i];
	}
	return true;
#endif
}


bool MBWriteCoil(modbus_ascii_ros::MBBitRegisterCommand::Request  &req,
        modbus_ascii_ros::MBBitRegisterCommand::Response &res)
{
#ifdef SIM
	int i;
	for (i=req.register_start_address;i<req.register_start_address + req.register_quantity;i++)
	{
		register_bank[req.device_address].coil[i] = req.status[i - req.register_start_address];
	}
	return true;
#else
	int register_count = req.register_quantity;
	int i, is_success;
	uint8_t bit_stream[(register_count+7)/8];
	for (i=0;i<register_count;i++)
	{
		if (i%8 == 0) bit_stream[i/8] = 0;
		bit_stream[i/8] |= (req.status[i]?( 1<<(i%8) ):0);
	}
	{
		boost::mutex::scoped_lock(g_mutex);
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Write_multiple_coils((uint16_t)req.register_start_address, (uint16_t)req.register_quantity, bit_stream);
	}
	ROS_INFO("Write Coils - Dev Address = %d Start = %d count = %d", req.device_address, req.register_start_address ,register_count);
	if (!is_success) return false;
	return true;
#endif
}

bool MBWriteHoldingRegister(modbus_ascii_ros::MBWordRegisterCommand::Request  &req,
        modbus_ascii_ros::MBWordRegisterCommand::Response &res)
{
#ifdef SIM
	int i;
	for (i=req.register_start_address;i<req.register_start_address+req.register_quantity;i++)
	{
		register_bank[req.device_address].holding_register[i] = req.status[i - req.register_start_address];
	}
	return true;
#else
	int register_count = req.register_quantity;
	int i, is_success;
	uint16_t word_stream[register_count];
	for (i=0;i<register_count;i++)
	{
		word_stream[i] = req.status[i];
	}
	{
		boost::mutex::scoped_lock(g_mutex);
		is_success = DiscoveredDeviceList[DiscoveredDeviceAddress[req.device_address]].Write_multiple_registers((uint16_t)req.register_start_address, (uint16_t)req.register_quantity, word_stream);
	}
	if (!is_success) return false;
	return true;
#endif
}


int main(int argc, char **argv)
{
  fd = 0;
  show_debug_info = 1;
  ros::init(argc, argv, "zeabus_modbus_master");
  ros::NodeHandle nh("~");
  nhPtr = &nh;

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  	  ros::console::notifyLoggerLevelsChanged();
  }
  std::string device;
  int baudrate;
  show_debug_info = 0;
  nh.param<std::string>("device", device, "/dev/ttyUSB9");





#ifdef SIM

#else
  if(Open_port(&fd, device.c_str(), B115200, 7, 0, 2, 0) == 0)
  {
    ROS_FATAL("Cannot open: %s", device.c_str());    
    return -1;
  }
	

#endif

  ros::ServiceServer service_mb_device_scan = nh.advertiseService("MBDeviceScan", MBDeviceScan);
  ros::ServiceServer service_mb_read_all_coil = nh.advertiseService("MBReadAllCoil", MBReadAllCoil);
  ros::ServiceServer service_mb_read_all_discrete_input = nh.advertiseService("MBReadAllDiscreteInput", MBReadAllDiscreteInput);
  ros::ServiceServer service_mb_read_all_holding_register = nh.advertiseService("MBReadAllHoldingRegister", MBReadAllHoldingRegister);
  ros::ServiceServer service_mb_read_all_input_register = nh.advertiseService("MBReadAllInputRegister", MBReadAllInputRegister);
  ros::ServiceServer service_mb_write_coil = nh.advertiseService("MBWriteCoil", MBWriteCoil);
  ros::ServiceServer service_mb_write_holding_register = nh.advertiseService("MBWriteHoldingRegister", MBWriteHoldingRegister);  int baro_and_switch_rate = 100;

  ros::Rate rate(100);
  while(ros::ok())
  { 
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

