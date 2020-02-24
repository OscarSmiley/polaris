#include <ros/ros.h>

#include <string>
#include <memory>
#include <serial/serial.h>

#include "monitor/GetSerialDevice.h"
#include "peripherals/powerboard.h"
#include "peripherals/power_enable.h"
#include "peripherals/avg_data.h"
#include "serial_protocol.hpp"

#define PSI_TO_PASCAL_MUL (68.94757)
#define RETRY_COUNT (3)

using rosserv = ros::ServiceServer;
using powerboardInfo = peripherals::powerboard;
using PowerEnableReq = peripherals::power_enable::Request;
using PowerEnableRes = peripherals::power_enable::Response;
using AvgDataReq = peripherals::avg_data::Request;
using AvgDataRes = peripherals::avg_data::Response;


class power_board
{
public:
    power_board(const std::string & port, int baud_rate = 115200, int timeout = 1000);
    ~power_board();
    bool average_ext_pressure(AvgDataReq &req, AvgDataRes &res);
    bool populate_topic(powerboardInfo &msg);
private:
    std::unique_ptr<serial_protocol> serial_handle = nullptr;
};

power_board::power_board(const std::string &port, int baud_rate, int timeout)
{
    ROS_INFO("Connecting to power_board on port: %s", port.c_str());
    auto connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
    this->serial_handle = std::unique_ptr<serial_protocol>(new serial_protocol(std::move(connection)));
}

power_board::~power_board()
{

}

bool power_board::populate_topic(powerboardInfo &msg)
{
    bool ret = true;

    // Get Environment Data
    protocol_allMessages_U receivedMessage = {};
    if(serial_handle->request_data_message(PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_ENV_DATA, receivedMessage, protocol_MID_PB_envData))
    {
        msg.external_pressure = (double)receivedMessage.PB_envData.extPressure / 100U; // PSI
    }
    else
    {
        ret = false;
    }

    return ret;
}

bool power_board::average_ext_pressure(AvgDataReq &req, AvgDataRes &res)
{
    // Use rate to determine read speed
    ros::Rate r(req.acq_rate);

    // Try and acquire the appropriate amount of data
    int retry_count = 0;
    res.avg_data = 0;

    for (int i = 0; i < req.acq_count; i++)
    {
        // Read the external pressure, and add to average sum.
        protocol_allMessages_U response;
        if (this->serial_handle->request_data_message(PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_ENV_DATA, response, protocol_MID_PB_envData))
        {
            double external_pressure = (double)response.PB_envData.extPressure;
            res.avg_data += external_pressure / req.acq_count;
            r.sleep();
        }
        else if (retry_count < RETRY_COUNT) // Retry if read failed
        {
            ROS_ERROR("Failed to read pressure. Retry Count:%d", retry_count);
            i--;
            retry_count++;
        }
        else // If number of retries exceeded, fail the service call
        {
            ROS_ERROR("Failed to read pressure within retry count. Service request failed.");
            return false;
        }
    }

    // Convert pressure to Pascals
    res.avg_data *= PSI_TO_PASCAL_MUL;

    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "power_board");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    int loop_rate;
    nh.getParam("loop_rate", loop_rate);

    // Wait until serial device manager is ready
    ros::service::waitForService("/serial_manager/GetDevicePort", -1);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv))
    {
        ROS_INFO("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Power Board on fd %s\n", srv.response.device_fd.c_str());
    power_board device(srv.response.device_fd);

    ros::Publisher pub = nh.advertise<peripherals::powerboard>("power_board_data", 1);
    // ros::ServiceServer pwr_en = nh.advertiseService("PowerEnable", &power_board::power_enabler, &device);
    ros::ServiceServer avg_ext_p = nh.advertiseService("AverageExtPressure", &power_board::average_ext_pressure, &device);

    // Main loop
    ros::Rate r(loop_rate);
    while(ros::ok())
    {
        // Publish message to topic
        peripherals::powerboard msg;
        if(device.populate_topic(msg))
        {
            pub.publish(msg);
        }

        // End of loop maintenance
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
