#ifndef __WIFI_WATCHDOG_H
#define __WIFI_WATCHDOG_H

// Cpp libraries
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <boost/tokenizer.hpp>
#include <boost/process.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "mg_msgs/WifiStatus.h"
#include "mg_msgs/PingStatus.h"

class Pinger {
    public:
        Pinger(std::string host, std::string msg_name, ros::NodeHandle& node); 
        ~Pinger(void);
        void start();
    private:
        static bool ping(std::string host);
        void ping_loop();
        std::string _host;
        ros::Publisher _publisher;
};

#endif
