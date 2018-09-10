#include "include/wifi_watchdog.h"

Pinger::Pinger(std::string host, std::string msg_name, ros::NodeHandle& node) {
    _host = host;
    _publisher = node.advertise<mg_msgs::PingStatus>(msg_name, 1);
}

Pinger::~Pinger(void) {

}

void Pinger::ping_loop() {
    while (true) {
        bool ok = ping(_host);

        mg_msgs::PingStatus msg;
        msg.alive = ok;
        msg.host = _host;
        msg.ping_ms = -1;

        _publisher.publish(msg);
    }

}

void Pinger::start(void) {
    boost::thread t(boost::bind(&Pinger::ping_loop, this) );
}

// Danger: this function blocks while running an external ping command
bool Pinger::ping (std::string host) {
    
    std::string command("timeout 1 ping -n -c 1 " + host);
    
    int result = boost::process::system(command, boost::process::std_out > boost::process::null);

    boost::this_thread::sleep_for(boost::chrono::milliseconds{100});

    return (result == 0);
}

mg_msgs::WifiStatus get_wifi_status() {

    std::ifstream file("/proc/net/wireless");
    std::string line;

    // Discard 2 header lines
    getline(file, line);
    getline(file, line);

    // Read the first interface description line
    getline(file, line);

    // split status line into words
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char>> tokens(line, sep);

    std::vector<std::string> words;

    for (const std::string& word : tokens) {
        words.push_back(word);
    }

    // make sure we got enough words
    if (words.size() != 11) {
        ROS_ERROR("Malformed /proc/net/wireless!");
        mg_msgs::WifiStatus ret;
        ret.interface = "<none>";
        return ret; 
    }

    // Parse words
    mg_msgs::WifiStatus ret;
    
    ret.interface = words[0];
    ret.status = words[1];
    ret.link_quality = std::stod(words[2]);
    ret.signal_level = std::stod(words[3]);
    ret.noise_level = std::stod(words[4]);
    ret.discarded_nwid = std::stod(words[5]);
    ret.discarded_crypt = std::stod(words[6]);
    ret.discarded_frag = std::stod(words[7]);
    ret.discarded_retry = std::stod(words[8]);
    ret.discarded_misc = std::stod(words[9]);
    ret.missed_beacon = std::stod(words[10]);

    return ret; 
}

int main(int argc, char** argv){

	ros::init(argc, argv, "wifi_watchdog");
    ros::NodeHandle node = ros::NodeHandle("~");
	ROS_INFO("wifi_watchdog node started!");

    auto pub = node.advertise<mg_msgs::WifiStatus>("wifi_status", 1);

	const double rate = 10.0;
	ros::Rate loop_rate(rate);

    std::string ground_ip;
    std::string refnet_ip;

    node.param<std::string>("ground_ip", ground_ip, "localhost");
    node.param<std::string>("refnet_ip", refnet_ip, "localhost");

    std::vector<std::string> ping_hosts;
    ping_hosts.push_back("4.2.2.1");
    ping_hosts.push_back(refnet_ip);
    ping_hosts.push_back(ground_ip);

    std::vector<std::string> ping_names;
    ping_names.push_back("ping_internet");
    ping_names.push_back("ping_refnet");
    ping_names.push_back("ping_ground");

    std::vector<Pinger*> pingers;

    for (int i = 0; i < ping_hosts.size(); i++) {
        Pinger* pingy_boi = new Pinger(ping_hosts[i], ping_names[i], node); 
        pingy_boi->start();
        pingers.push_back(pingy_boi);
    }

    ROS_INFO("Threads started");    

	while (ros::ok()) {
		ros::spinOnce();
        pub.publish(get_wifi_status());
		loop_rate.sleep();
	}
}
