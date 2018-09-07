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

    std::cout << "result: " << result << std::endl;

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
    
    ROS_INFO("loop");

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

    std::vector<std::string> ping_hosts;
    ping_hosts.push_back("8.8.8.8");

    Pinger pingy_boi("8.8.8.8", "googel", node);
    pingy_boi.start();

    std::cout << "Thread is started!!!!!!!!!!!!!!!!!!" << std::endl;

	while (ros::ok()) {
		ros::spinOnce();
        pub.publish(get_wifi_status());
		loop_rate.sleep();
	}
}
