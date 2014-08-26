#include <ros/ros.h>
#include <oscpack/ip/UdpSocket.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>

#define OUTPUT_BUFFER_SIZE 64
int port = 7800;
char buffer[OUTPUT_BUFFER_SIZE];
UdpTransmitSocket* transmit_socket;


void callback(std_msgs::Float64MultiArray msg){
    int pitch = (int)msg.data[0];
    float duration = (float)msg.data[1];
    int velocity = (int)msg.data[2];
    
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/erohcispret" ) 
            << pitch << duration << velocity << osc::EndMessage
        << osc::EndBundle;
        
    transmit_socket->Send( p.Data(), p.Size() );
}

int main(int argc, char* argv[]){
	std::string address;
    if(argc < 2){
        address = "localhost";
    }else{
    	std::string line = argv[1];
    	
    	int colon_pos = line.find(":");
    	if(colon_pos != std::string::npos){
    		address = line.substr(0, colon_pos);
    		port = std::stoi(line.substr(colon_pos + 1, std::string::npos));
    	}else{
    		address = line;
    	}
    }
    std::cout << address << std::endl;
  	std::cout << port << std::endl;
    
    transmit_socket = new UdpTransmitSocket( IpEndpointName( address.c_str() , port ) );
    ros::init(argc, argv, "osc_relay");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("to_ableton", 100, callback);
    ros::Rate loop_rate(200);
    
    
    std_msgs::Float64MultiArray test;
    std::vector<double> stuff;
    stuff.push_back(49.0);
    stuff.push_back(0.5);
    stuff.push_back(100.0);
    test.data = stuff;
    callback(test);
    
    
    ros::spin();
    
    delete transmit_socket;
	return 0;
}