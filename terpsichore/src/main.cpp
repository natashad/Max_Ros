#include "LRReceiver.h"
#include <oscpack/ip/UdpSocket.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <thread>
#include <csignal>
#include <string>
#include <iostream>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <X11/Xlib.h>
 
#define PORT 8000
#define OUTPUT_BUFFER_SIZE 1024
char buffer[OUTPUT_BUFFER_SIZE];

bool playing = false;
bool quit = false;
	
std::string transport_string;
sf::Text transport_text;
std::string time_string;
sf::Text time_text;
sf::CircleShape play_shape(40, 3);
sf::RectangleShape stop_shape(sf::Vector2f(70, 70));

ros::Publisher pub;
std::string ADDRESS;
LRPacketListener* listener;
UdpListeningReceiveSocket* receive_socket;

ros::Time last_update_time;
ros::Time playback_start_time;
double sync_offset;
double t;

void publish(std::string s){
    std_msgs::String msg;
    msg.data = s.c_str();
    pub.publish(msg);
}

void sync(Transport tr){
    //std::cout << tr.toString() << std::endl;
    transport_string = tr.toString();
    if(playing){
    	ros::Duration elapsed = ros::Time::now() - playback_start_time;
    
    	int beats = ((tr.bar - 1) * tr.timeSignature.beatsPerBar) + (tr.beat - 1);
    	t = ((double)(beats + (tr.unit/tr.resolution))/tr.tempo) * 60.0;
    	
    	sync_offset = t - elapsed.toSec();
    }
}

void playbackChanged(Playback p){
	playing = p.playing;
	if(playing){
		playback_start_time = ros::Time::now();
	}
}


void send(const char* address, std::string msg, UdpTransmitSocket& transmit_socket){
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginBundleImmediate
        << osc::BeginMessage( address ) 
            << msg.c_str() << osc::EndMessage
        << osc::EndBundle;
    
    transmit_socket.Send( p.Data(), p.Size() );
}

void send(const char* address, int msg, UdpTransmitSocket& transmit_socket){
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginBundleImmediate
        << osc::BeginMessage( address ) 
            << msg << osc::EndMessage
        << osc::EndBundle;
        
    transmit_socket.Send( p.Data(), p.Size() );
}

void send(const char* address, float msg, UdpTransmitSocket& transmit_socket){
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
    
    p << osc::BeginBundleImmediate
        << osc::BeginMessage( address ) 
            << msg << osc::EndMessage
        << osc::EndBundle;
        
    transmit_socket.Send( p.Data(), p.Size() );
}

void listen(){

    std::cout << "press ctrl-c to end\n";
    std::cout << "osc thread is working\n";

    receive_socket->Run();
}

void ros_stuff(){
	int one = 1;
	int& argc = one;
	char* name = "terpsichore";
	char** argv = &name;
	ros::init(argc, argv, "osc_relay");
//    ros::init(argc, argv, "osc_receiver");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    std::string command_channel = n.resolveName("osc_test");
    pub = n.advertise<std_msgs::String>(command_channel,50);
    std::cout << "ros thread is working\n";	
    
    while(!quit){
        ros::spinOnce();
        
        if(playing){
        	last_update_time = ros::Time::now();
        	t = (last_update_time - playback_start_time).toSec() + sync_offset;
        	
	        //send("/time", (float)t, transmit_socket);
	        time_string = std::to_string(t);
	        
        }
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    if(argc < 2){
        std::cerr << "enter the address" << std::endl;
        return 1;
    }
    ADDRESS = argv[1];
    std::cout << ADDRESS << std::endl;
    
    //Initialize OSC stuff
    UdpTransmitSocket transmit_socket( IpEndpointName( ADDRESS.c_str(), PORT ) );
    listener = new LRPacketListener();
    listener->registerStringCallback(publish);
    listener->registerTransportCallback(sync);
    listener->registerPlaybackCallback(playbackChanged);
    receive_socket = new UdpListeningReceiveSocket(IpEndpointName( IpEndpointName::ANY_ADDRESS, PORT ), listener);
    
    XInitThreads();
    
    //Set up threads
    std::thread listen_thread(listen);
    std::thread ros_thread(ros_stuff);

	//interupt quits
    signal(SIGINT, [](int signum){std::cout << "okay" << std::endl; quit = true; receive_socket->Break(); receive_socket->AsynchronousBreak();});
    
    //SFML
    sf::Font font;
	if (!font.loadFromFile("Ubuntu-R.ttf"))
	{
		std::cout << "where's the font?" << std::endl;
	}
	play_shape.rotate(90);
	play_shape.setPosition(700, 10);
	play_shape.setFillColor(sf::Color::Green);
	stop_shape.setPosition(700, 10);
	stop_shape.setFillColor(sf::Color::Red);

	// select the font
	transport_text.setFont(font); // font is a sf::Font
	time_text.setFont(font);
	transport_text.setCharacterSize(24); // in pixels, not points!
	time_text.setCharacterSize(24);
	transport_text.setColor(sf::Color::White);
	time_text.setColor(sf::Color::White);
	transport_text.setPosition(10, 10);
	time_text.setPosition(10, 40);

	// set the string to display
	transport_text.setString("Hello world");
	time_text.setString("waiting...");

    //sfml window
    sf::ContextSettings settings;
	settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(800, 600), "Terpsichore", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);
    
    // run the program as long as the window is open
    while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed){
                window.close();
                quit = true;
                receive_socket->Break(); 
                receive_socket->AsynchronousBreak();
            }
        }
        // clear the window with black color
        window.clear(sf::Color::Black);

        // draw everything here...
        // window.draw(...);
        transport_text.setString(transport_string);
        window.draw(transport_text);
        time_text.setString(time_string);
        window.draw(time_text);

		if(playing){
			window.draw(play_shape);
		}else{
			window.draw(stop_shape);
		}

        // end the current frame
        window.display();
    }
    
    
    //stopping threads
    std::cout << "attempting to join\n";
    listen_thread.join();
    ros_thread.join();
    
    delete receive_socket;
    delete listener;

    return 0;
}