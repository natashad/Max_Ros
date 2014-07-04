#include "LRReceiver.h"
#include "Translator.h"
#include <oscpack/ip/UdpSocket.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <std_msgs/Float64.h>
#include <terpsichore/bardata.h>
#include <ros/ros.h>
#include <thread>
#include <csignal>
#include <string>
#include <iostream>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <X11/Xlib.h>
 
#define PORT 7100
#define OUTPUT_BUFFER_SIZE 1024
char buffer[OUTPUT_BUFFER_SIZE];

bool playing = false;
bool quit = false;
	
//graphical stuff
std::string transport_string;
sf::Text transport_text;
std::string time_string;
sf::Text time_text;
sf::CircleShape play_shape(40, 3);
sf::RectangleShape stop_shape(sf::Vector2f(70, 70));
std::string offset_string;
sf::Text offset_text;
sf::Text debug_text;
sf::CircleShape baton(10, 3);

std::string ADDRESS;
LRPacketListener* listener;
UdpListeningReceiveSocket* receive_socket;
Conductor conductor;

ros::Publisher data_pub;
ros::Time last_update_time;
ros::Time playback_start_time;
Transport last_transport;
double sync_offset;
double t;

terpsichore::bardata bardata;

void sync(Transport tr){
    //std::cout << tr.toString() << std::endl;
    transport_string = tr.toString();
    if(playing){
    	Transport diff = tr - last_transport;
    	t = diff.toSec();
    	ros::Duration elapsed = ros::Time::now() - last_update_time;
    	sync_offset = t - elapsed.toSec();
    	offset_string = std::to_string(sync_offset);
    	
    	//upon reaching a new bar, obtain note info for the next bar
    	if(tr.position.bar != last_transport.position.bar || tr.tempo != last_transport.tempo){
    		double cur_t = (ros::Time::now() - playback_start_time).toSec();
    		
    		//information about the beats
    		std::vector<terpsichore::pair> beats;
			//binary bars
			//if(tr.timeSignature.beatsPerBar % 2 == 0){
				for(int i = 1; i <= tr.timeSignature.beatsPerBar; i++){
					double beats_ahead = (1 * tr.timeSignature.beatsPerBar) + (i - 1);	
    				double beat_time = ((double)(beats_ahead + (tr.position.unit/tr.position.resolution))/tr.tempo) * 60.0 + cur_t;
					
					terpsichore::pair b;
					std::vector<double> d;
					b.t = beat_time;
					
					if(i % 2 == 0){
					//weak beat
						d.push_back(0.5);
					}else{
					//stronk beat
						d.push_back(1.0);
					}
					b.data = d;
					beats.push_back(b);
				}
			//}
    		bardata.beats = beats;
    		
    		std::vector<terpsichore::pair> events;
    		//grab the first clip in the map
    		std::map<int, Clip*>::iterator it = listener->clips.begin();
    		if(it != listener->clips.end()){
				Clip* c = it->second;
				
				//std::cout << "now in " << tr.position.toString() << std::endl;
				for(std::multimap<Position, Note>::iterator it = c->notes.begin(); it != c->notes.end(); it++){
					Position p = it->first;
					if(p.bar == (tr.position.bar + 1) || (tr.position.bar) % c->length.bar + 1 == p.bar){
						Note n = it->second;
						terpsichore::pair d;
						d.t = (1.0 * tr.timeSignature.beatsPerBar + p.toFloat(tr.timeSignature))/tr.tempo * 60.0 + cur_t;
						double scaled_pitch = (double)(n.pitch)/(double)(108);
						double scaled_vel = double(n.velocity)/(double)(127);
		
						std::vector<double> info;
						info.push_back(scaled_pitch);
						info.push_back((double)n.duration);
						info.push_back(scaled_vel);
						d.data = info;
						events.push_back(d);
						//std::cout << "added note in " << p.toString() << std::endl;
					}else{
						//std::cout << "not adding the note in " << p.toString() << std::endl;
						continue;
					}
			
				}
				bardata.events = events;
			}
    		
    		data_pub.publish(bardata);
    	}
    }
	last_update_time = ros::Time::now();
    last_transport = tr;
}

void loadClip(Clip* c){
	std::vector<terpsichore::pair> note_data;
	for(std::multimap<Position, Note>::iterator it = c->notes.begin(); it != c->notes.end(); it++){
		Position p = it->first;
		if(p.bar > 2)
			break;
			
		Note n = it->second;
		terpsichore::pair d;
		d.t = p.toFloat(last_transport.timeSignature)/last_transport.tempo * 60.0;
		double scaled_pitch = (double)(n.pitch - 21)/(double)(108-21);
		double scaled_vel = double(n.velocity)/(double)(127);
		
		std::vector<double> info;
		info.push_back(scaled_pitch);
		info.push_back((double)n.duration);
		info.push_back(scaled_vel);
		d.data = info;
		note_data.push_back(d);
	}
	bardata.events = note_data;
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

void update_baton(Transport tr){
	conductor.calculatePosition(tr);
	baton.setPosition(300 + 200*conductor.y, 400 - 200*conductor.z);
}

void ros_stuff(){
	int one = 1;
	int& argc = one;
	char* name = "terpsichore";
	char** argv = &name;
	ros::init(argc, argv, "osc_relay");
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    std::string timing_channel = n.resolveName("/terpsichore/ableton_time");
    std::string music_data_channel = n.resolveName("/terpsichore/music_data");
    ros::Publisher time_pub = n.advertise<std_msgs::Float64>(timing_channel,50);
    data_pub = n.advertise<terpsichore::bardata>(music_data_channel,50);
    std::cout << "ros thread is working\n";	
    
    while(!quit){
        ros::spinOnce();
        
        if(playing){
        	last_update_time = ros::Time::now();
        	t = (last_update_time - playback_start_time).toSec();// + sync_offset;
        	
        	std_msgs::Float64 time_data;
        	time_data.data = t;
	        time_pub.publish(time_data);
	        //send("/time", (float)t, transmit_socket);
	        
	        time_string = "t = " + std::to_string(t) + " sec";
	        
        }
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    if(argc < 2){
        ADDRESS = "localhost";
    }else{
    	ADDRESS = argv[1];
    }
    std::cout << ADDRESS << std::endl;
    
    //start ros thread
    XInitThreads();
    std::thread ros_thread(ros_stuff);
    
    //Initialize OSC stuff
    UdpTransmitSocket transmit_socket( IpEndpointName( ADDRESS.c_str(), PORT ) );
    
    listener = new LRPacketListener();
    //listener->registerStringCallback(nothing);
    listener->registerTransportCallback(sync);
    listener->registerPlaybackCallback(playbackChanged);
    listener->registerClipUpdateCallback(loadClip);
    receive_socket = new UdpListeningReceiveSocket(IpEndpointName( IpEndpointName::ANY_ADDRESS, PORT ), listener);
    
    //Set up threads
    std::thread listen_thread(listen);

	//interupt quits
    signal(SIGINT, [](int signum){std::cout << "okay" << std::endl; quit = true; receive_socket->Break(); receive_socket->AsynchronousBreak();});
    
    //conductor
    listener->registerTransportCallback(update_baton);
    
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
	baton.setFillColor(sf::Color::Red);

	// select the font
	transport_text.setFont(font); // font is a sf::Font
	time_text.setFont(font);
	offset_text.setFont(font);
	debug_text.setFont(font);
	transport_text.setCharacterSize(24); // in pixels, not points!
	time_text.setCharacterSize(24);
	offset_text.setCharacterSize(24);
	debug_text.setCharacterSize(24);
	transport_text.setColor(sf::Color::White);
	time_text.setColor(sf::Color::White);
	offset_text.setColor(sf::Color::White);
	debug_text.setColor(sf::Color::White);
	transport_text.setPosition(10, 10);
	time_text.setPosition(10, 40);
	offset_text.setPosition(10, 70);
	debug_text.setPosition(400, 70);

	// set the string to display
	transport_text.setString("Hello world");
	time_text.setString("waiting...");
	offset_text.setString("no offset");

    //sfml window
    sf::ContextSettings settings;
	settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(800, 600), "Terpsichore", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);
    
        
    //request initial information
    send("/terpsichore", (int)1, transmit_socket);
    
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
        
        // draw the notes of the currently playing clips
        double y = 100.0;
        double scale = 50.0;
        for(std::map<int, Clip*>::iterator clip_it = listener->clips.begin(); clip_it != listener->clips.end(); clip_it++){
        	
        	Clip* c = clip_it->second;
        	for(std::multimap<Position, Note>::iterator note_it = c->notes.begin(); note_it != c->notes.end(); note_it++){
        		Position p = note_it->first;
        		Note n = note_it->second;
        		double x = p.toFloat(listener->transport.timeSignature) * scale + 10;
        		double w = n.duration * scale;
        		double h = n.pitch;
        		
        		sf::RectangleShape noteRect(sf::Vector2f(w, h));
        		noteRect.setFillColor(sf::Color::Blue);
        		noteRect.setOutlineThickness(2);
        		noteRect.setOutlineColor(sf::Color::Cyan);
        		noteRect.setPosition(x, y);
        		window.draw(noteRect); 
        	}
        	y += 80;
        	debug_text.setString(std::to_string(y));
        }
        
        
        // window.draw(...);
        transport_text.setString(transport_string);
        window.draw(transport_text);
        time_text.setString(time_string);
        window.draw(time_text);
        offset_text.setString(offset_string);
        window.draw(offset_text);
        window.draw(debug_text);

		if(playing){
			window.draw(play_shape);
		}else{
			window.draw(stop_shape);
		}
        //draw the baton point;
        window.draw(baton);

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