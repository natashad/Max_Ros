#include "LRReceiver.h"
#include <sstream>
#include <iostream>

using namespace std;

TimeSignature TimeSignature::parse(string s){
    return TimeSignature();
}

string TimeSignature::toString(){
    stringstream ss;
    ss << beatUnit << "/" << beatsPerBar;
    return ss.str();
}

string Transport::toString(){
    stringstream ss;
    ss << bar << "." << beat << "." << unit << " at " << tempo << " bpm and " << 
        resolution << " ppq in " << timeSignature.toString() << " time";
    return ss.str();
}

void LRPacketListener::ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint){
    (void) remoteEndpoint; // suppress unused parameter warning

    try{
        // example of parsing single messages. osc::OsckPacketListener
        // handles the bundle traversal.
        /*
        if( std::strcmp( m.AddressPattern(), "/test1" ) == 0 ){
            // example #1 -- argument stream interface
            osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
            bool a1;
            osc::int32 a2;
            float a3;
            const char *a4;
            args >> a1 >> a2 >> a3 >> a4 >> osc::EndMessage;
            
            std::cout << "received '/test1' message with arguments: "
                << a1 << " " << a2 << " " << a3 << " " << a4 << "\n";
            
        *///}else if( std::strcmp( m.AddressPattern(), "/test2" ) == 0 ){
        // example #2 -- argument iterator interface, supports
        // reflection for overloaded messages (eg you can call 
        // (*arg)->IsBool() to check if a bool was passed etc).
        string address = m.AddressPattern();
        if(address.find("/transport") == 0){
            string type = address.substr(10, string::npos);
            
            osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
            
            if(type.find("/bar") == 0){
                args >> transport.bar >> osc::EndMessage;
                barUpdated = true;
            }else if(type.find("/beat") == 0){
                args >> transport.beat >> osc::EndMessage;
                beatUpdated = true;
            }else if(type.find("/unit") == 0){
                args >> transport.unit >> osc::EndMessage;
                unitUpdated = true;
            }else if(type.find("/tempo") == 0){
                args >> transport.tempo >> osc::EndMessage;
                tempoUpdated = true;
            }else if(type.find("/resolution") == 0){
                args >> transport.resolution >> osc::EndMessage;
                resUpdated = true;
            }else if(type.find("/timesig") == 0){
                args >> transport.timeSignature.beatUnit >> transport.timeSignature.beatsPerBar >> osc::EndMessage;
                timeSigUpdated = true;
            }
            
            //cout << "received transport message at " << m.AddressPattern() << endl;
            bool changed = (barUpdated && beatUpdated && unitUpdated) && (tempoUpdated) && (timeSigUpdated) && (resUpdated);
            
            if(changed){
                for(vector<TransportCallbackType>::iterator it = transportCallbacks.begin(); it != transportCallbacks.end(); ++it){
                    (*it)(transport);
                }
                barUpdated = false;
                beatUpdated = false;
                unitUpdated = false;
                tempoUpdated = false;
                timeSigUpdated = false;
                resUpdated = false;
            }
        }else if(address.find("/playback") == 0){
			osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
        	Playback p;
        	int play;
        	args >> play >> osc::EndMessage;
        	if(play != 0)
        		p.playing = true;
        	else
        		p.playing = false;
        	for(vector<PlaybackCallbackType>::iterator it = playbackCallbacks.begin(); it != playbackCallbacks.end(); ++it){
                (*it)(p);
            }
        }else{
            string s;
            stringstream ss;
            
            ss << m.AddressPattern() ;
            for(osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin(); arg != m.ArgumentsEnd(); arg++){
                if(arg->IsBool()){
                    bool a = arg->AsBool();
                    s = to_string(a);
                }else if(arg->IsInt32()){
                    int a = arg->AsInt32();
                    s = to_string(a);
                }else if(arg->IsFloat()){
                    float a = arg->AsFloat();
                    s = to_string(a);
                }else if(arg->IsString()){
                    const char* a = arg->AsString();
                    string s = a;
                }
                ss << "\t" << s;
            }
            ss << endl;
            s = ss.str();
            cout << s;
            
            for(vector<StringCallbackType>::iterator it = stringCallbacks.begin(); it != stringCallbacks.end(); ++it){
                (*it)(s);
            }
        }
    }catch( osc::Exception& e ){
        // any parsing errors such as unexpected argument types, or 
        // missing arguments get thrown as exceptions.
        cout << "error while parsing message: "
            << m.AddressPattern() << ": " << e.what() << "\n";
    }
}
    
void LRPacketListener::registerStringCallback(StringCallbackType cb){
    stringCallbacks.push_back(cb);
}

void LRPacketListener::registerTransportCallback(TransportCallbackType cb){
    transportCallbacks.push_back(cb);
}

void LRPacketListener::registerPlaybackCallback(PlaybackCallbackType cb){
	playbackCallbacks.push_back(cb);
}