#include "LiveReceiver.h"
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;

TimeSignature TimeSignature::parse(string s){
    return TimeSignature();
}

string TimeSignature::toString(){
    stringstream ss;
    ss << beatsPerBar << "/" << beatUnit;
    return ss.str();
}

float Position::toFloat(TimeSignature ts){
	return ((bar - 1)*ts.beatsPerBar) + (beat - 1) + unit/resolution;
}

string Position::toString(){
	stringstream ss;
    ss << bar << "." << beat << "." << unit;
    return ss.str();
}

bool Position::operator<(const Position& d) const
{
	if(bar>d.bar)
		return false;
	else if(bar<d.bar)
		return true;
	else{
		if(beat>d.beat)
			return false;
		else if(beat<d.beat)
			return true;
		else{
			return unit<d.unit;
		}
	}
}

bool Position::operator>(const Position& d) const
{
	if(bar<d.bar)
		return false;
	else if(bar>d.bar)
		return true;
	else{
		if(beat<d.beat)
			return false;
		else if(beat>d.beat)
			return true;
		else{
			return unit>d.unit;
		}
	}
}

bool Position::operator==(const Position& d){
	return (bar == d.bar) && (beat == d.beat) && abs(unit - d.unit) < 10.0;
}

Position Position::parse(float pos, TimeSignature ts){
	Position result;
	result.bar = floor(pos/ts.beatsPerBar) + 1.0;
	result.beat = fmod(pos, (float)ts.beatsPerBar) + 1.0;
	float junk;
	//assume 480 ppq
	result.unit = modf(pos, &junk)*RESOLUTION;
	return result;
}

string Transport::toString(){
    stringstream ss;
    ss << position.bar << "." << position.beat << "." << position.unit << " at " << tempo << " bpm and " << 
        position.resolution << " ppq in " << timeSignature.toString() << " time";
    return ss.str();
}

double Transport::toSec(){
	int beats = ((position.bar - 1) * timeSignature.beatsPerBar) + (position.beat - 1);
    double t = ((double)(beats + (position.unit/position.resolution))/tempo) * 60.0;
    return t;
}

const Transport Transport::operator+(const Transport& other){
	double num_beats_me = ((position.bar - 1.0)*timeSignature.beatsPerBar) + (position.beat - 1.0) + position.unit/position.resolution;
	double num_beats_other = ((other.position.bar - 1.0)*other.timeSignature.beatsPerBar) + (other.position.beat - 1.0) + other.position.unit/other.position.resolution;
	double num_beats_total = num_beats_me + num_beats_other;
	
	//keeping own time signature and tempo
	Transport result;
	result.position.bar = floor(num_beats_total/timeSignature.beatsPerBar) + 1.0;
	result.position.beat = fmod(num_beats_total, (double)timeSignature.beatsPerBar) + 1.0;
	result.position.resolution = this->position.resolution;
	double junk;
	result.position.unit = modf(num_beats_total, &junk)*position.resolution;
	result.tempo = this->tempo;
	result.timeSignature = this->timeSignature;
	return result;
}

const Transport Transport::operator-(const Transport& other){
	double num_beats_me = ((position.bar - 1.0)*timeSignature.beatsPerBar) + (position.beat - 1.0) + position.unit/position.resolution;
	double num_beats_other = ((other.position.bar - 1.0)*other.timeSignature.beatsPerBar) + (other.position.beat - 1.0) + other.position.unit/other.position.resolution;
	double num_beats_total = num_beats_me - num_beats_other;
	
	//keeping own time signature and tempo
	Transport result;
	result.position.bar = floor(num_beats_total/timeSignature.beatsPerBar) + 1.0;
	result.position.beat = fmod(num_beats_total, (double)timeSignature.beatsPerBar) + 1.0;
	double junk;
	result.position.unit = modf(num_beats_total, &junk)*position.resolution;
	result.tempo = this->tempo;
	result.timeSignature = this->timeSignature;
	result.position.resolution = this->position.resolution;
	return result;
}

LRPacketListener::LRPacketListener(){
};

LRPacketListener::~LRPacketListener(){
	for(map<int,Clip*>::iterator it = clips.begin(); it != clips.end(); it++){
		delete it->second;
	}
};

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
            
            args >> transport.position.bar >> transport.position.beat >> transport.position.unit >> transport.position.resolution 
            	>> transport.tempo >> transport.timeSignature.beatsPerBar >> transport.timeSignature.beatUnit >> osc::EndMessage;
            	
            for(vector<TransportCallbackType>::iterator it = transportCallbacks.begin(); it != transportCallbacks.end(); ++it){
                (*it)(transport);
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
        }else if(address.find("/track") == 0){
    	}else if(address.find("/clip") == 0){
			osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
		
			const char* clip_name_cstr = (arg++)->AsString();
			string clip_name = clip_name_cstr;
			cout << clip_name << "\t";
			int clip_id = (arg++)->AsInt32();
			cout << clip_id << "\t";
			const char* track_name_cstr = (arg++)->AsString();
			string track_name = track_name_cstr;
			cout << track_name << "\t";
			int track_id = (arg++)->AsInt32();
			cout << track_id << "\t";
    		
    		string type = address.substr(5, string::npos);
    		if(type.find("/notes") == 0){
				int numnotes = (arg++)->AsInt32();
				cout << numnotes << endl;
				multimap<Position, Note>* notes = (multimap<Position, Note>*)0;
				    		
				map<int, Clip*>::iterator it = clips.find(clip_id);
				Clip* c;
				if(it == clips.end()){
					//new clip, create it
					c = new Clip();
					c->id = clip_id;
					notes = &(c->notes);
				}else{
					//found it
					Clip* old_c = it->second;
					c = new Clip(*old_c);
					notes = &(c->notes);
					clips.erase(clip_id);
				}
				
				for(int i = 0; i < numnotes; i++){
					Note n;
					n.pitch = (arg++)->AsInt32();
					cout << n.pitch << "\t";
					Position pos = Position::parse((arg++)->AsFloat(), transport.timeSignature);
					cout << pos.toString() << "\t";
					n.duration = (arg++)->AsFloat();
					cout << n.duration << "\t";
					n.velocity = (arg++)->AsInt32();
					cout << n.velocity << "\t";
					int muted = (arg++)->AsInt32();
					n.muted = muted != 0;
					cout << n.muted << "\t";
					
					//Check if the note already exists
					pair<multimap<Position, Note>::iterator, multimap<Position, Note>::iterator> existing_notes = notes->equal_range(pos);
					for(multimap<Position, Note>::iterator it2 = existing_notes.first; it2 != existing_notes.second; ++it2){
						Note n2 = it2->second;
						//same pitch?
						if(n2.pitch == n.pitch){
							//overwrite
							notes->erase(it2);
						}
					}
					
					notes->insert(pair<Position, Note>(pos, n));
					cout << "added a note at " << pos.toString() << " to clip " << clip_id << endl;
				}
				clips.insert(std::pair<int, Clip*>(clip_id, c));
				cout << "number of notes in this clip: " << notes->size() << endl;
				for(vector<ClipUpdateCallbackType>::iterator it = clipUpdateCallbacks.begin(); it != clipUpdateCallbacks.end(); ++it){
					cout << "calling callback....";
                    (*it)(c);
                    cout << "done" << endl;
                }
    		}else if(type.find("/playback") == 0){
    			bool playing = ((arg++)->AsInt32() <= 0 ? false : true);
    			bool triggered = ((arg++)->AsInt32() <= 0 ? false : true);
    			
    			map<int, Clip*>::iterator it = clips.find(clip_id);
				Clip* c;
				if(it == clips.end()){
					//clip doesn't exist?
					cerr << "I don't have clip id " << clip_id << endl;
				}else{
					//found it
					c = it->second;
					c->isPlaying = playing;
					c->isTriggered = triggered;
				}
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

void LRPacketListener::registerClipUpdateCallback(ClipUpdateCallbackType cb){
	clipUpdateCallbacks.push_back(cb);
}