#ifndef __LIVERECEIVER__
#define __LIVERECEIVER__

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

#define RESOLUTION	480

struct TimeSignature {
    int beatUnit;
    int beatsPerBar;
    TimeSignature() : beatUnit(4), beatsPerBar(4) {}
    TimeSignature parse(string);
    string toString();
};

struct Position {
	int bar;
	int beat;
	float unit;
	float resolution;
	
	Position():bar(1), beat(1), unit(0), resolution(RESOLUTION){};
	Position(int a):bar(a), beat(0), unit(0), resolution(RESOLUTION){};
	float toFloat(TimeSignature	);
	string toString();
	bool operator<(const Position&) const;
	bool operator>(const Position&) const;
	bool operator==(const Position&);
	
	static Position parse(float, TimeSignature);
};

struct Transport {
	Position position;
	float tempo;
	TimeSignature timeSignature;

	Transport() : tempo(0), timeSignature() {}
	string toString();
	double toSec();
	
	const Transport operator+(const Transport&);
	const Transport operator-(const Transport&);
};

struct Playback {
    bool playing;
};

struct Note {
	int pitch;
	int velocity;
	float duration;
	bool muted;
};

struct Clip {
	int id;
	string name;
	multimap<Position, Note> notes;
	Position start;
	Position length;
	bool isLooping;	
	bool isPlaying;
	bool isTriggered;
	
	Clip():id(0), name(""), start(), length(8), isLooping(false), isPlaying(true), isTriggered(false){};
	Clip(const Clip& o):id(o.id), name(o.name), start(o.start), length(o.length), 
				isLooping(o.isLooping), isPlaying(o.isPlaying), 
				isTriggered(o.isTriggered) { notes = o.notes;};
};

struct Track {
	string name;
	vector<Clip*> clips;
};

typedef void (*StringCallbackType)(string);
typedef void (*TransportCallbackType)(Transport);
typedef void (*PlaybackCallbackType)(Playback);
typedef void (*ClipUpdateCallbackType)(Clip*);

class LRPacketListener : public osc::OscPacketListener {
    protected:

        vector<StringCallbackType> stringCallbacks;
        vector<TransportCallbackType> transportCallbacks;
		vector<PlaybackCallbackType> playbackCallbacks;
		vector<ClipUpdateCallbackType> clipUpdateCallbacks;
        
        virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint);
        
    public:
        
        LRPacketListener();
        ~LRPacketListener();
        
		vector<Track> tracks;
		map<int, Clip*> clips;		
        Transport transport;
        
        void registerStringCallback(StringCallbackType cb);
        
        void registerTransportCallback(TransportCallbackType cb);
        
        void registerPlaybackCallback(PlaybackCallbackType cb);
        
        void registerClipUpdateCallback(ClipUpdateCallbackType cb);
};



#endif