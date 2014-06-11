#ifndef __LRRECEIVER__
#define __LRRECEIVER__

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
	
	Position():bar(1), beat(1), unit(0){};
	Position(int a):bar(a), beat(0), unit(0){};
	float toFloat(TimeSignature, double);
	string toString();
	bool operator<(const Position&) const;
	bool operator>(const Position&) const;
	bool operator==(const Position&);
	
	static Position parse(float, double, TimeSignature);
};

struct Transport {
	Position position;
	float tempo;
	float resolution;
	TimeSignature timeSignature;

	Transport() : tempo(0), resolution(RESOLUTION), timeSignature() {}
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
typedef void (*ClipUpdateCallbackType)(Clip);

class LRPacketListener : public osc::OscPacketListener {
    protected:

        vector<StringCallbackType> stringCallbacks;
        vector<TransportCallbackType> transportCallbacks;
		vector<PlaybackCallbackType> playbackCallbacks;
		vector<ClipUpdateCallbackType> clipUpdateCallbacks;

        bool barUpdated, beatUpdated, unitUpdated, tempoUpdated, timeSigUpdated, resUpdated;
        
        virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint);
        
    public:
        
        LRPacketListener();
        ~LRPacketListener();
        
		vector<Track> tracks;
		map<int, Clip> clips;		
        Transport transport;
        
        void registerStringCallback(StringCallbackType cb);
        
        void registerTransportCallback(TransportCallbackType cb);
        
        void registerPlaybackCallback(PlaybackCallbackType cb);
        
        void registerClipUpdateCallback(ClipUpdateCallbackType cb);
};



#endif