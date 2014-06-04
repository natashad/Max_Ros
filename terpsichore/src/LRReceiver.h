#ifndef __LRRECEIVER__
#define __LRRECEIVER__

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>
#include <string>
#include <vector>

using namespace std;

struct TimeSignature {
    int beatUnit;
    int beatsPerBar;
    TimeSignature() : beatUnit(4), beatsPerBar(4) {}
    TimeSignature parse(string);
    string toString();
};

struct Transport {
	int bar;
	int beat;
	float unit;
	float tempo;
	float resolution;
	TimeSignature timeSignature;

	Transport() : bar(0), beat(0), unit(0), tempo(0), resolution(480) {}
	string toString();
};

struct Playback {
    bool playing;
};

typedef void (*StringCallbackType)(string);
typedef void (*TransportCallbackType)(Transport);
typedef void (*PlaybackCallbackType)(Playback);

class LRPacketListener : public osc::OscPacketListener {
    protected:

        vector<StringCallbackType> stringCallbacks;
        vector<TransportCallbackType> transportCallbacks;
		vector<PlaybackCallbackType> playbackCallbacks;

        bool barUpdated, beatUpdated, unitUpdated, tempoUpdated, timeSigUpdated, resUpdated;
        
        virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint);
        
    public:
        
        Transport transport;
        
        void registerStringCallback(StringCallbackType cb);
        
        void registerTransportCallback(TransportCallbackType cb);
        
        void registerPlaybackCallback(PlaybackCallbackType cb);
};



#endif