#ifndef __LATENCYTESTRECEIVER__
#define __LATENCYTESTRECEIVER__

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>
#include <vector>

using namespace std;

typedef void (*CallbackType)();

class LatencyTestListener : public osc::OscPacketListener {
    protected:
		vector<CallbackType> callbacks;

        virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint);
        
    public:
        
        LatencyTestListener();
        ~LatencyTestListener();
        
        void registerCallback(CallbackType cb);
};



#endif