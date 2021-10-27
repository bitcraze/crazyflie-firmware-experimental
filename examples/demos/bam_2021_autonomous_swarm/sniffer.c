#include "sniffer.h"
#include "radiolink.h"
#include "app_channel.h"

static void rxCallback(P2PPacket *packet);

void initSniffer() {
    p2pRegisterCB(rxCallback);
}

static void rxCallback(P2PPacket *packet) {
    // Pass on to the app channel.
    // Note: p2p packets can be larger than what can be transmitted on the
    // app channel. We design our p2p protocol in this application to use
    // small packets and this limitation should not be a problem here
    appchannelSendDataPacket(packet->data, packet->size);
}
