#include "p2p_interface.h"

// copter_t copters[MAX_ADDRESS]
uint8_t otherStates[MAX_ADDRESS];//array of states of the other drones

uint8_t getCopterState(uint8_t copter_id){
    return otherStates[copter_id];
}

void p2pcallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    // uint8_t rssi = p->rssi;
    uint8_t received_id = p->data[0];
    // uint8_t counter = p->data[1];
    otherStates[received_id]=p->data[2];

    positionMeasurement_t pos_measurement;
    memcpy(&pos_measurement.pos, &(p->data[3]), sizeof(Position));
    
    // DEBUG_PRINT("===================================================\n");
    // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, received_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);    
    
    pos_measurement.source =  MeasurementSourceLighthouse;
    pos_measurement.stdDev = 0.01f; //
    peerLocalizationTellPosition(received_id,&pos_measurement);//TODO: if id is 0--> PROBLEM WITH THE LOGIC OF THE PEER LOCALIZATION (maybe add 1 to the id)
}

void initOtherStates(){
    for(int i=0;i<MAX_ADDRESS;i++){
        otherStates[i] = 255;
    }
}
