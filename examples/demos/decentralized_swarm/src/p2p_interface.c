#include "p2p_interface.h"

copter_t copters[MAX_ADDRESS];
// uint8_t otherStates[MAX_ADDRESS];//array of states of the other drones

uint8_t getCopterState(uint8_t copter_id){
    return copters[copter_id].state;
}

void p2pcallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    // uint8_t rssi = p->rssi;
    uint8_t received_id = p->data[0];
    uint8_t counter = p->data[1];
    uint32_t now_ms = T2M(xTaskGetTickCount());

    copters[received_id].id=received_id;
    copters[received_id].counter=counter;
    copters[received_id].state = p->data[2];
    copters[received_id].timestamp=now_ms;

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
        copters[i].state = 255;
    }
}

bool isAlive(uint8_t copter_id){
    uint32_t dt = T2M(xTaskGetTickCount()) - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}