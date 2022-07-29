#include "p2p_interface.h"

extern enum  State state;

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
    // uint8_t counter = p->data[1];
    uint32_t now_ms = T2M(xTaskGetTickCount());

    copters[received_id].id=received_id;
    copters[received_id].counter=p->data[1];
    copters[received_id].state = p->data[2];
    copters[received_id].battery_voltage = p->data[15];
    copters[received_id].terminateApp = p->data[16] == 1;

    copters[received_id].timestamp=now_ms;

    if (copters[received_id].terminateApp){
        DEBUG_PRINT("Copter %d has requested to terminate the application\n", received_id);
        if (!getTerminateApp() && state!=STATE_WAIT_FOR_POSITION_LOCK ){
            setTerminateApp(true);
        }
    }

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
        copters[i].state = STATE_UNKNOWN;
    }
}

bool isAlive(uint8_t copter_id){
    uint32_t dt = T2M(xTaskGetTickCount()) - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}

uint8_t compressVoltage(float voltage){
    return (uint8_t)((voltage-VOLTAGE_MIN)/(VOLTAGE_MAX-VOLTAGE_MIN)*255);
}

float decompressVoltage(uint8_t voltage){
    return (voltage/255.0f)*(VOLTAGE_MAX-VOLTAGE_MIN)+VOLTAGE_MIN;
}

bool atLeastOneCopterHasFlown(void){
    for(int i=0;i<MAX_ADDRESS;i++){
        if(copters[i].state != STATE_UNKNOWN){
            return true;
        }
    }
    return false;
}

void printOtherCopters(void){
    for(int i=0;i<MAX_ADDRESS;i++){
        if (copters[i].state != STATE_UNKNOWN){
            if (!peerLocalizationIsIDActive(i)){
                DEBUG_PRINT("Copter %d is not active\n",i);
            }else{
                peerLocalizationOtherPosition_t *pos= peerLocalizationGetPositionByID(i);
                DEBUG_PRINT("Copter %d : %.2f , %.2f , %.2f --> %d with latest counter %d \n",i,(double)pos->pos.x,(double)pos->pos.y,(double)pos->pos.z,copters[i].state,copters[i].counter);
            }
        }
    }
}

uint8_t otherCoptersActiveNumber(void){
    uint8_t nr=0;
    for(int i=0;i<MAX_ADDRESS;i++){
        // if they are active and not requesting to terminate the application
        if (peerLocalizationIsIDActive(i) && !copters[i].terminateApp ){
            nr++;
        }
    }
    return nr;
}

bool isCopterIdActive(uint8_t copter_id){
    uint32_t now = T2M(xTaskGetTickCount());
    uint32_t dt = now - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}
