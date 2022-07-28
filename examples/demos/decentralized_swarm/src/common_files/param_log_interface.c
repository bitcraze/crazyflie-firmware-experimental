#include "param_log_interface.h"

//App parameters
static bool takeOffWhenReady = false;
static bool terminateTrajectoryAndLand = false;
static bool terminateApp = false;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdStateEstimateVx;
static logVarId_t logIdStateEstimateVy;
static logVarId_t logIdStateEstimateVz;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;
static logVarId_t logIdPmState;
static logVarId_t logIdVBat;
static logVarId_t logIdlighthouseEstBs0Rt;
static logVarId_t logIdlighthouseEstBs1Rt;
static paramVarId_t paramIdStabilizerController;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdLighthouseMethod;
static paramVarId_t paramIdCollisionAvoidanceEnable;
static paramVarId_t paramIdCollisionAvoidanceEllipsoidX;
static paramVarId_t paramIdCollisionAvoidanceEllipsoidY;
static paramVarId_t paramIdCollisionAvoidanceHorizon;
static paramVarId_t paramIdCollisionAvoidanceMaxVel;

static paramVarId_t paramIdCollisionAvoidanceBBoxMaxX;
static paramVarId_t paramIdCollisionAvoidanceBBoxMaxY;
static paramVarId_t paramIdCollisionAvoidanceBBoxMaxZ;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinX;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinY;
static paramVarId_t paramIdCollisionAvoidanceBBoxMinZ;

// getting  parameters
float getX() { return (float) logGetFloat(logIdStateEstimateX); }
float getY() { return (float) logGetFloat(logIdStateEstimateY); }
float getZ() { return (float) logGetFloat(logIdStateEstimateZ); }
float getVx() { return (float) logGetFloat(logIdStateEstimateVx); }
float getVy() { return (float) logGetFloat(logIdStateEstimateVy); }
float getVz() { return (float) logGetFloat(logIdStateEstimateVz); }
float getVelMagnitude() { return sqrtf(getVx()*getVx() + getVy()*getVy() + getVz()*getVz()); }
float getVarPX() { return logGetFloat(logIdKalmanVarPX); }
float getVarPY() { return logGetFloat(logIdKalmanVarPY); }
float getVarPZ() { return logGetFloat(logIdKalmanVarPZ); }
bool isBatLow() { return logGetInt(logIdPmState) == lowPower; }
float getVoltage() { return logGetFloat(logIdVBat); }
bool isCharging() { return logGetInt(logIdPmState) == charging; }
bool isLighthouseAvailable() { return logGetFloat(logIdlighthouseEstBs0Rt) >= 0.0f || logGetFloat(logIdlighthouseEstBs1Rt) >= 0.0f; }

//App Parameters setters
void setTerminateTrajectoryAndLand(bool value) { terminateTrajectoryAndLand = value; }
void setTakeOffWhenReady(bool value) { takeOffWhenReady = value; }
void setTerminateApp(bool value) { terminateApp = value; }

//App Parameters getters
bool getTerminateTrajectoryAndLand() { return terminateTrajectoryAndLand; }
bool getTakeOffWhenReady() { return takeOffWhenReady; }
bool getTerminateApp() { return terminateApp; }

void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }
void enableCollisionAvoidance() { 
    static bool enabled = false;
    if (!enabled) {
        DEBUG_PRINT("Enabling Collision Avoidance\n");
        paramSetInt(paramIdCollisionAvoidanceEnable, 1);
        enabled = true;
    }
}

void initCollisionAvoidance(){
    paramSetFloat(paramIdCollisionAvoidanceEllipsoidX, COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS);
    paramSetFloat(paramIdCollisionAvoidanceEllipsoidY, COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS);
    paramSetFloat(paramIdCollisionAvoidanceHorizon, COLLISION_AVOIDANCE_HORIZON);
    paramSetFloat(paramIdCollisionAvoidanceMaxVel, COLLISION_AVOIDANCE_MAX_VELOCITY);
    
    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxX, COLLISION_AVOIDANCE_BBOX_MAX_X);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxY, COLLISION_AVOIDANCE_BBOX_MAX_Y);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMaxZ, COLLISION_AVOIDANCE_BBOX_MAX_Z);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinX, COLLISION_AVOIDANCE_BBOX_MIN_X);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinY, COLLISION_AVOIDANCE_BBOX_MIN_Y);
    paramSetFloat(paramIdCollisionAvoidanceBBoxMinZ, COLLISION_AVOIDANCE_BBOX_MIN_Z);

}

void initLogIds(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateVx = logGetVarId("stateEstimate", "vx");
    logIdStateEstimateVy = logGetVarId("stateEstimate", "vy");
    logIdStateEstimateVz = logGetVarId("stateEstimate", "vz");
}

void initParamLogInterface(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdKalmanVarPX = logGetVarId("kalman", "varPX");
    logIdKalmanVarPY = logGetVarId("kalman", "varPY");
    logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");
    logIdPmState = logGetVarId("pm", "state");
    logIdVBat = logGetVarId("pm", "vbat");
    logIdlighthouseEstBs0Rt = logGetVarId("lighthouse", "estBs0Rt");
    logIdlighthouseEstBs1Rt = logGetVarId("lighthouse", "estBs1Rt");
    paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdLighthouseMethod = paramGetVarId("lighthouse", "method");
    paramIdCollisionAvoidanceEnable = paramGetVarId("colAv", "enable");
    paramIdCollisionAvoidanceEllipsoidX = paramGetVarId("colAv", "ellipsoidX");
    paramIdCollisionAvoidanceEllipsoidY = paramGetVarId("colAv", "ellipsoidY");
    paramIdCollisionAvoidanceHorizon = paramGetVarId("colAv", "horizon");
    paramIdCollisionAvoidanceMaxVel = paramGetVarId("colAv", "maxSpeed");
    paramIdCollisionAvoidanceBBoxMaxX = paramGetVarId("colAv", "bboxMaxX");
    paramIdCollisionAvoidanceBBoxMaxY = paramGetVarId("colAv", "bboxMaxY");
    paramIdCollisionAvoidanceBBoxMaxZ = paramGetVarId("colAv", "bboxMaxZ");
    paramIdCollisionAvoidanceBBoxMinX = paramGetVarId("colAv", "bboxMinX");
    paramIdCollisionAvoidanceBBoxMinY = paramGetVarId("colAv", "bboxMinY");
    paramIdCollisionAvoidanceBBoxMinZ = paramGetVarId("colAv", "bboxMinZ");
     
    initLogIds();
}

PARAM_GROUP_START(app)
  PARAM_ADD(PARAM_UINT8, takeoff, &takeOffWhenReady)
  PARAM_ADD(PARAM_UINT8, stop, &terminateTrajectoryAndLand)
  PARAM_ADD(PARAM_UINT8, terminateApp, &terminateApp)
PARAM_GROUP_STOP(app)