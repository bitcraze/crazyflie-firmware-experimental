#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
// vector calculations
#define ADD_VECTORS_3D(a, b) {a.x += b.x; a.y += b.y; a.z += b.z;}
#define SUB_VECTORS_3D(a, b) {a.x -= b.x; a.y -= b.y; a.z -= b.z;}
#define MUL_VECTORS_3D(a, b) {a.x *= b.x; a.y *= b.y; a.z *= b.z;}
#define MUL_VECTOR_3D_WITH_SCALAR(v, scalar) {v.x *= scalar; v.y *= scalar; v.z *= scalar;}
#define PRINT_POSITION_3D(pos) {DEBUG_PRINT("(%f , %f , %f)\n", (double) pos.x, (double) pos.y, (double) pos.z);}

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define DISTANCE3D(a, b) sqrtf(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)) + ((a.z - b.z) * (a.z - b.z)))

typedef struct Position_struct {
    float x;
    float y;
    float z;
} Position;


//The following functions are used to return a new Position struct
//If you want to save the result of teh calculation, you can use macros
Position addVectors3D(Position a, Position b) {
    Position result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Position subtractVectors3D(Position a, Position b) {
    Position result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Position multiplyVectorWithScalar(Position a, float scalar) {
    Position result;
    result.x = a.x * scalar;
    result.y = a.y * scalar;
    result.z = a.z * scalar;
    return result;
}

float getVectorMagnitude3D(Position a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

float getVectorMagnitude2D(Position a) {
    return sqrt(a.x * a.x + a.y * a.y );
}

float getDistanceBetweenVectors3D(Position a, Position b) {
    return getVectorMagnitude3D(subtractVectors3D(a, b));
}

uint8_t getIdWithClosestDistance(Position p,Position positions[10],uint8_t positions_len ){
    float min_distance = FLT_MAX;
    uint8_t min_id = 0;
    for(uint8_t i = 0; i < positions_len; i++){
        float distance = getDistanceBetweenVectors3D(p,positions[i]);
        if(distance < min_distance){
            min_distance = distance;
            min_id = i;
        }
    }
    return min_id;
}