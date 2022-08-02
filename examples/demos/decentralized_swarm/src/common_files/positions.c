/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * positions.c - Positioning data types and functions
 * 
 */

#include "positions.h"

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

Position getRandomPositionOnCircle() {
    Position result;
    float step = 2.0f * (float) M_PI / NUMBER_OF_RANDOM_POINTS_ON_CIRCLE;
    float angle = (rand() % NUMBER_OF_RANDOM_POINTS_ON_CIRCLE) * step;
    result.x = CIRCLE_RADIUS * (float) cos(angle);
    result.y = CIRCLE_RADIUS * (float) sin(angle);
    result.z = TAKE_OFF_HEIGHT;

    return result;
}
