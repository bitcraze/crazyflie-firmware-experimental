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
 * positions.h - Positioning data types and functions
 * 
 */

#ifndef POSITIONS_H
#define POSITIONS_H

#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "settings.h"
#include "stdlib.h"

//Transform a peerLocalizationOtherPosition_t to Position
#define OTHER_PEER_LOCALIZATION_POS_TO_POSITION(other) (Position){other->pos.x, other->pos.y, other->pos.z}

// vector calculations
#define ADD_VECTORS_3D(a, b) {a.x += b.x; a.y += b.y; a.z += b.z;}
#define SUB_VECTORS_3D(a, b) {a.x -= b.x; a.y -= b.y; a.z -= b.z;}
#define MUL_VECTORS_3D(a, b) {a.x *= b.x; a.y *= b.y; a.z *= b.z;}
#define MUL_VECTOR_3D_WITH_SCALAR(v, scalar) {v.x *= scalar; v.y *= scalar; v.z *= scalar;}
#define PRINT_POSITION_3D(pos) {DEBUG_PRINT("(%f , %f , %f)\n", (double) pos.x, (double) pos.y, (double) pos.z);}

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define DISTANCE3D(a, b) sqrtf(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)) + ((a.z - b.z) * (a.z - b.z)))
#define DISTANCE2D(a, b) sqrtf(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)))
typedef struct Position_struct {
    float x;
    float y;
    float z;
} Position;


//The following functions are used to return a new Position struct
//If you want to save the result of teh calculation, you can use macros
Position addVectors3D(Position a, Position b);

Position subtractVectors3D(Position a, Position b);

Position multiplyVectorWithScalar(Position a, float scalar);

float getVectorMagnitude3D(Position a);

float getVectorMagnitude2D(Position a);

float getDistanceBetweenVectors3D(Position a, Position b);

//Returns the id of the closest position to the given position
uint8_t getIdWithClosestDistance(Position p,Position positions[10],uint8_t positions_len );

Position getRandomPositionOnCircle();
#endif // POSITIONS_H
