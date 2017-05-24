/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * sitaw.h - Interface to situation awareness.
 */

#ifndef __SEQUENCE_H__
#define __SEQUENCE_H__

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

typedef struct {
  point_t* end;
  point_t* curr;
  point_t* last;
  point_t* first;
  bool reverseMode;
} sequence_t;

void sequenceInit(sequence_t* seq, uint32_t length, point_t* data);
void sequenceInitStatic(sequence_t* seq, uint32_t length, point_t* data);
void sequenceRecord(sequence_t* seq, point_t* point);
point_t* sequenceReplay(sequence_t* seq);
bool sequenceHasNext(sequence_t* seq);
void sequenceReset(sequence_t* seq);
void sequenceResetReverse(sequence_t* seq);
bool sequenceIsClosedLoop(sequence_t* seq);

#endif // __SEQUENCE_H__