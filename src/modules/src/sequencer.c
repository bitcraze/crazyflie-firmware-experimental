#include "sequencer.h"

void sequenceInit(sequence_t* seq, uint32_t length, point_t* data){
    seq->first = data;
    seq->end = seq->first + length;
    seq->curr = seq->first;
    seq->last = seq->first;
    seq->reverseMode = false;
}

void sequenceInitStatic(sequence_t* seq, uint32_t length, point_t* data){
    sequenceInit(seq, length, data);
    seq->last = seq->end;
}

void sequenceRecord(sequence_t* seq, point_t* point){
  if(seq->curr < seq->end) {
      *(seq->curr) = *point;
      seq->curr++;
      seq->last = seq->curr;
  }
}

point_t* sequenceReplay(sequence_t* seq){
  point_t* result = 0;

  if (seq->reverseMode) {
    if(seq->curr > seq->first) {
        seq->curr--;
        result = seq->curr;
    }
  } else {
    if(seq->curr < seq->last) {
        result = seq->curr;
        seq->curr++;
    }
  }

  return result;
}

bool sequenceHasNext(sequence_t* seq){
  if (seq->reverseMode) {
    return seq->curr > seq->first;
  } else {
    return seq->curr < seq->last;
  }
}

void sequenceReset(sequence_t* seq){
    seq->curr = seq->first;
}

void sequenceResetReverse(sequence_t* seq){
    seq->reverseMode = true;
    seq->curr = seq->last + 1;
}