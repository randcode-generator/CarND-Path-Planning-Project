#include "StateMachine.h"
#include <math.h>
#include <iostream>

using namespace std;

sensor_sig sensor_signals;

void StateMachine::setCarInfo(int current_lane, double car_x, double car_y, double car_s, double car_d) {
  this->current_lane = current_lane;
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
}

// Check if car can enter the specified lane
bool StateMachine::isLaneChangable(bool &laneChangable, int lane, double x, double y, double s, double d) {
  bool isCarInFront = false;
  if(d < 2+4*lane+2 && d > 2+4*lane-2) {
    float distance = sqrt((car_x-x)*(car_x-x) + (car_y-y)*(car_y-y));
    if(distance < 30 && car_s > s) {
      cout<<"lane: " << lane <<" "<<distance<<"m car behind"<<endl;
      laneChangable = false;
    }
    if(distance < 30 && car_s < s) {
      cout<<"lane: "<< lane <<" " <<distance <<"m car in front"<<endl;
      laneChangable = false;
      isCarInFront = true;
    }
  }
  return isCarInFront;
}

sensor_sig StateMachine::process_sensors(json::iterator begin, json::iterator end) {
  bool tooClose = false;
  bool emergencyBrakes = false;
  bool laneChangable_right = true;
  bool laneChangable_left = true;

  for (json::iterator it = begin; it != end; ++it) {
    float id = (*it)[0];
    float x = (*it)[1];
    float y = (*it)[2];
    float s = (*it)[5];
    float d = (*it)[6];

    if(current_lane == 1) {
      bool left_car_inFront = isLaneChangable(laneChangable_left, 0, x, y, s, d);
      bool right_car_inFront = isLaneChangable(laneChangable_right, 2, x, y, s, d);
      
      // Choose the lane that does not have a car in front
      // If both lane has car in front, report true for both lane
      if(left_car_inFront) {
        laneChangable_left = laneChangable_left & false;
        laneChangable_right = laneChangable_right & true;
      } else if(right_car_inFront) {
        laneChangable_left = laneChangable_left & true;
        laneChangable_right = laneChangable_right & false;
      }

    } else if(current_lane == 0) {
      isLaneChangable(laneChangable_right, 1, x, y, s, d);
      laneChangable_left = false;
    } else if(current_lane == 2) {
      isLaneChangable(laneChangable_left, 1, x, y, s, d);
      laneChangable_right = false;
    } else {
      laneChangable_right = false;
      laneChangable_left = false;
    }

    // If the distance between current car and car in front is:
    // less than 30: toggle tooClose and increment count
    // less than 10: toggle emergencyBrakes
    if(d < 2+4*current_lane+2 && d > 2+4*current_lane-2 && s > car_s) {
      float distance = sqrt((car_x-x)*(car_x-x) + (car_y-y)*(car_y-y));
      if(distance < 30) {
        cout<< "lane: "<<current_lane<< " " <<distance <<"m in front, count: "<<tooCloseCount<<endl;
        tooClose = true;
        tooCloseCount++;
      }
      if(distance < 15) {
        emergencyBrakes = true;
      }
    }

    // Once lane changed occurred, make sure not to do anything while it is changing lane
    if(laneChangedCount > 0) {
      laneChangedCount++;
      laneChangable_left = false;
      laneChangable_right = false;
    }
    if(laneChangedCount > 300) {
      laneChangedCount = 0;
    }
  }
  sensor_signals.tooCloseCount = tooCloseCount;
  sensor_signals.tooClose = tooClose;
  sensor_signals.emergencyBrakes = emergencyBrakes;
  sensor_signals.laneChangable_left = laneChangable_left;
  sensor_signals.laneChangable_right = laneChangable_right;

  return sensor_signals;
}