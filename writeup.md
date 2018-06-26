# CarND-Path-Planning-Project

The goal of this project was to get a car to drive around a track with other cars present.

## Architecture

### Getting the car to move in smooth motion
In order to get the car to move in a smooth motion, we do the following:
1) Get the previous x, y coordinates, current x, y coordinates, and coordinates of waypoints 30, 60, 90 meters away 
```
ref_x = previous_path_x[prev_size-1];
ref_y = previous_path_y[prev_size-1];

double ref_x_prev = previous_path_x[prev_size-2];
double ref_y_prev = previous_path_y[prev_size-2];
ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

ptsx.push_back(ref_x_prev);
ptsx.push_back(ref_x);

ptsy.push_back(ref_y_prev);
ptsy.push_back(ref_y);
```
```
vector<double>next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double>next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double>next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

2) Convert the global coordinates to car coordinates
```
double shift_x = ptsx[i]-ref_x;
double shift_y = ptsy[i]-ref_y;

ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
```

3) Calculate spline of the 5 points. 

```
s.set_points(ptsx, ptsy);
```

4) Calculate the spline y value given the specified x interval
```
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x+target_y*target_y);
```
```
// Number of points
double N = target_dist/interval;
```
```
double x_point = x_pos+target_x/N;
double y_point = s(x_point);
```

5) Convert the values back to global coordinates
```
x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
```

### Lane changing
1) Lane change is triggered when the car has to slow down for a certain period of time
```
if(d < 2+4*current_lane+2 && d > 2+4*current_lane-2 && s > car_s) {
  float distance = sqrt((car_x-x)*(car_x-x) + (car_y-y)*(car_y-y));
  if(distance < 30) {
    cout<< "lane: "<<current_lane<< " " <<distance <<"m in front, count: "<<tooCloseCount<<endl;
    tooClose = true;
    tooCloseCount++;
  }
}
```
2) Once it is determined that lane change is necessary, the car must slow down to 31 mph to prevent max acceleration and jerk warnings
```
if(tooCloseCount > 100 && car_speed <= 31) {
  // Change lane to left lane
  if(laneChangable_left==true) {
    lane--;
    sm.setCurrentLane(lane);
    cout<<"count reset left_lane"<<endl;
  }
  // Otherwise change to right lane
  else if(laneChangable_right==true) {
    lane++;
    sm.setCurrentLane(lane);
    cout<<"count reset right_lane"<<endl;
  }
}
```
3) Check if the car can enter specified lane. If there are cars in front and back of that specific lane, make sure the distance between the car and cars in the other lane is 30.
```
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
``` 

4) If you are in the middle lane, the lane that does not have a car in front, takes precedence. Otherwise, left lane takes precedence.
```
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
``` 

```
if(laneChangable_left==true) {
  lane--;
  sm.setCurrentLane(lane);
  cout<<"count reset left_lane"<<endl;
}
// Otherwise change to right lane
else if(laneChangable_right==true) {
  lane++;
  sm.setCurrentLane(lane);
  cout<<"count reset right_lane"<<endl;
}
```
5) Make sure the car cannot change lane while it's in middle of changing lane
```
// Once lane changed occurred, make sure not to do anything while it is changing lane
if(laneChangedCount > 0) {
  laneChangedCount++;
  laneChangable_left = false;
  laneChangable_right = false;
}
if(laneChangedCount > 300) {
  laneChangedCount = 0;
}
```

### Speed logic
1) If the car is not at 44 mph, then increase the interval to increase the acceleration
```
// If the speed is not near speed limit and it's not too close to car in front, increase the interval
if(car_speed < 44.0 && tooClose == false) {
  interval += 0.0009;
}
```

2) If the car is too close (less than 30) to the car in front, decelerate
```
if(d < 2+4*current_lane+2 && d > 2+4*current_lane-2 && s > car_s) {
  float distance = sqrt((car_x-x)*(car_x-x) + (car_y-y)*(car_y-y));
  if(distance < 30) {
    cout<< "lane: "<<current_lane<< " " <<distance <<"m in front, count: "<<tooCloseCount<<endl;
    tooClose = true;
    tooCloseCount++;
  }
}
```
```
// If tooClose is true, subtract from the interval to slow down
if(tooClose) {
  interval -= 0.0009;
}
```

1) If the car is dangerously close (less than 15) to the car in front, floor the brakes
```
if(distance < 15) {
  emergencyBrakes = true;
}
```
```
if(emergencyBrakes) {
  interval -= 0.001;
}
```

4) If the car at falls below 28 mph, accelerate to maintain speed
```
// If car speed is too slow, accelerate to maintain speed
if(car_speed <= 20.0) {
  interval += 0.001;
}
```