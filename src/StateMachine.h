#include<tuple>
#include "json.hpp"

using json = nlohmann::json;

class StateMachine {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  int current_lane;
  
	int tooCloseCount = 0;
	int laneChangedCount = 0;

  bool isLaneChangable(bool &laneChangable, int lane, double x, double y, double s, double d);

  public:
  std::tuple<int, bool, bool, bool, bool> process_sensors(json::iterator begin, json::iterator end);
  void setCarInfo(int current_lane, double car_x, double car_y, double car_s, double car_d);
  void setCurrentLane(int lane) { 
    current_lane = lane;
    tooCloseCount = 0;
    laneChangedCount = 1;
  }
};