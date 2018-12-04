#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
// use spline to optimize the original interplotation method 
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0;

  h.onMessage([&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		
		bool too_close = false;
		double targete_vel = 0;

		int path_size = previous_path_x.size();
		double future_car_s;
		if(path_size >0){
			future_car_s = end_path_s;
		}
		
		for(int i=0; i<sensor_fusion.size(); i++){
			double d = sensor_fusion[i][6];
			if(d>4*lane && d<4*lane+4){
				double front_car_vx = sensor_fusion[i][3];
				double front_car_vy = sensor_fusion[i][4];
				double front_car_v = sqrt(front_car_vx*front_car_vx + front_car_vy*front_car_vy);
				double front_car_s = sensor_fusion[i][5];

				double future_front_car_s = front_car_s+front_car_v*0.02*path_size;

				if((future_front_car_s - future_car_s < 30) && (future_front_car_s > future_car_s)){
					too_close = true;
					targete_vel = front_car_v;
				}
			}

		}
		

		bool left_have_space = false;
		bool right_have_space = false;
		bool change_lane = false;

		if(too_close){
			int left_car_in_space = 0;
			int right_car_in_space = 0;
			vector<double> future_left_car;
			vector<double> future_right_car;

			for(int i=0; i<sensor_fusion.size(); i++){
				if(lane > 0){
					double d_lane_l = sensor_fusion[i][6];
					if(d_lane_l >4*(lane-1) && d_lane_l<4*(lane-1)+4){
						double left_car_vx = sensor_fusion[i][3];
						double left_car_vy = sensor_fusion[i][4];
						double left_car_v = sqrt(left_car_vx*left_car_vx + left_car_vy*left_car_vy);
						double left_car_s = sensor_fusion[i][5];

						double future_left_car_s = left_car_s+left_car_v*0.02*path_size;
						future_left_car.push_back(future_left_car_s);
					}	
				}

				if(lane <2){
					double d_lane_r = sensor_fusion[i][6];
					if(d_lane_r >4*(lane+1) && d_lane_r<4*(lane+1)+4){
						double right_car_vx = sensor_fusion[i][3];
						double right_car_vy = sensor_fusion[i][4];
						double right_car_v = sqrt(right_car_vx*right_car_vx + right_car_vy*right_car_vy);
						double right_car_s = sensor_fusion[i][5];

						double future_right_car_s = right_car_s+right_car_v*0.02*path_size;
						future_right_car.push_back(future_right_car_s);
					}
				}
			}

			

			for(int j=0; j<future_left_car.size(); j++){
				if(future_left_car[j]>(future_car_s - 15) && future_left_car[j]< (future_car_s + 40)){
					left_car_in_space +=1;
				}
			}

			if(lane>0 && left_car_in_space == 0){
			
				left_have_space = true;
			}



			for(int j=0; j<future_right_car.size(); j++){
				if(future_right_car[j]>(future_car_s - 15) && future_right_car[j]< (future_car_s + 40)){
					right_car_in_space +=1;
				}
			}

			if(lane<2 && right_car_in_space == 0){
				right_have_space = true;
			}

		}



		if(too_close && left_have_space){
				lane -=1;
				change_lane = true;
		}
		else if(too_close && right_have_space){
				lane +=1;
				change_lane = true;
		}


		if(too_close && !left_have_space && !right_have_space){
			if(ref_vel > targete_vel){
				ref_vel -=0.13;
			}
		}
		else if(ref_vel<49.5){
			ref_vel +=0.15;
		}


		vector<double> spine_x;
		vector<double> spine_y;
		double pos_x, pos_y;

		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		if(path_size < 5){ 
			pos_x = ref_x - cos(ref_yaw);
			pos_y = ref_y - sin(ref_yaw);
			spine_x.push_back(pos_x);
			spine_y.push_back(pos_y);
			
			//vector<double> getxy= getXY(car_s - 30, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//spine_x.push_back(getxy[0]);
			//spine_y.push_back(getxy[1]);

			spine_x.push_back(ref_x);
			spine_y.push_back(ref_y);

		}
		else{
			ref_x = previous_path_x[path_size-1];
			ref_y = previous_path_y[path_size-1];

			double pre_ref_x = previous_path_x[path_size-5];
			double pre_ref_y = previous_path_y[path_size-5];

			spine_x.push_back(pre_ref_x);
			spine_y.push_back(pre_ref_y);

			spine_x.push_back(ref_x);
		        spine_y.push_back(ref_y);

			ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x); 
		}

		

		//vector<double> getxy1,getxy2,getxy3,getxy4,getxy5,getxy6,getxy7;
		vector<double> getxy1,getxy2,getxy3;
		if(change_lane = true){
			getxy1= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			getxy2= getXY(car_s + 80, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			getxy3= getXY(car_s + 100, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy4= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy5= getXY(car_s + 100, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy6= getXY(car_s + 110, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy7= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		}
		else{
			getxy1= getXY(car_s + 30, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			getxy2= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy4= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy5= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy6= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			//getxy7= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		}

		spine_x.push_back(getxy1[0]);
		spine_x.push_back(getxy2[0]);
		spine_x.push_back(getxy3[0]);
		//spine_x.push_back(getxy4[0]);
		//spine_x.push_back(getxy5[0]);
		//spine_x.push_back(getxy6[0]);
		//spine_x.push_back(getxy7[0]);

		spine_y.push_back(getxy1[1]);
		spine_y.push_back(getxy2[1]);
		spine_y.push_back(getxy3[1]);
		//spine_y.push_back(getxy4[1]);
		//spine_y.push_back(getxy5[1]);
		//spine_y.push_back(getxy6[1]);
		//spine_y.push_back(getxy7[1]);

		
		for(int i =0; i<spine_x.size(); i++){
			double shifto_car_x = spine_x[i] - ref_x;
		        double shifto_car_y = spine_y[i] - ref_y;

			spine_x[i] = shifto_car_x*cos(0-ref_yaw) - shifto_car_y*sin(0-ref_yaw);
			spine_y[i] = shifto_car_x*sin(0-ref_yaw) + shifto_car_y*cos(0-ref_yaw);
		}

		tk::spline s;
		s.set_points(spine_x, spine_y);

		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt(target_x*target_x + target_y*target_y);
		double x_add = 0;

		 for(int i = 0; i<previous_path_x.size(); i++) {
			 next_x_vals.push_back(previous_path_x[i]);
		   	 next_y_vals.push_back(previous_path_y[i]);
	               }


		for(int i = 1; i<= 30-previous_path_x.size(); i++){
			double num = target_dist/(0.02*ref_vel/2.24);
			double x_point = x_add + target_x/num;
			double y_point = s(x_point);

			x_add = x_point;

			double x_ref = x_point;
			double y_ref = y_point;

			x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
			y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

			x_point += ref_x;
			y_point += ref_y;


			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);

		}


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
