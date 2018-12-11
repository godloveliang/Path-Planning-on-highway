#include <cmath>
#include <vector>
#include <iostream>
#include "trajectory.h"
#include "spline.h"

using namespace std;

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


vector<vector<double>> trajectory(const vector<double> &pre_path_x, const vector<double> &pre_path_y, double ref_x,
	double ref_y, double ref_yaw, bool change_lane, int PATH_SIZE,double car_s, const vector<double> &map_waypoints_x,
	const vector<double> &map_waypoints_y, const vector<double> &map_waypoints_s, double lane, double ref_vel)
{
	vector<double> spine_x,spine_y;
	vector<double> next_x_vals, next_y_vals;
	vector<vector<double>> trjectoryXY;
	int path_size = pre_path_x.size();

	if(path_size < 5){
		double pos_x = ref_x - cos(ref_yaw);
		double pos_y = ref_y - sin(ref_yaw);
		spine_x.push_back(pos_x);
		spine_y.push_back(pos_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);
	}
	else{
		ref_x = pre_path_x[path_size-1];
		ref_y = pre_path_y[path_size-1];
		double pre_ref_x = pre_path_x[path_size-5];
		double pre_ref_y = pre_path_y[path_size-5];

		spine_x.push_back(pre_ref_x);
		spine_y.push_back(pre_ref_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);

		ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);
	}

	vector<double> getxy1,getxy2,getxy3;
	if(change_lane = true){  //set further points to get more smooth trajectory, avoid large lateral jerk.
		getxy1= getXY(car_s + 50, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 70, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	else{
		getxy1= getXY(car_s + 30, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	spine_x.push_back(getxy1[0]);
	spine_x.push_back(getxy2[0]);
	spine_x.push_back(getxy3[0]);

	spine_y.push_back(getxy1[1]);
	spine_y.push_back(getxy2[1]);
	spine_y.push_back(getxy3[1]);

	//Convert the global coordinate system to the vichle coordinate for math easy.
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

	for(int i = 0; i<pre_path_x.size(); i++) {
		next_x_vals.push_back(pre_path_x[i]);
		next_y_vals.push_back(pre_path_y[i]);
	}

	for(int i = 1; i<= PATH_SIZE - pre_path_x.size(); i++){
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

	trjectoryXY.push_back(next_x_vals);
	trjectoryXY.push_back(next_y_vals);

	return trjectoryXY;

}
