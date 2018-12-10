#ifndef vehicle_H
#define vehicle_H

#include <math.h>
#include <vector>

using namespace std; 

double cost_of_all(double weight_crash, double weight_buffer, double weight_save_time,
		double diff_front_s, double diff_after_s, double our_vehicle_v,
		double front_vehicle_v, double after_vehicle_v, double max_v);

/*
double change_left(double weight_crash, double weight_buffer, double weight_save_time,
		double diff_front_s, double diff_after_s, double our_vehicle_v,
		double front_vehicle_v, double after_vehicle_v, double max_v, int PATH_SIZE);

double change_right(double weight_crash, double weight_buffer, double weight_save_time,
		double diff_front_s, double diff_after_s, double our_vehicle_v,
		double front_vehicle_v, double after_vehicle_v, double max_v, int PATH_SIZE);
*/

bool vehicle(vector<double> prediction_front, vector<double> prediction_left_right,
		double &ref_vel, double &lane, double max_v);


#endif
