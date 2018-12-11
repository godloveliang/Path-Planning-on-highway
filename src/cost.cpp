#include "cost.h"
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;


double cost_buffer(double weight_buffer, double diff_s){
	double cost_buf = -1;
	if(diff_s >0){
		 cost_buf =(1.0/diff_s)*weight_buffer;
	}
	return cost_buf;
}


//Punish the crash when traffic in the lane next to our vehicle change to our lane suddenly.
//or the vehicle behind crash into our vechicle.
//punish the crash to beside vehicles when our vehicle turn left or right.
double cost_crash(double weight_crash, double diff_front_s,double diff_after_s){

	double cost_crash_front = -1;
	double cost_crash_after = -1;

	if(diff_front_s > 0){
		if(diff_front_s<15){
			cost_crash_front = (20 - diff_front_s)*weight_crash;
		}
	}

	if(diff_after_s > 0){
		if(diff_after_s<15){
			cost_crash_after = (20 - diff_after_s)*weight_crash;
		}
	}
 
	double cost_crash_max = max(cost_crash_front,cost_crash_after);

	return cost_crash_max;
}


double cost_save_time(double weight_save_time, double front_vehicle_v, double max_v){
	double cost_savetime = -1;
	if(front_vehicle_v > 0){
		cost_savetime = weight_save_time*(max_v - front_vehicle_v)/max_v;
		}

	return cost_savetime;
}

