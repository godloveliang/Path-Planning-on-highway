#include "vehicle.h"
#include <vector>
#include <iostream>
#include <cmath>
#include "cost.h"

using namespace std;


//the total cost 
double cost_of_all(double weight_crash, double weight_buffer, double weight_save_time, 
		double diff_front_s, double diff_after_s, double our_vehicle_v,
	       double front_vehicle_v, double after_vehicle_v, double max_v){

	double cost_cra = cost_crash(weight_crash, diff_front_s, diff_after_s);

	double cost_buf = cost_buffer(weight_buffer, diff_front_s);
	double cost_sav = cost_save_time(weight_save_time, front_vehicle_v, max_v);
	double cost_all = cost_buf + cost_sav + cost_cra;

	return cost_all;
}


bool vehicle(vector<double> prediction_front, vector<double> prediction_left_right,
		double &ref_vel, double &lane, double max_v){

	double weight_crash = 20;
	double weight_save_time = 50;
	double weight_buffer = 1000;

	double lane_before = lane;
	double cost_keep_lane,cost_change_left, cost_change_right;

	//if have no vehicle or vehicle is vary far away in front of our car, just drive as fast as we can,
	//no need to calculate the three states cost.
	if(prediction_front[0] == -1 || prediction_front[0]>30){   
		if(ref_vel<max_v){
			ref_vel +=0.23;
		}
	}

	// if finde vehicle in front of us, calculate the cost of the three states, choose the min cost state.
	
	// *****************the three states:keep_lane, change_left, change_right.************
	
	else{
	       //when close to front vehicle,and it fast than our car, no need to slow down..	
		if(ref_vel<max_v + 0.5){
			if(prediction_front[1] > ref_vel){
				ref_vel +=0.23;
			}
			//other conditions, when close to front vehicle slow down the acc according to the diff of v.
			else{
				ref_vel -=exp((ref_vel - prediction_front[1])/(max_v+0.5))/8.0;
			}
		}


		cost_keep_lane = cost_of_all(weight_crash, weight_buffer, weight_save_time,
				prediction_front[0], -1, ref_vel, prediction_front[1], -1, max_v);


		if(lane > 0){
			//plus 12 to the cost of change lang, avoid frequent lane changes when cost_change_left
			// and cost_change_right wave aroud cost_keep_lane.
			cost_change_left =10 + cost_of_all(weight_crash, weight_buffer, weight_save_time,
					prediction_left_right[0], prediction_left_right[2], ref_vel,
					prediction_left_right[4],prediction_left_right[6], max_v);
		}
		else{
			cost_change_left = 1000;
		}



		if(lane < 2){
			cost_change_right =12 +  cost_of_all(weight_crash,weight_buffer, weight_save_time,
					prediction_left_right[1],prediction_left_right[3], ref_vel,
					prediction_left_right[5], prediction_left_right[7], max_v);
		}
		else{
			cost_change_right = 1000;
		}
	

 		bool change_left_state = false;
		bool change_right_state = false;

		//set default state keep_lane.
		double min_cost;
		min_cost = cost_keep_lane;

		if(cost_change_left<min_cost){
			min_cost = cost_change_left;
			change_left_state = true;
		}
		if(cost_change_right < min_cost){
			change_right_state = true;
		}

		//when cost_change_right is min, change to right.
		if(change_right_state){
			if(lane < 2){
				lane += 1;
			}
		} 

		//when cost_change_left is min, change to left.
		if(change_left_state && !change_right_state){
			if(lane > 0){
				lane -= 1;
			}
		}
	}
	
	double lane_after = lane;
	bool change_lane = false;
	if(abs(lane_before - lane_after) > 0.1){
		change_lane = true;
	}

	return change_lane;

}
