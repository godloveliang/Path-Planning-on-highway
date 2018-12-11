#include <vector>
#include <iostream>
#include <math.h>
#include "prediction.h"

using namespace std;


vector<double> prediction_front(vector<vector<double>> sensor_fusion,int path_size, const double &lane, double future_car_s){
	double too_close = -1;
	double targete_vel = -1;
	double too_close_min = 1000;
	for(int i=0; i<sensor_fusion.size(); i++){
		double d = sensor_fusion[i][6];
		if(d>4*lane && d<4*lane+4){
			double front_car_vx = sensor_fusion[i][3];
			double front_car_vy = sensor_fusion[i][4];
			double front_car_v = sqrt(front_car_vx*front_car_vx + front_car_vy*front_car_vy);
			double front_car_s = sensor_fusion[i][5];

			double future_front_car_s = front_car_s+front_car_v*0.02*path_size;

			if(future_front_car_s > future_car_s){
				if((future_front_car_s - future_car_s) < too_close_min){
					too_close = future_front_car_s - future_car_s;
					too_close_min = too_close;
					targete_vel = front_car_v;
				}
			}
		}
	}
	//cout<<too_close<<"  "<<targete_vel<<endl;

	return {too_close, targete_vel};
}
vector<double> prediction_left_right(vector<vector<double>> sensor_fusion,int path_size,const double &lane, double future_car_s){
	vector<double> future_s_left_car, future_s_right_car;
	vector<double> future_v_left_car, future_v_right_car;
	for(int i=0; i<sensor_fusion.size(); i++){


		if(lane > 0){
			double d_lane_l = sensor_fusion[i][6];
			if(d_lane_l >4*(lane-1) && d_lane_l<4*(lane-1)+4){
  				double left_car_vx = sensor_fusion[i][3];
				double left_car_vy = sensor_fusion[i][4];
				double left_car_v = sqrt(left_car_vx*left_car_vx + left_car_vy*left_car_vy);
				double left_car_s = sensor_fusion[i][5];

				double future_left_car_s = left_car_s+left_car_v*0.02*path_size;
				future_s_left_car.push_back(future_left_car_s);
				future_v_left_car.push_back(left_car_v);
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
				future_s_right_car.push_back(future_right_car_s);
				future_v_right_car.push_back(right_car_v);
			}
		}
	}


	//choice the car front and behind our car in the future time(the curent path end point).
	double left_front_close = -1;
	double left_front_vel = -1;
	double left_after_close = -1;
	double left_after_vel = -1;
	double left_front_min = 1000;
	double left_after_min = 1000;
	for( int i = 0; i<future_s_left_car.size(); i++){

		if(future_s_left_car[i] > future_car_s){
			if((future_s_left_car[i] - future_car_s)< left_front_min){

				left_front_close = future_s_left_car[i] - future_car_s;
				left_front_min = left_front_close;
				left_front_vel = future_v_left_car[i];
			}
		}
		
		if(future_s_left_car[i] < future_car_s){
		
			if((future_car_s - future_s_left_car[i])< left_after_min){
				left_after_close = future_car_s- future_s_left_car[i];
				left_after_min = left_after_close;
				left_after_vel = future_v_left_car[i];
			}
		}
		
	}

	double right_front_close = -1;
	double right_front_vel = -1;
	double right_after_close = -1;
	double right_after_vel = -1;
	double right_front_min = 1000;
	double right_after_min = 1000;
	for( int i = 0; i<future_s_right_car.size(); i++){

		if(future_s_right_car[i] > future_car_s){
			if((future_s_right_car[i] - future_car_s)< right_front_min){
				right_front_close = future_s_right_car[i] - future_car_s;
				right_front_min = right_front_close;
				right_front_vel = future_v_right_car[i];
			}
		}

		if(future_s_right_car[i] < future_car_s){
			if((future_car_s - future_s_right_car[i])< right_after_min){
				right_after_close = future_car_s- future_s_right_car[i];
				right_after_min = right_after_close;
				right_after_vel = future_v_right_car[i];
			}
		}
	}
	//cout<<left_front_close<<"  "<<right_front_close<<endl;

	return {left_front_close, right_front_close, left_after_close, right_after_close, 
       		left_front_vel, right_front_vel, left_after_vel,right_after_vel};
}
