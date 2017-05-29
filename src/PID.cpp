#include "PID.h"
#include <iostream>

#define MAX_UPDATES 1000

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	p_error = 0;
	i_error = 0;
	d_error = 0;

	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	// initialize twiddel d's to half of gains or 0.1, whichever is lower.
	d[0] = d[0] < .5*Kp ? d[0] : 0.5*Kp;
	d[1] = d[1] < .5*Kd ? d[1] : 0.5*Kd;
	d[2] = d[2] < .5*Ki ? d[2] : 0.5*Ki;
};


void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;


}

double PID::TotalError() {
	double err = -Kp_ * p_error - Kd_ * d_error - Ki_ * i_error;

	// bound the error output [1, -1]
	if (err > 1.0) {
		return 1.0;
	}
	else if (err < -1.0) {
		return -1.0;
	}
	else {
		return err;
	}
}

bool PID::getTwiddleFlag() {
	return twiddle_flag;
}

double PID::twiddle(double err) {

	if (!twiddle_flag) return 0;

	// If a d value is small, stop tuning the parameter
	if (d[twiddle_ind] <= 1e-3) twiddle_ind = (twiddle_ind + 1) % 3;

	cout.precision(6);
	cout << "twiddle_ind: " << twiddle_ind << " num_ind_updates: " << num_ind_updates << " best_err: " << best_err << endl;
	cout << "d: " << d[0] <<", " << d[1] << ", " << d[2] << endl;

	i_error = 0;
	p_error = 0;
	d_error = 0;

	double sum_d = d[0] + d[1] + d[2];

	// Conditions to stopp tuning parameters
	if (sum_d <= 1e-2 || err<=0.1) {
		twiddle_flag = false;
		if (err < best_err) {
			best_err = err;
		}
		
		cout.precision(6);
		cout << "Finished Twiddle with error: " << best_err<< endl;
		cout << "Best parameters:" << endl;
		cout << "Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << endl;
		cout << "Minimum error obtained: " << best_err << endl;
		return sum_d;
	}
	
	// Initial parameter output
	if (best_err > 1e7) {
		best_err = err;
		cout << "Inital parameters: " << endl;
		cout << "Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << endl;
		cout << "Initial error: " << best_err << endl;
	}


	// Performing twiddle
	// num_ind_updates represents the times a parameter was changed
	// and error was checkes
	if (num_ind_updates == 0) {

		if (err < best_err) {
			best_err = err;
		}
		if (twiddle_ind == 0) Kp_ += d[twiddle_ind];
		else if (twiddle_ind == 1) Kd_ += d[twiddle_ind];
		else Ki_ += d[twiddle_ind];

		num_ind_updates = 1;

		cout << "Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << endl;
		return sum_d;
	}
	else if (num_ind_updates == 1) {
		if (err < best_err) {
			best_err = err;
			d[twiddle_ind] *= 1.1;

			twiddle_ind = (twiddle_ind + 1) % 3;
			num_ind_updates = 0;
		}
		else {
			if (twiddle_ind == 0) Kp_ -= 2*d[twiddle_ind];
			else if (twiddle_ind == 1) Kd_ -= 2*d[twiddle_ind];
			else Ki_ -= 2*d[twiddle_ind];

			num_ind_updates = 2;
		}
		cout.precision(6);
		cout << "Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << endl;
		return sum_d;
	}
	else {
		if (err < best_err) {
			best_err = err;
			d[twiddle_ind] *= 1.1;
		}
		else {
			if (twiddle_ind == 0) Kp_ += d[twiddle_ind];
			else if (twiddle_ind == 1) Kd_ += d[twiddle_ind];
			else Ki_ += d[twiddle_ind];

			d[twiddle_ind] *= 0.9;
		}

		twiddle_ind = (twiddle_ind + 1) % 3;
		num_ind_updates = 0;
		cout.precision(6);
		cout << "Kp: " << Kp_ << " Kd: " << Kd_ << " Ki: " << Ki_ << endl;
		return sum_d;
	}
	
}

