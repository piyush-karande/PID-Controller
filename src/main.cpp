#define _USE_MATH_DEFINES

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define NUM_TWIDDLE_FRAMES 800

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
std::stringstream hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return std::stringstream();
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		std::stringstream tmp = std::stringstream();
		tmp.str(s.substr(b1, b2 - b1 + 1));
		return tmp;
	}
	return std::stringstream();
}


int main()
{
	uWS::Hub h;

	PID pid;
	// TODO: Initialize the pid variable.
	//pid.Init(.1, .004, 1.);

	// Final best parameters obtained after several runs on the simulator
	pid.Init(0.178899, 0.0022, 1.57257);

	int frames_since_update = 0;
	double squared_error = 0;

	h.onMessage([&pid, &frames_since_update, &squared_error](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data));
			if (s.str() != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;
					/*
					* TODO: Calcuate steering value here, remember the steering value is
					* [-1, 1].
					* NOTE: Feel free to play around with the throttle and speed. Maybe use
					* another PID controller to control the speed!
					*/

					// reset simulator if car is going off the track
					if (fabs(cte) > 4) {
						std::string msg = "42[\"reset\",{}]";
						//std::cout << msg << std::endl;
						(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}


					pid.UpdateError(cte);
					steer_value = pid.TotalError();

					// change trottle based on steering angle.
					// smaller values at sharper turns
					double acc = 0.4 - .4*(fabs(steer_value));
					frames_since_update += 1;

					// Calculating error for twiddle
					if (frames_since_update >= 150) squared_error += cte*cte;
					if (frames_since_update >= NUM_TWIDDLE_FRAMES && pid.getTwiddleFlag()) {
						double avg_err = squared_error / (NUM_TWIDDLE_FRAMES-150);
						cout << "avg_err: " << avg_err << endl;
						double sum_d = pid.twiddle(avg_err);
						/*
						if (sum_d > 1e-3 && squared_error > 1e-3) {
							std::string msg = "42[\"reset\",{}]";
							//std::cout << msg << std::endl;
							(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}
						*/
						frames_since_update = 0;
						squared_error = 0;
					}
					
					// DEBUG
					//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = acc;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		(*ws).close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen("0.0.0.0", port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}