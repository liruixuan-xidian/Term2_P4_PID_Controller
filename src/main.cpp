#include <uWS/uWS.h>
#include <iostream>
#include <algorithm>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Resetting the Simulator
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

// Create a twiddle class
struct parameter{
  bool fwd_flag, bwd_flag;
  double val;
};

class Twiddle {
  int max_dist, np;
  public:
    std::vector<parameter> dp;
    std::vector<double> best_param;
    int dist_count, ind, iter;
    double best_err, err, avg_err;
    bool is_used, is_initialized;
    Twiddle(int);
    bool distance_reached();
    double sum_dp();
    //void initialize();
};

Twiddle::Twiddle(int maxval){
  max_dist = maxval;
  np = 3;	    // number of parameters for twiddle
  dist_count = 0;
  iter = 0;
  err = 0;
  avg_err = 0;
  best_err = 0;
  for (int i=0; i< np; i++){
    parameter p;
    p.fwd_flag = false;
    p.bwd_flag = false;
    p.val = 1;
    dp.push_back(p);
  }
  is_initialized = false;
  is_used = false;      // Default value is false
}

bool Twiddle::distance_reached(){
  return dist_count >= max_dist;
}

double Twiddle::sum_dp(){
  double sum=0;
  for (std::vector<parameter>::iterator it = dp.begin(); it!= dp.end(); ++it){
    sum += (*it).val;
  }
  return sum;
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable. 0.3,0,3.5 @34
  double kp_init, ki_init, kd_init;
  int max_dist;
  if (argc == 1){
    kp_init = 0.64;
    ki_init = 0.0002;
    kd_init = 7.5;
    max_dist = 4000;
  }
  else if(argc == 4){
    kp_init = atof(argv[1]);
    ki_init = atof(argv[2]);
    kd_init = atof(argv[3]);
    max_dist = 2000;
  }
  else{
    kp_init = atof(argv[1]);
    ki_init = atof(argv[2]);
    kd_init = atof(argv[3]);
    max_dist = atoi(argv[4]);
  }
  pid.Init(kp_init, ki_init, kd_init);
  Twiddle tw(max_dist);     // initializing twiddle
  // To not use twiddle, comment the line below.
  //tw.is_used = true;
  h.onMessage([&pid,&tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      //std::cout<<"the dist count is "<< dist_count<< std::endl;
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
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
	  // using twiddle
	  if (tw.is_used){
	    // Updating distance count and error
	    tw.dist_count += 1;
	    //if (tw.dist_count > 100)
	    tw.err += cte*cte;   
            // storing average error
            tw.avg_err = tw.err/tw.dist_count;
	    // Checking if distance count has exceeded limit or the vehicle has gone off the road.
	    if(tw.distance_reached()||(std::fabs(cte) >= 2.2)){
	      // Twiddle initialization - assign best_err to the first err calculation
              //if(std::fabs(cte) >= 2.2){
                //tw.avg_err += 50;
                //std::cout<<"This has run across the road\n";
              //}
              //std::cout<<"sum of dp: "<<tw.sum_dp()<<std::endl;
	      if(!tw.is_initialized){
		//tw.initialize();
                //std::cout <<"Twiddle is about to be initialized..\n";
                tw.best_err = tw.avg_err;
                tw.ind = 0;
                tw.dp[0].fwd_flag = true;
                tw.dp[0].bwd_flag = false;
                // going forward post initialization
                pid.Kp += tw.dp[0].val;
                tw.is_initialized = true;
                //std::cout <<"Initialization is done!\n";
              }
	      else {
	        // if error condition passes
		if (tw.avg_err < tw.best_err){
                  //std::cout<<"new best error found!\n";
		  tw.best_err = tw.avg_err;
                  // update dp
                  tw.dp[tw.ind].val *= 1.1;
                  //std::cout<<"the dp value is multiplied be 1.1\n";
                  // change index since best config found!
                  tw.ind = (tw.ind + 1) % 3;
                  tw.dp[tw.ind].fwd_flag = true;
                  tw.dp[tw.ind].bwd_flag = false;
                  std::cout<<"Best PID params- Kp: "<<pid.Kp<<" Ki: "<<pid.Ki<<" Kd: "<<pid.Kd<<"\n";
                  //std::cout<<"The index is changed by 1 and fwd_flag is 1 and bwd_flag is 0\n";
		}
                else{
                  // change direction if unsuccessful in the forward direction
                  if (tw.dp[tw.ind].fwd_flag){
		    tw.dp[tw.ind].bwd_flag = true;
		    tw.dp[tw.ind].fwd_flag = false;
                    // changing direction of parameter
                    switch(tw.ind){
                      case 0:
                        pid.Kp -= 2*tw.dp[0].val;
                        break;
                      case 1:
                        pid.Ki -= 2*tw.dp[1].val;
                        break;
                      case 2:
                        pid.Kd -= 2*tw.dp[2].val;
                        break;
                    }
                    //reset_simulator(ws);
                    //std::cout << "It was unsuccessful in the forward direction of "<<tw.ind<<" index\n";
                    //std::cout << "The parameters go two steps back\n";
		  }
                  // reduce factor if unsuccessful in the backward direction too
		  else {
		    //bringing back previous configuration of PID parameters
                    switch(tw.ind){
                      case 0:
                        pid.Kp += tw.dp[0].val;
                        break;
                      case 1:
                        pid.Ki += tw.dp[1].val;
                        break;
                      case 2:
                        pid.Kd += tw.dp[2].val;
                        break;
                    }
                    //std::cout << "it was unsuccessful in the backward direction of "<<tw.ind<<" index\n";
                    // updating dp.val
                    tw.dp[tw.ind].val *= 0.9;
                    //std::cout <<"The dp value is multiplied by 0.9 and the parameters are brought back\n";
                    // change index since best config not found
                    tw.ind = (tw.ind + 1) % 3;
                    tw.dp[tw.ind].fwd_flag = true;
                    tw.dp[tw.ind].bwd_flag = false;
                    //std::cout<<"we now move onto the new "<<tw.ind<<" index with fwd_flag as 1 and bwd_flag as 0\n";
		  }
		}
	        // updating state
	        if(tw.dp[tw.ind].fwd_flag) {
                  // going one step forward
		  switch(tw.ind){
		    case 0:
		      pid.Kp += tw.dp[0].val;
		      break;
		    case 1:
		      pid.Ki += tw.dp[1].val;
		      break;
		    case 2:
		      pid.Kd += tw.dp[2].val;
		      break;
		  }
                  //reset_simulator(ws);      //reseting the simulator after updating params
                  //std::cout<<"and we take one step forward in "<<tw.ind<<" index\n";
	        }
              }
	      // reinitializing dist and err
	      tw.dist_count = 0;
	      tw.err = 0;
              ++tw.iter;
              std::cout<<"iteration= "<<tw.iter<<std::endl;
              std::cout<<"The sum of dp = "<<tw.sum_dp()<<std::endl;
              std::cout<<"PID- Kp: "<<pid.Kp<<" Ki: "<<pid.Ki<<" Kd: "<<pid.Kd<<std::endl;
              std::cout <<"Average error: "<< tw.avg_err<< std::endl;
              //std::cout<<"reinitializing dist_count and err and resetting the simulator\n";
	      // reset simulator
              //if (std::fabs(cte)>2.2){
              reset_simulator(ws);
              //}
	      //reset_simulator(ws);
	    }
	  }
	  // Calculating Steering value
          pid.UpdateError(cte);
	  steer_value = -pid.TotalError();
          if (steer_value > 1)
            steer_value = 1;
          else if(steer_value < -1)
            steer_value = -1;
          // DEBUG
          //std::cout << "chosen index: "<< tw.ind <<" Forward: "<< tw.dp[tw.ind].fwd_flag;
          //std::cout << " Backward: " << tw.dp[tw.ind].bwd_flag<< std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "P Gain: " << pid.Kp <<" I Gain: "<< pid.Ki<<" D Gain: "<<pid.Kd<< std::endl;
          std::cout << "Mean Squared Error: "<<pid.mean_sq_err<< std::endl;
          //std::cout << "Average error: " << tw.avg_err <<" Best error: "<<tw.best_err << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3/35*50;
          //msgJson["throttle"] = 0.3/35*(35+15*(1-std::fabs(steer_value)));
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
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
