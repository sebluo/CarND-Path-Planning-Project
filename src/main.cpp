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

int d2Lane(double d) {
    if (d>0.0 && d < 4.0) return 0;
    else if (d >= 4.0 & d < 8.0) return 1;
    else if (d >= 8.0 & d < 12.0)return 2;

    return -1;
}


vector<double>  closestCarParams(double car_s, int car_l, bool isFront, vector<vector<double> > sensor_fusion) {
    double closest_distance = 100;
    double speed = 0;
    vector<double> params(2);

    for (int i = 0; i < sensor_fusion.size(); i++) {
        if (sensor_fusion[i][6] > car_l*4.0 && sensor_fusion[i][6] < (car_l+1)*4.0) {   //sensor_fusion[i][6] refer to  d value
            double diff = 0.0;
            if (isFront) diff = sensor_fusion[i][5] - car_s;    //sensor_fusion[i][5] refer to  s value
            else diff = car_s - sensor_fusion[i][5];

            if (diff >= 0.0 && diff < closest_distance) {
                closest_distance = diff;

                //sensor_fusion[i][3] :vx   sensor_fusion[i][4]:vy	
	        speed = sqrt(sensor_fusion[i][3] * sensor_fusion[i][3] + sensor_fusion[i][4] * sensor_fusion[i][4]);
            }
        }
    }
      params[0] = closest_distance;
      params[1] = speed;

      return params;
}


double getCostOfStraightCourse(double car_s, int car_l, vector<vector<double> > sensor_fusion) {
    double front_dist = closestCarParams(car_s, car_l, true, sensor_fusion)[0];
    double front_vel = closestCarParams(car_s, car_l, true, sensor_fusion)[1];
    if (front_dist != 0.0) {
        return 1.0/ front_dist;
    }
    return 10000.0;
}

double getCostOfLaneChange(double car_s, int car_l, double car_speed,int direction, vector<vector<double> > sensor_fusion,double sensing_period) {
    int new_lane = car_l + direction;
    if (new_lane < 0 | new_lane > 2) {
        return 10000.0;
    }

    double present_lane_front_dist = closestCarParams(car_s, car_l , true, sensor_fusion)[0];
    double present_lane_behind_dist = closestCarParams(car_s, car_l , false, sensor_fusion)[0];
    double present_lane_front_vel = closestCarParams(car_s, car_l , true, sensor_fusion)[1];
    double present_lane_behind_vel = closestCarParams(car_s, car_l , false, sensor_fusion)[1];

    double new_lane_front_dist = closestCarParams(car_s, new_lane, true, sensor_fusion)[0];
    double new_lane_behind_dist = closestCarParams(car_s, new_lane, false, sensor_fusion)[0];
    double new_lane_front_vel = closestCarParams(car_s, new_lane, true, sensor_fusion)[1];
    double new_lane_behind_vel = closestCarParams(car_s, new_lane, false, sensor_fusion)[1];

    double lane_change_half_time= 33*0.02;
    double behind_smoothy_speed_down_time= ceil(fabs(new_lane_behind_vel - car_speed)/0.22)*sensing_period ;
    double new_lane_behind_safe_distance=  2.5* behind_smoothy_speed_down_time * behind_smoothy_speed_down_time +5.0; //5.0m buffeer

    double front_smoothy_speed_down_time= ceil(fabs(car_speed - new_lane_front_vel )/0.22)*sensing_period ;
    double new_lane_front_safe_distance=  2.5* front_smoothy_speed_down_time * front_smoothy_speed_down_time +5.0;   //5.0m buffer
    

    if (new_lane_front_dist > 30.0 && new_lane_behind_dist > 10.0  ) 
    {   // front safe distance set to 30m,behind safe distance maybe set between (10m,30m),here 10m is bounday test for simulator, in reality ,it is better set to 30.0m
       if(car_speed >= new_lane_behind_vel || ((car_speed < new_lane_behind_vel) &&(new_lane_behind_dist -lane_change_half_time*(new_lane_behind_vel - car_speed) ) >  new_lane_behind_safe_distance ) )
	{
		if(car_speed <= new_lane_front_vel || ((car_speed > new_lane_front_vel) &&((new_lane_front_dist -2.0*lane_change_half_time*(car_speed - new_lane_front_vel ) ) > new_lane_front_safe_distance)))
		{
     //&&(( 3.0*lane_change_half_time*(car_speed - new_lane_front_dist )- present_lane_front_dist  )> 5.0) )                             // more constrained conditons, safer but less chance to change lane
	
		return 1.0 / new_lane_front_dist + 0.5 / new_lane_behind_dist;
		}
	   }
    }

    return 10000.0;
}

int getLowestCostAction(double car_s, int car_l,double car_speed, vector<vector<double> > sensor_fusion,double sensing_period) {

    int action_select ;
    vector<double> costs={};
    
    double left_cost = getCostOfLaneChange(car_s, car_l,car_speed, -1, sensor_fusion,sensing_period);
    double keep_cost = getCostOfStraightCourse(car_s, car_l, sensor_fusion);
    double right_cost = getCostOfLaneChange(car_s, car_l,car_speed, 1, sensor_fusion,sensing_period);
    costs.push_back(left_cost);
    costs.push_back(keep_cost);
    costs.push_back(right_cost);
    sort(costs.begin(),costs.end());

    if(costs[0]==left_cost) action_select=-1;
    if(costs[0]==keep_cost) action_select=-0;
    if(costs[0]==right_cost) action_select=1;

    //cout<<"  action_select:"<<action_select<<"   left_cost: "<<left_cost<<"    keep_cost:"<<keep_cost<< "     right_cost:"<<right_cost<<endl;
  
    return action_select;
}

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

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> transform_global_to_local(double global_x,double global_y,double ref_x,double ref_y,double ref_yaw)
{
			
			double local_x,local_y;
			double shift_x=global_x-ref_x;
			double shift_y=global_y-ref_y;

			local_x=shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
			local_y=shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
			return {local_x,local_y};
}

vector<double> transform_local_to_global(double local_x,double local_y,double ref_x,double ref_y,double ref_yaw)
{
			
			double global_x,global_y;

			global_x=(local_x*cos(ref_yaw)-local_y*sin(ref_yaw) ) + ref_x;
			global_y=(local_x*sin(ref_yaw)+local_y*cos(ref_yaw) ) + ref_y;
			return {global_x,global_y};
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

 int lane=1; //starting from middle lane
 double sensing_period = 0.02; // sensing data feedback every 0.02s 
 double ref_vel=0.22; //mph  starting reference speed    
 double lane_change_interval= 1.0;
 double time_passed_lane_changed=0.0;
 double previous_car_speed=0.0;
 double previous_car_acceleration=0.0;
 double time=0.0;

  // set up logging
  string log_file = "../logging.txt";
  ofstream out_log(log_file.c_str(), ofstream::out);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&lane_change_interval,&time_passed_lane_changed,&previous_car_speed,&previous_car_acceleration,&time,&sensing_period,&out_log](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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



		int prev_path_size=previous_path_x.size();
		if(prev_path_size>0)
		{
			car_s= end_path_s;
		}
		
		bool too_close=false;

		for(int i=0;i<sensor_fusion.size();i++)
		{
			double d=sensor_fusion[i][6];
			if(d>(4*lane)&& d<(4*lane+4))
			{
				double vx=sensor_fusion[i][3];
				double vy=sensor_fusion[i][4];
				double check_speed=sqrt(vx*vx+vy*vy);
				double check_car_s=sensor_fusion[i][5];

				check_car_s+=(double)prev_path_size*sensing_period*check_speed;
				if((car_s<check_car_s) &&  (check_car_s-car_s)<30)  // 30 meters safe distance in 25 mps
				{
					too_close=true;
					//ref_vel=check_speed;
					//change lane logic
					//cout<< " car_s is  "<<car_s <<"  check_car_s is "<<check_car_s<<"  car distance is "<<(check_car_s-car_s)<<endl;
					
				}

			}
		}

		if(too_close)
		{	
			ref_vel -=0.22;  //0.1 mps velocity change during 0.02 seconds , about 5 m/s2
		}
		else
		{
			if(ref_vel < 49.5)
			{
			ref_vel +=0.22; //mph   0.22  corresponding to 5m/t2  ,0.36 corresponding to 8m/t2

			}

		}
		//cout<<" too_close is "<<too_close<<"  ref_vel is "<<ref_vel <<"car_speed is "<<car_speed<<endl;
		//cout<<"car_d is "<<car_d<<endl;

          	json msgJson;
		
		vector<double> ptsx,ptsy;//key points ,maybe 30 meters away each other

		//reference x,y,yaw,  for starting point or end of path points or reselect path points		
		double ref_x=car_x;
		double ref_y=car_y;
		double ref_yaw=deg2rad(car_yaw);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

		

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

   		 double dist_inc = 0.5;
		double next_s,next_d;
		//add the first two new path points derived from end tangent of last state
		if(prev_path_size<2)
		{
			double prev_car_x= car_x - cos(car_yaw);
			double prev_car_y= car_y - sin(car_yaw);

			ptsx.push_back(prev_car_x);
			ptsx.push_back(car_x);
			ptsy.push_back(prev_car_y);
			ptsy.push_back(car_y);

		}
		else
		{
			ref_x= previous_path_x[prev_path_size-1];
			ref_y= previous_path_y[prev_path_size-1];
			
			double prev_ref_x= previous_path_x[prev_path_size-2];
			double prev_ref_y= previous_path_y[prev_path_size-2];

			ref_yaw=atan2(ref_y-prev_ref_y,ref_x-prev_ref_x );
			
			ptsx.push_back(prev_ref_x);
			ptsx.push_back(ref_x);
			ptsy.push_back(prev_ref_y);
			ptsy.push_back(ref_y);

		}
		
                 //lane select according to cost calculation

        	 int end_path_lane = d2Lane(end_path_d);

        	 int action = getLowestCostAction(end_path_s, end_path_lane, car_speed,sensor_fusion,sensing_period);

		  //if(action == 0) lane=lane;
		  //  if(action <0) lane -=1;
                  //  if(action >0) lane +=1;

                 vector<double> setpoints(6);
		 vector<double> next_wp0,next_wp1,next_wp2;

		// add 3 key points every 30 metrs


                //if (action == 0){
			next_wp0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		//}
		time_passed_lane_changed +=sensing_period; //time passed from last lane change
		if (action < 0 ){
			next_wp0=getXY(car_s+sensing_period*33*ref_vel,(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp1=getXY(car_s+sensing_period*66*ref_vel,(-2.0+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			//next_wp2=getXY(car_s+90,(-2.0+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			//setpoints = determineNewCourseSetpoints(car_s, car_l, car_speed, -1);
			if(lane >= 1 && (time_passed_lane_changed >  lane_change_interval)){
				lane -=1;
				time_passed_lane_changed=0.0;
			}
		}
		else if (action >0 ){
			next_wp0=getXY(car_s+sensing_period*33*ref_vel,(4.0+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp1=getXY(car_s+sensing_period*66*ref_vel,(6.0+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			//next_wp2=getXY(car_s+90,(6.0+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			//setpoints = determineNewCourseSetpoints(car_s, car_l, car_speed, 1);
			if(lane<=1  && (time_passed_lane_changed >  lane_change_interval)){
                        	lane +=1;
				time_passed_lane_changed=0.0;
			}
		}
                else{
			next_wp0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			next_wp2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		}

		ptsx.push_back(next_wp0[0]);
		ptsy.push_back(next_wp0[1]);
		ptsx.push_back(next_wp1[0]);
		ptsy.push_back(next_wp1[1]);
		ptsx.push_back(next_wp2[0]);
		ptsy.push_back(next_wp2[1]);

		//tranformation from gloabl coordonee to car coordonnee
		for(int i=0;i<ptsx.size();i++)
		{
			vector<double> local_xy= transform_global_to_local(ptsx[i],ptsy[i],ref_x,ref_y,ref_yaw);
			ptsx[i]=local_xy[0];
			ptsy[i]=local_xy[1];
			//double shift_x=ptsx[i]-ref_x;
			//double shift_y=ptsy[i]-ref_y;

			//ptsx[i]=shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
			//ptsy[i]=shift_x*sin(0-ref-yaw)+shift_y*cos(0-ref_yaw);
		}

		//create a spline
		tk::spline s;
		s.set_points(ptsx,ptsy);

          	vector<double> next_s_vals;
          	vector<double> next_d_vals;

		//add the rest of previous path points. this can be optimized 
		for(int i=0;i<previous_path_x.size();i++)
		{

			vector<double>  sd = getFrenet(previous_path_x[i], previous_path_y[i], ref_yaw, map_waypoints_x,map_waypoints_y);

			next_s_vals.push_back(sd[0]);
			next_d_vals.push_back(sd[1]);

			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
 
		}


		double target_x=30.0;
		double target_y=s(target_x);
		double target_distance=sqrt(target_x*target_x+target_y*target_y);
		double x_add_on=0;

			
    		for(int i = 1; i < 25-previous_path_x.size(); i++)
   		 {
			//next_s=car_s+(i+1)*dist_inc;
			//vector<double> xy=getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
			
			double N=target_distance/(sensing_period*ref_vel/2.24);
			double x_point=x_add_on + target_x/N;
			double y_point=s(x_point);
			x_add_on=x_point;

			double x_ref=x_point;		
			double y_ref=y_point;
			//transformation from car coordonate to global coordonnate
			x_point=x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
			y_point=x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
			x_point+=ref_x;
			y_point+=ref_y;


			vector<double>  sd = getFrenet(x_point, y_point, ref_yaw, map_waypoints_x,map_waypoints_y);

			next_s_vals.push_back(sd[0]);
			next_d_vals.push_back(sd[1]);


			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);

			//vector<double> xy=transform_local_to_global(x_point,y_point,ref_x,ref_y,ref_yaw);

          		//next_x_vals.push_back(xy[0]);
          		//next_y_vals.push_back(xy[1]);
    		 }

			double car_acceleration=(car_speed -previous_car_speed) /sensing_period /2.2369; //mps
			double car_jerk= (car_acceleration -previous_car_acceleration) /sensing_period;
			previous_car_speed = car_speed;
			previous_car_acceleration= car_acceleration ;
                        time +=sensing_period;

    			double present_lane_front_dist = closestCarParams(end_path_s, lane, true, sensor_fusion)[0];
    			double present_lane_behind_dist = closestCarParams(end_path_s, lane , false, sensor_fusion)[0];
   			 double new_lane_front_dist = closestCarParams(end_path_s, lane + action , true, sensor_fusion)[0];
   			 double new_lane_behind_dist = closestCarParams(end_path_s, lane + action , false, sensor_fusion)[0];


		cout<< "time: " <<time <<"   lane : "<< lane <<"   too_close: "<<too_close<<"    laneChange action: "<<action <<"     car_jerk: "<<car_jerk<< "    car_acceleration: "<<car_acceleration << "   car_speed: "<<car_speed<<endl;
		//cout<< "present_lane_front_dist: " <<present_lane_front_dist<<"   new_lane_front_dist: "<<new_lane_front_dist <<"   present_lane_behind_dist : "<< present_lane_behind_dist <<"    new_lane_behind_dist: "<<new_lane_behind_dist<<endl;
		//out_log<< "time: " <<time <<"   lane : "<< lane <<"   too_close: "<<too_close<<"    laneChange action: "<<action <<"     car_jerk: "<<car_jerk<< "    car_acceleration: "<<car_acceleration << "   car_speed: "<<car_speed<<endl;
		//out_log<< "present_lane_front_dist: " <<present_lane_front_dist<<"   new_lane_front_dist: "<<new_lane_front_dist <<"   present_lane_behind_dist : "<< present_lane_behind_dist <<"    new_lane_behind_dist: "<<new_lane_behind_dist<<endl;


              // for(int i=0;i<25;i++)
			//out_log<<" previous_path_x.size():"<<previous_path_x.size()<< "  next_x_vals.size():" << next_x_vals.size() <<"  next_s_vals["<<i<<"]: "<<next_s_vals[i]  << "    next_d_vals["<<i<<"]: "<<next_d_vals[i] <<endl;  

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
