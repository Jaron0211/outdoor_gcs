/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/outdoor_gcs/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace outdoor_gcs {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"outdoor_gcs");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	
	// uav_state_sub 	= n.subscribe<mavros_msgs::State>("/mavros/state", 1, &QNode::state_callback, this);
	uav_imu_sub 	= n.subscribe<Imu>("/mavros/imu/data", 1, &QNode::imu_callback, this);
	uav_gps_sub 	= n.subscribe<Gpsraw>("/mavros/gpsstatus/gps1/raw", 1, &QNode::gps_callback, this);
	uav_gpsG_sub 	= n.subscribe<Gpsglobal>("/mavros/global_position/global", 1, &QNode::gpsG_callback, this);
	uav_gpsL_sub 	= n.subscribe<Gpslocal>("/mavros/global_position/local", 1, &QNode::gpsL_callback, this);
	uav_gpsH_sub 	= n.subscribe<GpsHomePos>("/mavros/home_position/home", 1, &QNode::gpsH_callback, this);
	uav_bat_sub 	= n.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1, &QNode::bat_callback, this);
	uav_from_sub 	= n.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 1, &QNode::from_callback, this);

	uav_setpoint_pub = n.advertise<PosTarg>("/mavros/setpoint_raw/local", 1);
	uav_gps_home_pub = n.advertise<GpsHomePos>("/mavros/global_position/home", 1);

	uav_arming_client 	= n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	uav_setmode_client 	= n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	uav_sethome_client 	= n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

	ntrip_rtcm = n.subscribe<RTCM>("/rtcm", 1, &QNode::rtcm_callback, this);
	for (int i = 0; i < DroneNumber; i++) {
		// std::cout << "/uav" + std::to_string(i+1) + "/mavros/imu/data" << std::endl;
		// std::string name = "/uav" + std::to_string(i+1) + "/mavros/imu/data";
		uavs_state_sub[i]	= n.subscribe<mavros_msgs::State>("/uav" + std::to_string(i+1) + "/mavros/state", 1, std::bind(&QNode::uavs_state_callback, this, std::placeholders::_1, i));
		uavs_imu_sub[i] 	= n.subscribe<Imu>("/uav" + std::to_string(i+1) + "/mavros/imu/data", 1, std::bind(&QNode::uavs_imu_callback, this, std::placeholders::_1, i));
		uavs_gps_sub[i] 	= n.subscribe<Gpsraw>("/uav" + std::to_string(i+1) + "/mavros/gpsstatus/gps1/raw", 1, std::bind(&QNode::uavs_gps_callback, this, std::placeholders::_1, i));
		uavs_gpsG_sub[i] 	= n.subscribe<Gpsglobal>("/uav" + std::to_string(i+1) + "/mavros/global_position/global", 1, std::bind(&QNode::uavs_gpsG_callback, this, std::placeholders::_1, i));
		uavs_gpsL_sub[i] 	= n.subscribe<Gpslocal>("/uav" + std::to_string(i+1) + "/mavros/global_position/local", 1, std::bind(&QNode::uavs_gpsL_callback, this, std::placeholders::_1, i));
		uavs_from_sub[i] 	= n.subscribe<mavros_msgs::Mavlink>("/uav" + std::to_string(i+1) + "/mavlink/from", 1, std::bind(&QNode::uavs_from_callback, this, std::placeholders::_1, i));
		uavs_log_sub[i]		= n.subscribe<outdoor_gcs::Topic_for_log>("/uav" + std::to_string(i+1) + "/px4_command/topic_for_log", 1, std::bind(&QNode::uavs_log_callback, this, std::placeholders::_1, i));

		uavs_setpoint_pub[i] 		= n.advertise<PosTarg>("/uav" + std::to_string(i+1) + "/mavros/setpoint_raw/local", 1);
		uavs_setpoint_alt_pub[i] 	= n.advertise<AltTarg>("/uav" + std::to_string(i+1) + "/mavros/setpoint_raw/attitude", 1);
		uavs_gps_home_pub[i] 		= n.advertise<GpsHomePos>("/uav" + std::to_string(i+1) + "/mavros/global_position/home", 1);
        uavs_move_pub[i] 			= n.advertise<outdoor_gcs::ControlCommand>("/uav" + std::to_string(i+1) + "/px4_command/control_command", 1);
		uavs_gps_rtcm_pub[i] 		= n.advertise<RTCM>("/uav" + std::to_string(i+1) + "/mavros/gps_rtk/send_rtcm", 1);

		uavs_arming_client[i] 	= n.serviceClient<mavros_msgs::CommandBool>("/uav" + std::to_string(i+1) +"/mavros/cmd/arming");
		uavs_setmode_client[i] 	= n.serviceClient<mavros_msgs::SetMode>("/uav" + std::to_string(i+1) +"/mavros/set_mode");

		// APM
		uavs_apm_land_client[i] 	= n.serviceClient<mavros_msgs::CommandTOL>("/uav" + std::to_string(i+1) +"/mavros/cmd/land");
		uavs_apm_toff_client[i] 	= n.serviceClient<mavros_msgs::CommandTOL>("/uav" + std::to_string(i+1) +"/mavros/cmd/takeoff");

	}
	uavs_pathplan_sub = n.subscribe<outdoor_gcs::PathPlan>("/uavs/pathplan_nxt",1, &QNode::uavs_pathplan_callback, this);
	uavs_pathplan_pub = n.advertise<outdoor_gcs::PathPlan>("/uavs/pathplan",1);
	last_change = ros::Time::now();

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(freq); // change update rate here

	while ( ros::ok() ) {

		pub_command();
		UAVS_Do_Plan(); // for multi-uav
		uavs_call_service(); // for multi-uav
		uavs_pub_command(); // for multi-uav
		ros::spinOnce();

		uav_received.stateReceived = false;
		uav_received.imuReceived = false;
		uav_received.gpsReceived = false;
		uav_received.gpsGReceived = false;
		uav_received.gpsLReceived = false;
		uav_received.gpsHReceived = false;
		if (uav_received.prestate){
			uav_received.stateReceived = true;
		}
		if (uav_received.preimu){
			uav_received.imuReceived = true;
		}
		if (uav_received.pregps){
			uav_received.gpsReceived = true;
		}
		if (uav_received.pregpsG){
			uav_received.gpsGReceived = true;
		}
		if (uav_received.pregpsL){
			uav_received.gpsLReceived = true;
		}
		if (uav_received.pregpsH){
			uav_received.gpsHReceived = true;
		}
		uav_received.prestate = false;
		uav_received.preimu = false;
		uav_received.pregps = false;
		uav_received.pregpsG = false;
		uav_received.pregpsL = false;
		uav_received.pregpsH = false;
		// std::cout << uav_received.stateReceived << std::endl;


		//////////////// Multi-uav /////////////////

    	for (const auto &i : avail_uavind){
			UAVs_info[i].stateReceived = false;
			UAVs_info[i].imuReceived = false;
			UAVs_info[i].gpsReceived = false;
			UAVs_info[i].gpsLReceived = false;
			if (UAVs_info[i].prestateReceived){
				UAVs_info[i].stateReceived = true;
			}
			if (UAVs_info[i].preimuReceived){
				UAVs_info[i].imuReceived = true;
			}
			if (UAVs_info[i].pregpsReceived){
				UAVs_info[i].gpsReceived = true;
			}
			if (UAVs_info[i].pregpsLReceived){
				UAVs_info[i].gpsLReceived = true;
			}
			UAVs_info[i].prestateReceived = false;
			UAVs_info[i].preimuReceived = false;
			UAVs_info[i].pregpsReceived = false;
			UAVs_info[i].pregpsLReceived = false;
		}

		/* signal a ros loop update  */
		Q_EMIT rosLoopUpdate();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

///////////// Single uav ///////////////////
void QNode::state_callback(const mavros_msgs::State::ConstPtr &msg){
	uav_state = *msg;
	uav_received.prestate = true;
}
void QNode::imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
	uav_imu = *msg;
	uav_received.preimu = true;
}
void QNode::gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg){
	uav_gps = *msg;
	uav_received.pregps = true;
}
void QNode::gpsG_callback(const Gpsglobal::ConstPtr &msg){
	uav_gpsG = *msg;
	uav_received.pregpsG = true;
}
void QNode::gpsL_callback(const Gpslocal::ConstPtr &msg){
	uav_gpsL = *msg;
	uav_received.pregpsL = true;
}
void QNode::gpsH_callback(const GpsHomePos::ConstPtr &msg){
	uav_gpsH = *msg;
	uav_received.pregpsH = true;
}
void QNode::bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg){
	uav_bat = *msg;
}
void QNode::from_callback(const mavros_msgs::Mavlink::ConstPtr &msg){
	uav_from = *msg;
}

void QNode::pub_command(){
	uav_setpoint.header.stamp = ros::Time::now();
	uav_setpoint_pub.publish(uav_setpoint);
}

void QNode::Set_Arm(bool arm_disarm){
	uav_arm.request.value = arm_disarm;
	uav_arming_client.call(uav_arm);
}
void QNode::Set_Mode(std::string command_mode){
	uav_setmode.request.custom_mode = command_mode;
	uav_setmode_client.call(uav_setmode);
	// std::cout << uav_setmode.response.mode_sent << std::endl;
}
void QNode::Set_Home(){
	uav_sethome.request.current_gps = true;
	// uav_sethome.request.yaw = 0.0;
	// uav_sethome.request.latitude = uav_gps.lat*1e-7;
	// uav_sethome.request.longitude = uav_gps.lon*1e-7;
	// uav_sethome.request.altitude = uav_gps.alt/1000.0;
	uav_sethome_client.call(uav_sethome);
}
void QNode::Set_GPS_Home(){
	uav_gps_home.geo.latitude  = uav_gpsG.latitude;
	uav_gps_home.geo.longitude = uav_gpsG.longitude;
	uav_gps_home.geo.altitude  = uav_gpsG.altitude;
	uav_gps_home.orientation.x = uav_imu.orientation.x;
	uav_gps_home.orientation.y = uav_imu.orientation.y;
	uav_gps_home.orientation.z = uav_imu.orientation.z;
	uav_gps_home.orientation.w = uav_imu.orientation.w;
	uav_gps_home_pub.publish(uav_gps_home);
}

// void QNode::move_uav(float target[3], float target_yaw){
// 	uav_setpoint.header.stamp = ros::Time::now();
// 	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
//     //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
//     //Bit 10 should set to 0, means is not force sp
//     uav_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
//     uav_setpoint.coordinate_frame = 1;
// 	uav_setpoint.position.x = target[0];
// 	uav_setpoint.position.y = target[1];
// 	uav_setpoint.position.z = target[2];
// 	uav_setpoint.yaw = target_yaw;
// }
void QNode::move_uav(bool mask[3], float target[4]){
	uav_setpoint.header.stamp = ros::Time::now();
    uav_setpoint.coordinate_frame = 1;
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
	if (mask[0]){
		uav_setpoint.type_mask = 0b100111111000;
		uav_setpoint.position.x = target[0];
		uav_setpoint.position.y = target[1];
		uav_setpoint.position.z = target[2];
		uav_setpoint.yaw = (target[3])/180*3.14157;
	}
	else if (mask[1]){
		uav_setpoint.type_mask = 0b010111000111;
		uav_setpoint.velocity.x = target[0];
		uav_setpoint.velocity.y = target[1];
		uav_setpoint.velocity.z = target[2];
		uav_setpoint.yaw_rate = (target[3])/180*3.14157;
	}
	else if (mask[2]){
		uav_setpoint.type_mask = 0b100000111111;
		uav_setpoint.acceleration_or_force.x = target[0];
		uav_setpoint.acceleration_or_force.y = target[1];
		uav_setpoint.acceleration_or_force.z = target[2];
	}
}

void QNode::move_uav_keyboard(bool mask[3], float target[4]){
	uav_setpoint.header.stamp = ros::Time::now();
    uav_setpoint.coordinate_frame = 1;
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
	if (mask[0]){
		uav_setpoint.type_mask = 0b010111111000;
		uav_setpoint.position.x = target[0];
		uav_setpoint.position.y = target[1];
		uav_setpoint.position.z = target[2];
		uav_setpoint.yaw_rate = (target[3])/180*3.14157;
	}
	else if (mask[1]){
		uav_setpoint.type_mask = 0b010111000111;
		uav_setpoint.velocity.x = target[0];
		uav_setpoint.velocity.y = target[1];
		uav_setpoint.velocity.z = target[2];
		uav_setpoint.yaw_rate = (target[3])/180*3.14157;
	}
	else if (mask[2]){
		uav_setpoint.type_mask = 0b100000111111;
		uav_setpoint.acceleration_or_force.x = target[0];
		uav_setpoint.acceleration_or_force.y = target[1];
		uav_setpoint.acceleration_or_force.z = target[2];
	}
}

void QNode::move_uav_height(float height){
	// uav_setpoint.header.stamp = ros::Time::now();
	//Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    uav_setpoint.type_mask = 0b110111111011;
    uav_setpoint.coordinate_frame = 1;
	uav_setpoint.position.z = height;
}

State QNode::GetState(){
	return uav_state;
}
Imu QNode::GetImu(){
	return uav_imu;
}
Gpsraw QNode::GetGPS(){
	return uav_gps;
}
Gpsglobal QNode::GetGPSG(){
	return uav_gpsG;
}
Gpslocal QNode::GetGPSL(){
	return uav_gpsL;
}
GpsHomePos QNode::GetGPSH(){
	return uav_gpsH;
}
sensor_msgs::BatteryState QNode::GetBat(){
	return uav_bat;
}
mavros_msgs::Mavlink QNode::GetFrom(){
	return uav_from;
}
outdoor_gcs::signalRec QNode::Get_uav_signal(){
	return uav_received;
}


////////////////////////////////////////////// Multi-uav ///////////////////////////////////////////////////

void QNode::uavs_call_service(){
	for (const auto &ind : avail_uavind){
		if (service_flag[ind] == 1){ // arm or disarm
			uavs_arming_client[ind].call(uavs_arm[ind]);
			service_flag[ind] = 0;
		}
		else if (service_flag[ind] == 2){ // set mode (AUTO.TAKEOFF, AUTO.LAUBD, OFFBOARD ... etc)
			if (!px4_apm && apm_landtoff[ind]!=0){
				if (apm_landtoff[ind] == 1){
					uavs_apm_land_client[ind].call(uavs_apm_landtoff[ind]); 
				} else if (apm_landtoff[ind] == 2){
					uavs_apm_toff_client[ind].call(uavs_apm_landtoff[ind]); 
				}
			} else{
				uavs_setmode_client[ind].call(uavs_setmode[ind]);
			}
			service_flag[ind] = 0;
		}
	}
}

void QNode::uavs_pub_command(){
	bool pathplan_flag = false;
	for (const auto &ind : avail_uavind){
		if (pub_home_flag[ind]){ // gps set origin
			uavs_gps_home_pub[ind].publish(uavs_gps_home[ind]);
			pub_home_flag[ind] = false;
		}
		if (pub_move_flag[ind]){
			// uavs_setpoint_pub[ind].publish(uavs_setpoint[ind]);
        	uavs_move_pub[ind].publish(Command_List[ind]);
			pub_move_flag[ind] = false;
		}
		if (received_rtcm && pub_rtcm_flag){
			uavs_gps_rtcm[ind].data = gps_rtcm.data;
    		uavs_gps_rtcm[ind].header.stamp = ros::Time::now();
			uavs_gps_rtcm[ind].header.seq += 1;
        	uavs_gps_rtcm_pub[ind].publish(uavs_gps_rtcm[ind]);
		}
		if (pathplan){
			pathplan_flag=true;
		}
	}
	if (pathplan_flag){	
		Update_PathPlan();
		uavs_pathplan_pub.publish(uavs_pathplan); 
	}
	received_rtcm = false;
}
void QNode::rtcm_callback(const RTCM::ConstPtr &msg){
	gps_rtcm = *msg;
	received_rtcm = true;
}
void QNode::uavs_state_callback(const mavros_msgs::State::ConstPtr &msg, int ind){
	uavs_state[ind] = *msg;
	UAVs_info[ind].prestateReceived = true;
}
void QNode::uavs_imu_callback(const sensor_msgs::Imu::ConstPtr &msg, int ind){
	uavs_imu[ind] = *msg;
	UAVs_info[ind].preimuReceived = true;
	UAVs_info[ind].acc_cur[0] = uavs_imu[ind].linear_acceleration.x;
	UAVs_info[ind].acc_cur[1] = uavs_imu[ind].linear_acceleration.y;
	UAVs_info[ind].acc_cur[2] = uavs_imu[ind].linear_acceleration.z;
	float quat[4] = {uavs_imu[ind].orientation.w, uavs_imu[ind].orientation.x, uavs_imu[ind].orientation.y, uavs_imu[ind].orientation.z};
	outdoor_gcs::Angles uav_euler = quaternion_to_euler(quat);
	UAVs_info[ind].ang_cur[0] = uav_euler.roll*180/3.14159;
	UAVs_info[ind].ang_cur[1] = uav_euler.pitch*180/3.14159;
	UAVs_info[ind].ang_cur[2] = uav_euler.yaw*180/3.14159;
}
void QNode::uavs_gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg, int ind){
	uavs_gps[ind] = *msg;
	UAVs_info[ind].pregpsReceived = true;
}
void QNode::uavs_gpsG_callback(const Gpsglobal::ConstPtr &msg, int ind){
	uavs_gpsG[ind] = *msg;
}
void QNode::uavs_gpsL_callback(const Gpslocal::ConstPtr &msg, int ind){
	uavs_gpsL[ind] = *msg;
	UAVs_info[ind].pregpsLReceived = true;
	UAVs_info[ind].pos_cur[0] = uavs_gpsL[ind].pose.pose.position.x;
	UAVs_info[ind].pos_cur[1] = uavs_gpsL[ind].pose.pose.position.y;
	UAVs_info[ind].pos_cur[2] = uavs_gpsL[ind].pose.pose.position.z;
	UAVs_info[ind].vel_cur[0] = uavs_gpsL[ind].twist.twist.linear.x;
	UAVs_info[ind].vel_cur[1] = uavs_gpsL[ind].twist.twist.linear.y;
	UAVs_info[ind].vel_cur[2] = -uavs_gpsL[ind].twist.twist.linear.z; //Somehow z-velocity is in opposite direction
}
void QNode::uavs_from_callback(const mavros_msgs::Mavlink::ConstPtr &msg, int ind){
	uavs_from[ind] = *msg;
	UAVs_info[ind].id = uavs_from[ind].sysid;
}

void QNode::uavs_log_callback(const outdoor_gcs::Topic_for_log::ConstPtr &msg, int ind){
	uavs_log[ind] = *msg;
}
void QNode::uavs_pathplan_callback(const outdoor_gcs::PathPlan::ConstPtr &msg){
	uavs_pathplan_nxt = *msg;
	for (const auto &ind : avail_uavind){
		if (pathplan){
			UAVs_info[ind].pos_nxt[0] = uavs_pathplan_nxt.nxt_position[ind*3+0];
			UAVs_info[ind].pos_nxt[1] = uavs_pathplan_nxt.nxt_position[ind*3+1];
			UAVs_info[ind].pos_nxt[2] = uavs_pathplan_nxt.nxt_position[ind*3+2];
		}
		if (Plan_Dim[ind] == 4 || Plan_Dim[ind] == 6){ // 2D ORCA & 2D DW
			UAVs_info[ind].pos_nxt[2] = UAVs_info[ind].pos_des[2];
		}
	}
	uavs_pathplan.start = uavs_pathplan_nxt.start;
}

void QNode::Set_Arm_uavs(bool arm_disarm, int ind){
	uavs_arm[ind].request.value = arm_disarm;
	service_flag[ind] = 1;
}

void QNode::Set_Mode_uavs(std::string command_mode, int ind){
	uavs_setmode[ind].request.custom_mode = command_mode;
	if (!px4_apm){
		apm_landtoff[ind] = 0;
		if (command_mode == "AUTO.LAND"){
			apm_landtoff[ind] = 1;
			uavs_apm_landtoff[ind].request.min_pitch = 0.0;
			uavs_apm_landtoff[ind].request.yaw = 0.0;
			uavs_apm_landtoff[ind].request.latitude = uavs_gpsG[ind].latitude;
			uavs_apm_landtoff[ind].request.longitude = uavs_gpsG[ind].longitude;
			uavs_apm_landtoff[ind].request.altitude = 0.0;
		} else if (command_mode == "AUTO.TAKEOFF"){
			apm_landtoff[ind] = 2;
			uavs_apm_landtoff[ind].request.min_pitch = 0.0;
			uavs_apm_landtoff[ind].request.yaw = 0.0;
			uavs_apm_landtoff[ind].request.latitude = uavs_gpsG[ind].latitude;
			uavs_apm_landtoff[ind].request.longitude = uavs_gpsG[ind].longitude;
			uavs_apm_landtoff[ind].request.altitude = uavs_gpsG[ind].altitude + 2.5;
		} else if (command_mode == "OFFBOARD"){
			uavs_setmode[ind].request.custom_mode = "GUIDED";
		}
	}
	service_flag[ind] = 2;
}
void QNode::Set_GPS_Home_uavs(int host_ind, int origin_ind){
	uavs_gps_home[host_ind].geo.latitude  = uavs_gpsG[origin_ind].latitude;
	uavs_gps_home[host_ind].geo.longitude = uavs_gpsG[origin_ind].longitude;
	pub_home_flag[host_ind] = true;
}

// void QNode::move_uavs(int ind, float pos_input[3]){
// 	uavs_setpoint[ind].header.stamp = ros::Time::now();
//     uavs_setpoint[ind].type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
//     uavs_setpoint[ind].coordinate_frame = 1;
// 	uavs_setpoint[ind].position.x = pos_input[0];
// 	uavs_setpoint[ind].position.y = pos_input[1];
// 	uavs_setpoint[ind].position.z = pos_input[2];
// 	uavs_setpoint[ind].yaw = 0.0;
// 	publish_flag[ind] = 2;
// 	// Set_Mode_uavs("OFFBOARD", ind);
// }

void QNode::Set_Square_Circle(int host_ind, float input[2]){
	path_i = 0;
	sc_size = input[0]; // length of square or diameter of circle
	sc_time = input[1]; // time to finish one cycle
	centers[host_ind][0] = UAVs_info[host_ind].pos_des[0];
	centers[host_ind][1] = UAVs_info[host_ind].pos_des[1];
	centers[host_ind][2] = UAVs_info[host_ind].pos_des[2];
	sq_corners[host_ind][0][0] = sc_size/2+centers[host_ind][0];
	sq_corners[host_ind][0][1] = sc_size/2+centers[host_ind][1];
	sq_corners[host_ind][1][0] = -sc_size/2+centers[host_ind][0];
	sq_corners[host_ind][1][1] = sc_size/2+centers[host_ind][1];
	sq_corners[host_ind][2][0] = -sc_size/2+centers[host_ind][0];
	sq_corners[host_ind][2][1] = -sc_size/2+centers[host_ind][1];
	sq_corners[host_ind][3][0] = sc_size/2+centers[host_ind][0];
	sq_corners[host_ind][3][1] = -sc_size/2+centers[host_ind][1];
	// Extra corner for coding simplicity (to close up the loop)
	sq_corners[host_ind][4][0] = sc_size/2+centers[host_ind][0]; 
	sq_corners[host_ind][4][1] = sc_size/2+centers[host_ind][1];
}

void QNode::move_uavs(int ID, float pos_input[3]) {
	pub_move_flag[ID] = true;
    Command_List[ID].header.stamp = ros::Time::now();
    Command_List[ID].Mode = Move_ENU;

	Command_List[ID].Reference_State.Sub_mode  = 0;
	Command_List[ID].Reference_State.position_ref[0] = pos_input[0];
	Command_List[ID].Reference_State.position_ref[1] = pos_input[1];
	Command_List[ID].Reference_State.position_ref[2] = pos_input[2];
	Command_List[ID].Reference_State.velocity_ref[0] = 0;
	Command_List[ID].Reference_State.velocity_ref[1] = 0;
	Command_List[ID].Reference_State.velocity_ref[2] = 0;
    Command_List[ID].Reference_State.acceleration_ref[0] = 0;
    Command_List[ID].Reference_State.acceleration_ref[1] = 0;
    Command_List[ID].Reference_State.acceleration_ref[2] = 0;

    Command_List[ID].Reference_State.yaw_ref = 0;
    Command_List[ID].Command_ID = comid;
    comid++;
	// ROS_INFO("Sent");
}


void QNode::UAVS_Do_Plan(){
	for (const auto &host_ind : avail_uavind){

		float dist[3];
		dist[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_cur[0];
		dist[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_cur[1];
		dist[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_cur[2];
		if (sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2))<0.25){
			UAVs_info[host_ind].arrive = true;
		} else{ UAVs_info[host_ind].arrive = false; }

		if (!Move[host_ind]){ continue; }
		else{
			if (Plan_Dim[host_ind] == 0){
				move_uavs(host_ind, UAVs_info[host_ind].pos_des);
			}
			else if (Plan_Dim[host_ind] == 2){ // 2D Flock
				float force[2];
				force[0] = -flock_param[0]*(UAVs_info[host_ind].vel_cur[0])-flock_param[1]*(UAVs_info[host_ind].pos_cur[0]-UAVs_info[host_ind].pos_des[0]);
				force[1] = -flock_param[0]*(UAVs_info[host_ind].vel_cur[1])-flock_param[1]*(UAVs_info[host_ind].pos_cur[1]-UAVs_info[host_ind].pos_des[1]);
				for (const auto &other_ind : avail_uavind){
					float dist_v[2] = {	UAVs_info[host_ind].pos_cur[0]-UAVs_info[other_ind].pos_cur[0],
										UAVs_info[host_ind].pos_cur[1]-UAVs_info[other_ind].pos_cur[1]};
					float dist = std::pow(std::pow(dist_v[0],2) + std::pow(dist_v[1], 2), 0.5);
					if (host_ind != other_ind && dist < flock_param[3]){
						float ForceComponent = flock_param[2]*std::pow(dist - flock_param[3], 2);
						force[0] += ForceComponent*(dist_v[0]/dist);
						force[1] += ForceComponent*(dist_v[1]/dist);
					}
				}
				for (int i = 0; i < 2; i++) {
					force[i] = std::min(std::max(force[i], -flock_param[4]), flock_param[4]);
					float vel = std::min(std::max(UAVs_info[host_ind].vel_cur[i] + force[i]*dt, -flock_param[5]), flock_param[5]);
					UAVs_info[host_ind].pos_nxt[i] = UAVs_info[host_ind].pos_cur[i] + vel*dt;
					// std::cout << pos_input[i] << std::endl;
					// std::cout << UAVs_info[host_ind].pos_des[i] << std::endl;
				}
				UAVs_info[host_ind].pos_nxt[2] = UAVs_info[host_ind].pos_des[2];
				move_uavs(host_ind, UAVs_info[host_ind].pos_nxt);
			}
			else if (Plan_Dim[host_ind] == 3){ // 3D Flock
				float force[3];
				force[0] = -flock_param[0]*(UAVs_info[host_ind].vel_cur[0])-flock_param[1]*(UAVs_info[host_ind].pos_cur[0]-UAVs_info[host_ind].pos_des[0]);
				force[1] = -flock_param[0]*(UAVs_info[host_ind].vel_cur[1])-flock_param[1]*(UAVs_info[host_ind].pos_cur[1]-UAVs_info[host_ind].pos_des[1]);
				force[2] = -flock_param[0]*(UAVs_info[host_ind].vel_cur[2])-flock_param[1]*(UAVs_info[host_ind].pos_cur[2]-UAVs_info[host_ind].pos_des[2]);
				for (const auto &other_ind : avail_uavind){
					float dist_v[3] = {	UAVs_info[host_ind].pos_cur[0]-UAVs_info[other_ind].pos_cur[0],
										UAVs_info[host_ind].pos_cur[1]-UAVs_info[other_ind].pos_cur[1],
										UAVs_info[host_ind].pos_cur[2]-UAVs_info[other_ind].pos_cur[2]};
					float dist = std::pow(std::pow(dist_v[0],2) + std::pow(dist_v[1], 2) + std::pow(dist_v[2], 2), 0.5);
					if (host_ind != other_ind && dist < flock_param[3]){
						float ForceComponent = flock_param[2]*std::pow(dist - flock_param[3], 2);
						force[0] += ForceComponent*(dist_v[0]/dist);
						force[1] += ForceComponent*(dist_v[1]/dist);
						force[2] += ForceComponent*(dist_v[2]/dist);
					}
				}
				for (int i = 0; i < 3; i++) {
					force[i] = std::min(std::max(force[i], -flock_param[4]), flock_param[4]);
					float vel = std::min(std::max(UAVs_info[host_ind].vel_cur[i] + force[i]*dt, -flock_param[5]), flock_param[5]);
					UAVs_info[host_ind].pos_nxt[i] = UAVs_info[host_ind].pos_cur[i] + vel*dt;
					// std::cout << pos_input[i] << std::endl;
					// std::cout << UAVs_info[host_ind].pos_des[i] << std::endl;
				}
				move_uavs(host_ind, UAVs_info[host_ind].pos_nxt);
			}
			else if (pathplan){ //2D & 3D ORCA & DW Flock
				if (UAVs_info[host_ind].pos_nxt[0]!=0 && UAVs_info[host_ind].pos_nxt[1]!=0 && UAVs_info[host_ind].pos_nxt[2]!=0){
					move_uavs(host_ind, UAVs_info[host_ind].pos_nxt);
				}
			}
			else if (Plan_Dim[host_ind] == 10){ //Square path
				if (sc_time == 0){
					// Setting based on the location
					if (path_i >= 4){ path_i = 0; }
					UAVs_info[host_ind].pos_des[0] = sq_corners[host_ind][path_i][0];
					UAVs_info[host_ind].pos_des[1] = sq_corners[host_ind][path_i][1];
					UAVs_info[host_ind].pos_des[2] = centers[host_ind][2];
					move_uavs(host_ind, UAVs_info[host_ind].pos_des);

					float dist[3];
					dist[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_cur[0];
					dist[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_cur[1];
					dist[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_cur[2];
					if (sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2))<0.25){
					// if (ros::Time::now() - last_change >= ros::Duration(sc_time/4)){
						path_i += 1;
						// last_change = ros::Time::now();
					}
				} else{
					// Setting based on given time
					if (path_i >= sc_time*freq){ path_i = 0; }
					int turn = std::floor(path_i/(sc_time*freq/4.0)); // (sc_time*freq)/4 amount of i for one side
					UAVs_info[host_ind].pos_des[0] = sq_corners[host_ind][turn][0]+(sq_corners[host_ind][turn+1][0] - sq_corners[host_ind][turn][0])*(path_i%int(sc_time*freq/4.0))/(sc_time*freq/4.0);
					UAVs_info[host_ind].pos_des[1] = sq_corners[host_ind][turn][1]+(sq_corners[host_ind][turn+1][1] - sq_corners[host_ind][turn][1])*(path_i%int(sc_time*freq/4.0))/(sc_time*freq/4.0);
					UAVs_info[host_ind].pos_des[2] = centers[host_ind][2];
					move_uavs(host_ind, UAVs_info[host_ind].pos_des);

					float dist[3];
					dist[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_cur[0];
					dist[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_cur[1];
					dist[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_cur[2];
					if (!start_path && sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2))<0.25){
						start_path = true;
					}
					if (start_path){
						path_i += 1;
					}
				}
			}
			else if (Plan_Dim[host_ind] == 11){ // circle path
				if (sc_time == 0){
					// Setting based on the location (set 36 points! 1 per 10 degree.)
					if (path_i >= 36){ path_i = 0; }
					float theta = 2*M_PI*10*path_i/360;
					UAVs_info[host_ind].pos_des[0] = (sc_size/2)*cos(theta)+centers[host_ind][0];
					UAVs_info[host_ind].pos_des[1] = (sc_size/2)*sin(theta)+centers[host_ind][0];
					UAVs_info[host_ind].pos_des[2] = centers[host_ind][2];
					move_uavs(host_ind, UAVs_info[host_ind].pos_des);

					float dist[3];
					dist[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_cur[0];
					dist[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_cur[1];
					dist[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_cur[2];
					if (sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2))<0.25){
						path_i += 1;
					}
				} else{
					// Setting based on given time
					if (path_i >= sc_time*freq){ path_i = 0; }
					float theta = 2*M_PI*path_i/(sc_time*freq);
					UAVs_info[host_ind].pos_des[0] = (sc_size/2)*cos(theta)+centers[host_ind][0];
					UAVs_info[host_ind].pos_des[1] = (sc_size/2)*sin(theta)+centers[host_ind][0];
					UAVs_info[host_ind].pos_des[2] = centers[host_ind][2];
					move_uavs(host_ind, UAVs_info[host_ind].pos_des);

					float dist[3];
					dist[0] = UAVs_info[host_ind].pos_des[0] - UAVs_info[host_ind].pos_cur[0];
					dist[1] = UAVs_info[host_ind].pos_des[1] - UAVs_info[host_ind].pos_cur[1];
					dist[2] = UAVs_info[host_ind].pos_des[2] - UAVs_info[host_ind].pos_cur[2];
					if (!start_path && sqrt(pow(dist[0],2)+pow(dist[1],2)+pow(dist[2],2))<0.25){
						start_path = true;
					}
					if (start_path){
						path_i += 1;
					}
				}
			}
		}
	}	
}

void QNode::Update_UAV_info(outdoor_gcs::uav_info UAV_input, int ind){
	UAVs_info[ind] = UAV_input;
}
void QNode::Update_Avail_UAVind(std::list<int> avail_uavind_input){
	avail_uavind = avail_uavind_input;
}
void QNode::Update_RTCM(bool sent){
	pub_rtcm_flag = sent;
}
void QNode::Update_Move(int i, bool move){
	Move[i] = move;
	UAVs_info[i].move = move;
}
void QNode::Update_Planning_Dim(int host_ind, int i){
	// 0 for no planning, 2/3 for 2D/3D flock, 4/5 for 2D/3D ORCA, 6/7 for 2D/3D DW Flock, 10 for square, 11 for circle
	if (host_ind==99){
    	for (const auto &it : avail_uavind){
			Plan_Dim[it] = i;
		}
	} else{ Plan_Dim[host_ind] = i;}
	
	start_path = false;
	// start_path = true;
	if (i==4 || i==5 || i==6 || i==7){
		pathplan = true;
		uavs_pathplan.start = true;
	} else{
		pathplan = false;
	}
}
void QNode::Update_PathPlan(){
	uavs_pathplan.header.stamp = ros::Time::now();
	for (const auto &it : avail_uavind){
		uavs_pathplan.uavs_id[it] = true;
		uavs_pathplan.cur_position[3*it+0] = UAVs_info[it].pos_cur[0];
		uavs_pathplan.cur_position[3*it+1] = UAVs_info[it].pos_cur[1];
		uavs_pathplan.cur_position[3*it+2] = UAVs_info[it].pos_cur[2];
		uavs_pathplan.des_position[3*it+0] = UAVs_info[it].pos_des[0];
		uavs_pathplan.des_position[3*it+1] = UAVs_info[it].pos_des[1];
		uavs_pathplan.des_position[3*it+2] = UAVs_info[it].pos_des[2];
		uavs_pathplan.nxt_position[3*it+0] = UAVs_info[it].pos_nxt[0];
		uavs_pathplan.nxt_position[3*it+1] = UAVs_info[it].pos_nxt[1];
		uavs_pathplan.nxt_position[3*it+2] = UAVs_info[it].pos_nxt[2];
		uavs_pathplan.cur_velocity[3*it+0] = UAVs_info[it].vel_cur[0];
		uavs_pathplan.cur_velocity[3*it+1] = UAVs_info[it].vel_cur[1];
		uavs_pathplan.cur_velocity[3*it+2] = UAVs_info[it].vel_cur[2];
		if (Plan_Dim[it] == 4){ // 2D ORCA, assume uavs at same height of 3.0
			uavs_pathplan.cur_position[3*it+2] = 3.0;
		}
	}
	// for (int i = 0; i < 4; i++) {
	// 	uavs_pathplan.params[i] = orca_param[i];
	// }
	uavs_pathplan.num = avail_uavind.size();
}
void QNode::Update_Flock_Param(float param[6]){
	// if (param[0] != 0){	c1=param[0]; }
	// if (param[1] != 0){	c2=param[1]; }
	// if (param[2] != 0){	RepulsiveGradient=param[2]; }
	// if (param[3] != 0){	r_alpha=param[3]; }
	// if (param[4] != 0){	max_acc=param[4]; }
	// if (param[5] != 0){	max_vel=param[5]; }
	for (int i = 0; i < 6; i++) {
		if (param[i] != 0){
			uavs_pathplan.params[i] = param[i];
			flock_param[i] = param[i];
		}
	}
}
void QNode::Update_ORCA_Param(float param[4]){
	for (int i = 0; i < 4; i++) {
		if (param[i] != 0){
			uavs_pathplan.params[i] = param[i];
			orca_param[i] = param[i];
		}
	}
}
void QNode::Update_PathPlan_Pos(int i, float pos_input[3], bool init_fin){ //True for init, false for final pos
	if (init_fin){
		UAVs_info[i].pos_ini[0] = pos_input[0];
		UAVs_info[i].pos_ini[1] = pos_input[1];
		UAVs_info[i].pos_ini[2] = pos_input[2];
	} else {
		UAVs_info[i].pos_fin[0] = pos_input[0];
		UAVs_info[i].pos_fin[1] = pos_input[1];
		UAVs_info[i].pos_fin[2] = pos_input[2];
	}
}
void QNode::Update_PathPlan_Des(int i, bool init_fin){ //True for init, false for final pos
	if (init_fin){
		UAVs_info[i].pos_des[0] = UAVs_info[i].pos_ini[0];
		UAVs_info[i].pos_des[1] = UAVs_info[i].pos_ini[1];
		UAVs_info[i].pos_des[2] = UAVs_info[i].pos_ini[2];
	} else {
		UAVs_info[i].pos_des[0] = UAVs_info[i].pos_fin[0];
		UAVs_info[i].pos_des[1] = UAVs_info[i].pos_fin[1];
		UAVs_info[i].pos_des[2] = UAVs_info[i].pos_fin[2];
	}
}
void QNode::Update_px4_apm(bool TF){
	px4_apm = TF;
}

State QNode::GetState_uavs(int ind){
	return uavs_state[ind];
}
Imu QNode::GetImu_uavs(int ind){
	// std::cout << "Pass data" << std::endl;
	return uavs_imu[ind];
}
Gpsraw QNode::GetGPS_uavs(int ind){
	return uavs_gps[ind];
}
Gpslocal QNode::GetGPSL_uavs(int ind){
	return uavs_gpsL[ind];
}
mavros_msgs::Mavlink QNode::GetFrom_uavs(int ind){
	return uavs_from[ind];
}
outdoor_gcs::uav_info QNode::Get_UAV_info(int ind){
	return UAVs_info[ind];
}
outdoor_gcs::Topic_for_log  QNode::GetLog_uavs(int ind){
	return uavs_log[ind];
}
float QNode::GetFlockParam(int i){
	return flock_param[i];

	// if (i == 0){return c1;}
	// else if (i == 1){return c2;}
	// else if (i == 2){return RepulsiveGradient;}
	// else if (i == 3){return r_alpha;}
	// else if (i == 4){return max_acc;}
	// else if (i == 5){return max_vel;}
}
float QNode::GetORCAParam(int i){
	return orca_param[i];
}


QStringList QNode::lsAllTopics(){
	ros::master::getTopics(topic_infos);
	QStringList topic_names;
	for (const auto &it : topic_infos)
	{
		topic_names += QString::fromStdString(it.name);
	}
	return topic_names;
}

outdoor_gcs::Angles QNode::quaternion_to_euler(float quat[4]){
    outdoor_gcs::Angles ans;
    ans.roll = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans.pitch = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans.yaw = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}


}  // namespace outdoor_gcs
