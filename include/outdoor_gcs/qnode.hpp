/**
 * @file /include/outdoor_gcs/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef outdoor_gcs_QNODE_HPP_
#define outdoor_gcs_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_event.h>
#include <ros/master.h>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#endif

#include <string>
#include <cmath>
#include <math.h>
// #include <unistd.h>
// #include <Eigen/Eigen>
#include <QThread>
#include <QStringListModel>


#include <outdoor_gcs/GPSRAW.h>
#include <outdoor_gcs/HomePosition.h> //mavros_msgs/HomePosition.h
#include <outdoor_gcs/ControlCommand.h>
#include <outdoor_gcs/Topic_for_log.h>
#include <outdoor_gcs/PathPlan.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RTCM.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

using State 	= mavros_msgs::State;
using Imu 		= sensor_msgs::Imu;
using Gpsraw 	= outdoor_gcs::GPSRAW;
using Gpsglobal = sensor_msgs::NavSatFix;
using Gpslocal 	= nav_msgs::Odometry;
using GpsHomePos = outdoor_gcs::HomePosition;
using PosTarg 	= mavros_msgs::PositionTarget;
using AltTarg 	= mavros_msgs::AttitudeTarget;
using RTCM 		= mavros_msgs::RTCM;

namespace outdoor_gcs {

/*****************************************************************************
** Class
*****************************************************************************/

	struct signalRec
	{
		bool prestate = false;
		bool preimu = false;
		bool pregps = false;
		bool pregpsG = false;
		bool pregpsL = false;
		bool pregpsH = false;
		bool stateReceived = false;
		bool imuReceived = false;
		bool gpsReceived = false;
		bool gpsGReceived = false;
		bool gpsLReceived = false;
		bool gpsHReceived = false;
	};

	struct Angles
	{
		float roll;
		float pitch;
		float yaw;
	};

	struct uav_info
	{
		int id = 0;
		float pos_cur[3] = {99};
		float vel_cur[3] = {0};
		float acc_cur[3] = {0};
		float ang_cur[3] = {0}; // roll, pitch, yaw
		float pos_des[3] = {0};
		float pos_ini[3] = {0};
		float pos_fin[3] = {0};
		float pos_nxt[3] = {0};
		bool rosReceived = false;
		bool stateReceived = false;
		bool imuReceived = false;
		bool gpsReceived = false;
		bool gpsLReceived = false;
		bool prerosReceived = false;
		bool prestateReceived = false;
		bool preimuReceived = false;
		bool pregpsReceived = false;
		bool pregpsLReceived = false;
		bool move = false;
		bool arrive = false;
	};

	struct checkbox_status
	{
		bool print_imu = true;
		bool print_state = true;
		bool print_gps = true;
		bool print_local = true;
		bool print_des = true;
		bool print_pathplan = false;
		bool clear_each_print = true;
		int uav_item = 1;
	};

	enum Command_Type
	{
		Idle,
		Takeoff,
		Move_ENU,
		Move_Body,
		Hold,
		Land,
		Disarm,
		Payload_Stabilization_SingleUAV,
		Trajectory_Tracking,
		Payload_Stabilization,
		Payload_Land,
	};


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	
	ros::master::V_TopicInfo topic_infos;

	////////////////////// Single uav ////////////////////////////
	void pub_command();
	
	void Set_Arm(bool arm_disarm);
	void Set_Mode(std::string command_mode);
	void Set_Home();
	void Set_GPS_Home();
	// void move_uav(float target[3], float target_yaw);
	void move_uav(bool mask[3], float target[4]);
	void move_uav_keyboard(bool mask[3], float target[4]);
	void move_uav_height(float height);

	State GetState();
	Imu GetImu();
	Gpsraw GetGPS();
	Gpsglobal GetGPSG();
	Gpslocal GetGPSL();
	GpsHomePos GetGPSH();
	sensor_msgs::BatteryState GetBat();
	mavros_msgs::Mavlink GetFrom();
	outdoor_gcs::signalRec Get_uav_signal();

	////////////////////// Multi-uav ////////////////////////////
	void uavs_call_service();
	void uavs_pub_command();
	void Set_Arm_uavs(bool arm_disarm, int ind);
	void Set_Mode_uavs(std::string command_mode, int ind);
	void Set_GPS_Home_uavs(int host_ind, int origin_ind);
	void Set_Square_Circle(int host_ind, float input[2]);
	void move_uavs(int ind, float pos_input[3]);
	void UAVS_Do_Plan();

	void Update_UAV_info(outdoor_gcs::uav_info UAV_input, int ind);
	void Update_Avail_UAVind(std::list<int> avail_uavind_input);
	void Update_RTCM(bool sent);
	void Update_Move(int i, bool move);
	void Update_Planning_Dim(int host_ind, int i);
	void Update_PathPlan();
	void Update_Flock_Param(float param[6]);
	void Update_ORCA_Param(float param[4]);
	void Update_PathPlan_Pos(int i, float pos_input[3], bool init_fin);
	void Update_PathPlan_Des(int i, bool init_fin);
	void Update_px4_apm(bool TF);

	State GetState_uavs(int ind);
	Imu GetImu_uavs(int ind);
	Gpsraw GetGPS_uavs(int ind);
	Gpslocal GetGPSL_uavs(int ind);
	mavros_msgs::Mavlink GetFrom_uavs(int ind);
	outdoor_gcs::uav_info Get_UAV_info(int ind);
	outdoor_gcs::Topic_for_log GetLog_uavs(int ind);
	float GetFlockParam(int i);
	float GetORCAParam(int i);

	QStringList lsAllTopics();
	outdoor_gcs::Angles quaternion_to_euler(float quat[4]);

Q_SIGNALS:
	void rosLoopUpdate();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	int freq = 4;

	////////////////////// Single uav ////////////////////////////
	mavros_msgs::State uav_state;
	Imu uav_imu;
	Gpsraw uav_gps;
	Gpsglobal uav_gpsG;
	Gpslocal uav_gpsL;
	GpsHomePos uav_gpsH;
	sensor_msgs::BatteryState uav_bat;
	mavros_msgs::Mavlink uav_from;
	mavros_msgs::CommandBool uav_arm;
	mavros_msgs::SetMode uav_setmode;
	mavros_msgs::CommandHome uav_sethome;

	PosTarg uav_setpoint;
	GpsHomePos uav_gps_home; //origin of gps local
	signalRec uav_received;

	ros::Subscriber uav_state_sub;
	ros::Subscriber uav_imu_sub;
	ros::Subscriber uav_gps_sub;
	ros::Subscriber uav_gpsG_sub;
	ros::Subscriber uav_gpsL_sub;
	ros::Subscriber uav_gpsH_sub;
	ros::Subscriber uav_bat_sub;
	ros::Subscriber uav_from_sub;

	ros::Publisher uav_setpoint_pub;
	ros::Publisher uav_gps_home_pub;

	ros::ServiceClient uav_arming_client;
	ros::ServiceClient uav_setmode_client;
	ros::ServiceClient uav_sethome_client;

	void state_callback(const mavros_msgs::State::ConstPtr &msg);
	void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
	void gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg);
	void gpsG_callback(const Gpsglobal::ConstPtr &msg);
	void gpsL_callback(const Gpslocal::ConstPtr &msg);
	void gpsH_callback(const GpsHomePos::ConstPtr &msg);
	void bat_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
	void from_callback(const mavros_msgs::Mavlink::ConstPtr &msg);

	////////////////////// Multi-uav ////////////////////////////
	float dt = 0.25;
	ros::Time last_change;
	
	int Plan_Dim[9]; // 0 for move wo planning, 2 for 2DFlock, 3 for 3DFlock, 4 9 for ORCA, 10 for square, 11 for circle
	
	// Flocking params
	// float c1 = 10.0; //7.0;
	// float c2 = 10.0; //9.0;
	// float RepulsiveGradient = 50; //7.5*std::pow(10,6);
	// float r_alpha = 3.0;
	// float max_acc = 10.0;
	// float max_vel = 10.0;
	
	float orca_param[4] = {5.0, 10.0, 1.0, 3.0}; // tau, max_v, r, Neighbor Dist
	float flock_param[6] = {10.0, 10.0, 50.0, 3.0, 10.0, 10.0}; // c1, c2, RepulsiveGradient, r_alpha, max_acc, max_vel

	// Square & circle 
	float sc_size;
	float sc_time;
	float sq_corners[9][5][2]; // 5 uavs, 4+1 conrners (extra one for coding simplicity purpose), 2D: xy
	float square_x[8] = {2, 2, 2, 0, -2, -2, -2, 0}; 
	float square_y[8] = {2, 0, -2, -2, -2, 0, 2, 2};
	int path_i = 0;
	float centers[9][3];
	bool start_path = false;
	bool pathplan = false;

	int DroneNumber = 9;
	outdoor_gcs::uav_info UAVs_info[9];
	std::list<int> avail_uavind;
	int service_flag[9];
	bool px4_apm = true; // true: px4, false: apm
	int apm_landtoff[9]; // 1: land, 2: takeoff
	bool pub_move_flag[9];
	bool pub_home_flag[9];
	bool pub_rtcm_flag;
	bool received_rtcm;
	bool Move[9]; // default false

    int comid = 1;

	ros::Subscriber ntrip_rtcm;
	ros::Subscriber uavs_state_sub[9];
	ros::Subscriber uavs_imu_sub[9];
	ros::Subscriber uavs_gps_sub[9];
	ros::Subscriber uavs_gpsG_sub[9];
	ros::Subscriber uavs_gpsL_sub[9];
	ros::Subscriber uavs_from_sub[9];
	ros::Subscriber uavs_log_sub[9];
	ros::Subscriber uavs_pathplan_sub;

	ros::Publisher uavs_setpoint_pub[9];
	ros::Publisher uavs_setpoint_alt_pub[9];	
	ros::Publisher uavs_gps_home_pub[9];
	ros::Publisher uavs_move_pub[9];
	ros::Publisher uavs_gps_rtcm_pub[9];
	ros::Publisher uavs_pathplan_pub;

	ros::ServiceClient uavs_arming_client[9];
	ros::ServiceClient uavs_setmode_client[9];

	// APM (ardupilot)
	ros::ServiceClient uavs_apm_land_client[9];
	ros::ServiceClient uavs_apm_toff_client[9];

	RTCM gps_rtcm;
	mavros_msgs::State uavs_state[9];
	Imu uavs_imu[9];
	Gpsraw uavs_gps[9];
	Gpsglobal uavs_gpsG[9];
	Gpslocal uavs_gpsL[9];
	mavros_msgs::Mavlink uavs_from[9];
	outdoor_gcs::Topic_for_log uavs_log[9];
	mavros_msgs::CommandBool uavs_arm[9];
	mavros_msgs::SetMode uavs_setmode[9];
	PosTarg uavs_setpoint[9];
	AltTarg uavs_setpoint_alt[9];
	mavros_msgs::CommandTOL uavs_apm_landtoff[9];
	GpsHomePos uavs_gps_home[9]; //origin of gps local
	outdoor_gcs::ControlCommand Command_List[9];
	RTCM uavs_gps_rtcm[9];
	outdoor_gcs::PathPlan uavs_pathplan;
	outdoor_gcs::PathPlan uavs_pathplan_nxt;

	void rtcm_callback(const RTCM::ConstPtr &msg);
	void uavs_state_callback(const mavros_msgs::State::ConstPtr &msg, int ind);
	void uavs_imu_callback(const sensor_msgs::Imu::ConstPtr &msg, int ind);
	void uavs_gps_callback(const outdoor_gcs::GPSRAW::ConstPtr &msg, int ind);
	void uavs_gpsG_callback(const Gpsglobal::ConstPtr &msg, int ind);
	void uavs_gpsL_callback(const Gpslocal::ConstPtr &msg, int ind);
	void uavs_from_callback(const mavros_msgs::Mavlink::ConstPtr &msg, int ind);
	void uavs_log_callback(const outdoor_gcs::Topic_for_log::ConstPtr &msg, int ind);
	void uavs_pathplan_callback(const outdoor_gcs::PathPlan::ConstPtr &msg);

};

}  // namespace outdoor_gcs

#endif /* outdoor_gcs_QNODE_HPP_ */
