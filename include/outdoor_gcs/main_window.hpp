/**
 * @file /include/outdoor_gcs/main_window.hpp
 *
 * @brief Qt based gui for outdoor_gcs.
 *
 * @date November 2010
 **/
#ifndef outdoor_gcs_MAIN_WINDOW_H
#define outdoor_gcs_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <QMainWindow>
#include <QKeyEvent>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace outdoor_gcs {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	QStringList all_topics;

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	////////////////////// Single-uav ////////////////////////////
	void on_ARM_clicked(bool check);
	void on_SET_HOME_clicked(bool check);
	void on_TAKEOFF_clicked(bool check);
	void on_LAND_clicked(bool check);
	void on_MODE_RTL_clicked(bool check);
	void on_MODE_LOITER_clicked(bool check);
	void on_MODE_MANUAL_clicked(bool check);
	void on_MODE_POSCTL_clicked(bool check);
	void on_MODE_OFFBOARD_clicked(bool check);
	void on_Set_GPS_Home_clicked(bool check);
	void on_Enable_Planning_clicked(bool check);
	void on_Button_KeyBoard_mode_clicked(bool check);
	void on_Button_Set_Pos_clicked(bool check);
	void on_Button_Set_Vel_clicked(bool check);
	void on_Button_Set_Acc_clicked(bool check);
	void on_Button_Set_H_clicked(bool check);
	void on_Button_Get_clicked(bool check);

	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);

	////////////////////// Multi-uav ////////////////////////////
	void on_Update_UAV_List_clicked(bool check);
	void on_px4_apm_clicked(bool check);
	void on_Set_GPS_Origin_clicked(bool check);
	void on_Button_GetCur_Select_clicked(bool check);
	void on_Button_GetDes_Select_clicked(bool check);
	void on_Button_Set_All_Height_clicked(bool check);
	void on_Button_Set_Select_clicked(bool check);
	void on_ARM_ONE_clicked(bool check);
	void on_DISARM_ONE_clicked(bool check);
	void on_TAKEOFF_ONE_clicked(bool check);
	void on_LAND_ONE_clicked(bool check);
	void on_MODE_RTL_ONE_clicked(bool check);
	void on_MODE_LOITER_ONE_clicked(bool check);
	void on_MODE_OFFBOARD_ONE_clicked(bool check);
	void on_Button_Move_One_clicked(bool check);
	void on_Button_Stop_One_clicked(bool check);
	void on_ARM_ALL_clicked(bool check);
	void on_DISARM_ALL_clicked(bool check);
	void on_TAKEOFF_ALL_clicked(bool check);
	void on_LAND_ALL_clicked(bool check);
	void on_Button_Move_All_clicked(bool check);
	void on_Button_Stop_All_clicked(bool check);
	void on_MODE_RTL_ALL_clicked(bool check);
	void on_MODE_LOITER_ALL_clicked(bool check);
	void on_MODE_POSCTL_ALL_clicked(bool check);
	void on_MODE_OFFBOARD_ALL_clicked(bool check);
	void on_Button_Flock_Param_clicked(bool check);
	void on_Button_ORCA_Param_clicked(bool check);
	void on_Button_SetInit_clicked(bool check);
	void on_Button_SetFin_clicked(bool check);
	void on_Button_GoInit_clicked(bool check);
	void on_Button_GoFin_clicked(bool check);
	void on_Button_GoInit_ALL_clicked(bool check);
	void on_Button_GoFin_ALL_clicked(bool check);
	void on_Button_uavitem_clicked(bool check);
	// void on_InfoLogger_Clear_clicked(bool check);

	void on_checkBox_rtcm_stateChanged(int);
	void on_checkBox_square_stateChanged(int);
	void on_checkBox_circle_stateChanged(int);
	void on_checkBox_Flock_2D_stateChanged(int);
	void on_checkBox_Flock_3D_stateChanged(int);
	void on_checkBox_Flock_DW2_stateChanged(int);
	void on_checkBox_Flock_DW3_stateChanged(int);
	void on_checkBox_ORCA_2D_stateChanged(int);
	void on_checkBox_ORCA_3D_stateChanged(int);
	void on_checkBox_PPPrint_stateChanged(int);
	void on_checkBox_imu_stateChanged(int);
	void on_checkBox_mode_stateChanged(int);
	void on_checkBox_gps_stateChanged(int);
	void on_checkBox_local_stateChanged(int);
	void on_checkBox_des_stateChanged(int);
	void on_checkBox_clear_stateChanged(int);


	void on_Rostopic_Update_clicked(bool check);

    /******************************************
    ** Manual connections
    *******************************************/
	void updateuav();
	// void updateTopics();
	void updateuavs();
	void updateInfoLogger();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	////////////////////// Single-uav ////////////////////////////
	bool uav_ARMED;
	bool Planning_Enabled = false;
	bool KeyBoard_mode = false;
	float KeyPress_target[4] = { 0, 0, 0, 0};
	bool Keyboard_input = false;
	ros::Time last_change;
	float square_x[8] = {2, 2, 2, 0, -2, -2, -2, 0}; 
	float square_y[8] = {2, 0, -2, -2, -2, 0, 2, 2};
	int square_i = 0;

	////////////////////// Multi-uav ////////////////////////////
	int DroneNumber = 9;
	std::list<int> avail_uavind;
	int origin_ind;
	// bool continue_offboard;
	bool px4_apm = true; // true: px4, false: apm
	int init_fin = 0; // 0:none, 1: go init pos, 2: go fin pos
	bool all_arrive = false;
	ros::Time start_time;
	ros::Time arrive_time;

	outdoor_gcs::uav_info UAVs[9];
	QStringList UAV_Detected;
	QStringList UAV_Info_Logger;
	outdoor_gcs::checkbox_status checkbox_stat;
};

}  // namespace outdoor_gcs

#endif // outdoor_gcs_MAIN_WINDOW_H
