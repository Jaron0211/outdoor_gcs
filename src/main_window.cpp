/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/outdoor_gcs/main_window.hpp"
// #include "../include/outdoor_gcs/math_utils.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace outdoor_gcs {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	// ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    /*------------init ros node -----------*/
    bool init_ros_ok = qnode.init();
    if (!init_ros_ok)
    {
        showNoMasterMessage();
    }
    else
    {
        //ui.button_connect->setEnabled(false);
    }

	/*********************
	** Logging
	**********************/
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateuav()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateuavs()));
    QObject::connect(&qnode, SIGNAL(rosLoopUpdate()), this, SLOT(updateInfoLogger()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

////////////////////////// Single-uav Buttons /////////////////////////

void MainWindow::on_ARM_clicked(bool check){
	if (uav_ARMED){
		qnode.Set_Arm(false);
	}
	else{
		qnode.Set_Arm(true);
	}
}


void MainWindow::on_SET_HOME_clicked(bool check){
	qnode.Set_Home();
}

void MainWindow::on_TAKEOFF_clicked(bool check){
	qnode.Set_Mode("AUTO.TAKEOFF");
}
void MainWindow::on_LAND_clicked(bool check){
	qnode.Set_Mode("AUTO.LAND");
}
void MainWindow::on_MODE_RTL_clicked(bool check){
	qnode.Set_Mode("AUTO.RTL");
}
void MainWindow::on_MODE_LOITER_clicked(bool check){
	qnode.Set_Mode("AUTO.LOITER");
}
void MainWindow::on_MODE_MANUAL_clicked(bool check){
	qnode.Set_Mode("MANUAL");
}
void MainWindow::on_MODE_POSCTL_clicked(bool check){
	qnode.Set_Mode("POSCTL");
}
void MainWindow::on_MODE_OFFBOARD_clicked(bool check){
	qnode.Set_Mode("OFFBOARD");
}

void MainWindow::on_Set_GPS_Home_clicked(bool check){
	qnode.Set_GPS_Home();
}

void MainWindow::on_Enable_Planning_clicked(bool check){
	if (Planning_Enabled){
        Planning_Enabled = false;
    } else{
        Planning_Enabled = true;
    }
}

void MainWindow::on_Button_Set_Pos_clicked(bool check){
    /* read values from line edit */
    float target_state[4];
    target_state[0] =  ui.x_input->text().toFloat();
    target_state[1] =  ui.y_input->text().toFloat();
    target_state[2] =  ui.z_input->text().toFloat();
    target_state[3] =  ui.w_input->text().toFloat();
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;
    // if(target_state[0] < -10.0 || target_state[0] > 10.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[1] < -10.0 || target_state[1] > 10.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[2] < 0|| target_state[2] > 30.0) {
    //     input_is_valid = false;
    // }


    /*----------------send input ------------------*/
    if(input_is_valid){
        /*  update the ENU target label */
        ui.des_x->setText(QString::number(target_state[0], 'f', 2));
        ui.des_y->setText(QString::number(target_state[1], 'f', 2));
        ui.des_z->setText(QString::number(target_state[2], 'f', 2));
        bool mask[3] = { true, false, false};
        qnode.move_uav(mask, target_state);
    	qnode.Set_Mode("OFFBOARD");

    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Button_Set_Vel_clicked(bool check){
    /* read values from line edit */
    float target_state[3];

    target_state[0] =  ui.vx_input->text().toFloat();
    target_state[1] =  ui.vy_input->text().toFloat();
    target_state[2] =  ui.vz_input->text().toFloat();
    target_state[3] =  ui.vw_input->text().toFloat();
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    // if(target_state[0] < -5.0 || target_state[0] > 5.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[1] < -5.0 || target_state[1] > 5.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[2] < -5.0 || target_state[2] > 5.0) {
    //     input_is_valid = false;
    // }

    /*----------------send input ------------------*/
    if(input_is_valid){
        /*  update the ENU target label */
        ui.des_vx->setText(QString::number(target_state[0], 'f', 2));
        ui.des_vy->setText(QString::number(target_state[1], 'f', 2));
        ui.des_vz->setText(QString::number(target_state[2], 'f', 2));
        bool mask[3] = {false, true, false};
        qnode.move_uav(mask, target_state);
    	qnode.Set_Mode("OFFBOARD");
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Button_Set_Acc_clicked(bool check){
    /* read values from line edit */
    float target_state[3];

    target_state[0] =  ui.ax_input->text().toFloat();
    target_state[1] =  ui.ay_input->text().toFloat();
    target_state[2] =  ui.az_input->text().toFloat();
    target_state[3] =  ui.aw_input->text().toFloat();
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    // if(target_state[0] < -5.0 || target_state[0] > 5.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[1] < -5.0 || target_state[1] > 5.0) {
    //     input_is_valid = false;
    // }
    // if(target_state[2] < -5.0 || target_state[2] > 5.0) {
    //     input_is_valid = false;
    // }


    /*----------------send input ------------------*/
    if(input_is_valid){
        /*  update the ENU target label */
        ui.des_ax->setText(QString::number(target_state[0], 'f', 2));
        ui.des_ay->setText(QString::number(target_state[1], 'f', 2));
        ui.des_az->setText(QString::number(target_state[2], 'f', 2));
        bool mask[3] = {false, false, true};
        qnode.move_uav(mask, target_state);
    	qnode.Set_Mode("OFFBOARD");
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Button_Set_H_clicked(bool check){
    /* read values from line edit */
    float target_height;
    target_height =  ui.z_input->text().toFloat();

    if (target_height > 0 && target_height < 30.0) {
        ui.des_z->setText(QString::number(target_height, 'f', 2));
        qnode.move_uav_height(target_height);
    	qnode.Set_Mode("OFFBOARD");
    } else {
        QMessageBox msgBox;
        msgBox.setText("Input position is out of range!!");
        msgBox.exec();
    };
}

void MainWindow::on_Button_Get_clicked(bool check){
	Gpslocal gpsL_data = qnode.GetGPSL();
    Imu imu_data = qnode.GetImu();
    float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
    outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
    ui.x_input->setText(QString::number(gpsL_data.pose.pose.position.x, 'f', 2));
    ui.y_input->setText(QString::number(gpsL_data.pose.pose.position.y, 'f', 2));
    ui.z_input->setText(QString::number(gpsL_data.pose.pose.position.z, 'f', 2));
    ui.w_input->setText(QString::number(uav_euler.yaw*180/3.14159, 'f', 2));
}

void MainWindow::on_Button_KeyBoard_mode_clicked(bool check){//wo
    if(KeyBoard_mode){
        KeyBoard_mode = false;
    }else{
        KeyBoard_mode = true;
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event){
    
    Keyboard_input = true;

    if(event->key() == Qt::Key_A)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[1] = abs(ui.fly_speed_input->text().toFloat());
    }
    
    if(event->key() == Qt::Key_D)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[1] = -abs(ui.fly_speed_input->text().toFloat());
    }

    if(event->key() == Qt::Key_W)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[0] = abs(ui.fly_speed_input->text().toFloat());
    }
    
    if(event->key() == Qt::Key_S)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[0] = -abs(ui.fly_speed_input->text().toFloat());
    }

    if(event->key() == Qt::Key_Q)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[3] = abs(ui.rotation_speed_input->text().toFloat());
    }
    
    if(event->key() == Qt::Key_E)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[3] = -abs(ui.rotation_speed_input->text().toFloat());
    }

    if(event->key() == Qt::Key_Z)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[2] = abs(ui.climb_speed_input->text().toFloat());
    }
    
    if(event->key() == Qt::Key_C)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[2] = -abs(ui.climb_speed_input->text().toFloat());
    }

    bool mask[3] = {false, true, false};
    qnode.move_uav_keyboard(mask, KeyPress_target);
    qnode.Set_Mode("OFFBOARD");
}

void MainWindow::keyReleaseEvent(QKeyEvent *event){

    if(event->key() == Qt::Key_A)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[1] = 0.0;
    }
    
    if(event->key() == Qt::Key_D)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[1] = -0.0;
    }

    if(event->key() == Qt::Key_W)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[0] = 0.0;
    }
    
    if(event->key() == Qt::Key_S)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[0] = -0.0;
    }

    if(event->key() == Qt::Key_Q)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[3] = 0;
    }
    
    if(event->key() == Qt::Key_E)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[3] = 0;
    }

    if(event->key() == Qt::Key_Z)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[2] = 0;
    }
    
    if(event->key() == Qt::Key_C)
    {
        if(event->isAutoRepeat()){
            return;
        }
        KeyPress_target[2] = 0;
    }

    bool mask[3] = {false, true, false};
    qnode.move_uav_keyboard(mask, KeyPress_target);
    qnode.Set_Mode("OFFBOARD");
}

////////////////////////// Update signals /////////////////////////
void MainWindow::updateuav(){

    if (ui.checkBox_square1 -> isChecked()){
        float pos_input[3];
        pos_input[0] = square_x[square_i];
        pos_input[1] = square_y[square_i];
        pos_input[2] = 2.5;
        bool mask[3] = { true, false, false};
        qnode.move_uav(mask, pos_input);
        qnode.Set_Mode("OFFBOARD");
        if (ros::Time::now() - last_change >= ros::Duration(5.0)){
            square_i += 1;
            if (square_i == 8){
                square_i = 0;
            }
            last_change = ros::Time::now();
        }
    }

	mavros_msgs::State state_data = qnode.GetState();
    Imu imu_data = qnode.GetImu();
    sensor_msgs::BatteryState bat_data = qnode.GetBat();
	mavros_msgs::Mavlink from_data = qnode.GetFrom();
	outdoor_gcs::GPSRAW gps_data = qnode.GetGPS();
	Gpsglobal gpsG_data = qnode.GetGPSG();
	Gpslocal gpsL_data = qnode.GetGPSL();
	GpsHomePos gpsH_data = qnode.GetGPSH();
    outdoor_gcs::signalRec signal = qnode.Get_uav_signal();

	if (signal.imuReceived){
        ui.IMU_CONNECT->setText("<font color='green'>IMU CONNECTED</font>");
        ui.CONNECT->setText("UAV CONNECTED: " + QString::number(from_data.sysid));
        ui.Volt->setText(QString::number(bat_data.voltage, 'f', 2));
        // Eigen::Quaterniond uav_quat = Eigen::Quaterniond(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z);
        // Eigen::Vector3d uav_euler = quaternion_to_euler(uav_quat); //Transform the Quaternion to euler Angles
        float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
        outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
        ui.roll->setText(QString::number(uav_euler.roll*180/3.14159, 'f', 2));
        ui.pitch->setText(QString::number(uav_euler.pitch*180/3.14159, 'f', 2));
        ui.yaw->setText(QString::number(uav_euler.yaw*180/3.14159, 'f', 2));

        if (state_data.connected){
            ui.STATE_CONNECT->setText("<font color='green'>STATE CONNECTED</font>");
        }
        else{
            ui.STATE_CONNECT->setText("<font color='red'>STATE UNCONNECTED</font>");
        }
        if (state_data.mode.empty()){
            ui.MODE->setText("<font color='red'>---</font>");
        }
        else{
		    ui.MODE->setText(QString::fromStdString(state_data.mode));
        }
		if (state_data.armed){
			uav_ARMED = true;
			ui.ARM->setText("DISARM");
		}
		else{
			uav_ARMED = false;
			ui.ARM->setText("ARM");
		}
	}else{
        ui.CONNECT->setText("<font color='red'>UAV UNCONNECTED</font>");
        ui.STATE_CONNECT->setText("<font color='red'>STATE UNCONNECTED</font>");
        ui.IMU_CONNECT->setText("<font color='red'>IMU UNCONNECTED</font>");
        ui.Volt->setText("<font color='red'>---</font>");
        ui.MODE->setText("<font color='red'>---</font>");
        ui.roll->setText("<font color='red'>---</font>");
        ui.pitch->setText("<font color='red'>---</font>");
        ui.yaw->setText("<font color='red'>---</font>");
	}

	if (signal.gpsReceived){
        ui.gps_num->setText(QString::number(gps_data.satellites_visible));
		ui.gps_lat->setText(QString::number(gps_data.lat*1e-7, 'f', 7));
		ui.gps_lon->setText(QString::number(gps_data.lon*1e-7, 'f', 7));
		ui.gps_alt->setText(QString::number(gps_data.alt*1e-3, 'f', 3));
	}else{
		ui.gps_num->setText("<font color='red'>---</font>");
        ui.gps_lat->setText("<font color='red'>---</font>");
        ui.gps_lon->setText("<font color='red'>---</font>");
        ui.gps_alt->setText("<font color='red'>---</font>");
	}

    if (signal.gpsGReceived){
		ui.gps_lat_2->setText(QString::number(gpsG_data.latitude, 'f', 6));
		ui.gps_lon_2->setText(QString::number(gpsG_data.longitude, 'f', 6));
		ui.gps_alt_2->setText(QString::number(gpsG_data.altitude, 'f', 6));
	}else{
        ui.gps_lat_2->setText("<font color='red'>---</font>");
        ui.gps_lon_2->setText("<font color='red'>---</font>");
        ui.gps_alt_2->setText("<font color='red'>---</font>");
	}

    if (signal.gpsLReceived){
		ui.localx->setText(QString::number(gpsL_data.pose.pose.position.x, 'f', 6));
		ui.localy->setText(QString::number(gpsL_data.pose.pose.position.y, 'f', 6));
		ui.localz->setText(QString::number(gpsL_data.pose.pose.position.z, 'f', 6));
        ui.localvx->setText(QString::number(gpsL_data.twist.twist.linear.x, 'f', 6));
		ui.localvy->setText(QString::number(gpsL_data.twist.twist.linear.y, 'f', 6));
		ui.localvz->setText(QString::number(gpsL_data.twist.twist.linear.z, 'f', 6));
	}else{
        ui.localx->setText("<font color='red'>---</font>");
        ui.localy->setText("<font color='red'>---</font>");
        ui.localz->setText("<font color='red'>---</font>");
        ui.localvx->setText("<font color='red'>---</font>");
        ui.localvy->setText("<font color='red'>---</font>");
        ui.localvz->setText("<font color='red'>---</font>");
        
	}

    if (signal.gpsHReceived){
		ui.localx_2->setText(QString::number(gpsH_data.position.x, 'f', 6));
		ui.localy_2->setText(QString::number(gpsH_data.position.y, 'f', 6));
		ui.localz_2->setText(QString::number(gpsH_data.position.z, 'f', 6));
	}else{
        ui.localx_2->setText("<font color='red'>---</font>");
        ui.localy_2->setText("<font color='red'>---</font>");
        ui.localz_2->setText("<font color='red'>---</font>");
	}

    if (Planning_Enabled){
        // qnode.Do_Plan();
		ui.Enable_Planning->setText("Disable Planning");
    }else{
		ui.Enable_Planning->setText("Enable_Planning");
    }

    if (KeyBoard_mode){

        ui.Button_KeyBoard_mode->setText("KeyBoard Input On");
        ui.Button_Get->setEnabled(false);
        ui.Button_Set_H->setEnabled(false);
        ui.Button_Set_Pos->setEnabled(false);
        ui.Button_Set_Vel->setEnabled(false);
        ui.Button_Set_Acc->setEnabled(false);
        ui.checkBox_square1->setChecked(false);
        ui.checkBox_square1->setEnabled(false);

        /*  update the ENU target label */
        ui.des_vx->setText(QString::number(KeyPress_target[0], 'f', 2));
        ui.des_vy->setText(QString::number(KeyPress_target[1], 'f', 2));
        ui.des_vz->setText(QString::number(KeyPress_target[2], 'f', 2));    

    }else{

        ui.Button_KeyBoard_mode->setText("KeyBoard Input Off");
        ui.Button_Get->setEnabled(true);
        ui.Button_Set_H->setEnabled(true);
        ui.Button_Set_Pos->setEnabled(true);
        ui.Button_Set_Vel->setEnabled(true);
        ui.Button_Set_Acc->setEnabled(true);
        ui.checkBox_square1->setEnabled(true);
    }

}


////////////////////////// Multi-uav button /////////////////////////

void MainWindow::on_Update_UAV_List_clicked(bool check){
    ui.uav_detect_logger->clear();
    all_topics = qnode.lsAllTopics();
    UAV_Detected.clear();
    avail_uavind.clear();
    for(int i = 0; i < DroneNumber ; i++) {
        UAVs[i].rosReceived = false;
        QString filter_word = "uav" + QString::number(i+1) + "/mavlink/from"; // use this topic as a filter to detect uav
        QStringList filtered_topics = all_topics.filter(filter_word);
        if (filtered_topics.count() != 0){
            UAV_Detected += "uav" + QString::number(i+1);
            UAVs[i].rosReceived = true;
            avail_uavind.push_back(i);
        }
        qnode.Update_UAV_info(UAVs[i], i);
    }
    qnode.Update_Avail_UAVind(avail_uavind);
    ui.uav_detect_logger->addItems(UAV_Detected);
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Available uav list updated!");
}

void MainWindow::on_px4_apm_clicked(bool check){
    if (px4_apm){
        px4_apm = false;
        qnode.Update_px4_apm(false);
        ui.px4_apm->setText("px4");
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : APM !!!! ");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    } else{
        px4_apm = true;
        qnode.Update_px4_apm(true);
        ui.px4_apm->setText("apm");
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : PX4 !!!! ");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    }
}

void MainWindow::on_Set_GPS_Origin_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                origin_ind = i;
                break;
            }
        }
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : uav " + QString::number(origin_ind+1) + " selected to set for origin!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
        for (const auto &i : avail_uavind){
            qnode.Set_GPS_Home_uavs(i, origin_ind);
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to set GPS origin!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_Button_GetCur_Select_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                ui.x_input_all->setText(QString::number(UAVs[i].pos_cur[0], 'f', 2));
                ui.y_input_all->setText(QString::number(UAVs[i].pos_cur[1], 'f', 2));
                ui.z_input_all->setText(QString::number(UAVs[i].pos_cur[2], 'f', 2));
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Got current location for uav " + QString::number(i+1) + "!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to get current location!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_Button_GetDes_Select_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                ui.x_input_all->setText(QString::number(UAVs[i].pos_des[0], 'f', 2));
                ui.y_input_all->setText(QString::number(UAVs[i].pos_des[1], 'f', 2));
                ui.z_input_all->setText(QString::number(UAVs[i].pos_des[2], 'f', 2));
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Got desired location for uav " + QString::number(i+1) + "!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to get desired location!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}


void MainWindow::on_Button_Set_All_Height_clicked(bool check){
    float target_height;
    target_height =  ui.z_input_all->text().toFloat();

    if (target_height > -1.2 && target_height < 30.0) {
        for (const auto &i : avail_uavind){
            UAVs[i].pos_des[0] = UAVs[i].pos_cur[0];
            UAVs[i].pos_des[1] = UAVs[i].pos_cur[1];
            UAVs[i].pos_des[2] = target_height;
            qnode.Update_UAV_info(UAVs[i], i);
        }
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired height of all uav is set to " + QString::number(target_height));
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    } else {
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Input height is out of range!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    };
}

void MainWindow::on_Button_Set_Select_clicked(bool check){
    /* read values from line edit */
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();

        float target_state[3];
        target_state[0] =  ui.x_input_all->text().toFloat();
        target_state[1] =  ui.y_input_all->text().toFloat();
        target_state[2] =  ui.z_input_all->text().toFloat();
        /*----------------determine whether the input is in safe range ------------------*/
        bool input_is_valid = true;
        if(target_state[0] < -20.0 || target_state[0] > 20.0) {
            input_is_valid = false;
        }
        if(target_state[1] < -20.0 || target_state[1] > 20.0) {
            input_is_valid = false;
        }
        if(target_state[2] < -1.20 || target_state[2] > 30.0) {
            input_is_valid = false;
        }

        /*----------------send input ------------------*/
        if(input_is_valid){
            for (const auto &i : avail_uavind){
                if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                    UAVs[i].pos_des[0] = target_state[0];
                    UAVs[i].pos_des[1] = target_state[1];
                    UAVs[i].pos_des[2] = target_state[2];
                    qnode.Update_UAV_info(UAVs[i], i);
                    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired location of uav " + QString::number(i+1) + " is set! ");
                    int item_index = ui.notice_logger->count()-1;
                    ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
                    break;
                }
            }
        } else {
            ui.notice_logger->addItem(QTime::currentTime().toString() + " : Input location is out of range!");
            int item_index = ui.notice_logger->count()-1;
            ui.notice_logger->item(item_index)->setForeground(Qt::red);
        };

    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to assign desired location!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_ARM_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                qnode.Set_Arm_uavs(true, i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to ARM is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to arm!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_DISARM_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                qnode.Set_Arm_uavs(false, i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to DISARM is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to disarm!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_TAKEOFF_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Set_Mode_uavs("AUTO.TAKEOFF", i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to TAKEOFF is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to takeoff!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_LAND_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Set_Mode_uavs("AUTO.LAND", i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to LAND is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to land!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_MODE_RTL_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Set_Mode_uavs("AUTO.RTL", i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to RTL is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to rtl!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_MODE_LOITER_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Set_Mode_uavs("AUTO.LOITER", i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to LOITER is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to loiter!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_MODE_OFFBOARD_ONE_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Set_Mode_uavs("OFFBOARD", i);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to set OFFBOARD is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to set offboard!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_Button_Move_One_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Update_Move(i, true);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to MOVE is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to move!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_Button_Stop_One_clicked(bool check){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
        	    qnode.Update_Move(i, false);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Command for uav " + QString::number(i+1) + " to STOP is sent!");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to stop!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}

void MainWindow::on_ARM_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
        qnode.Set_Arm_uavs(true, i);
        // sleep(1.0);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : ARM ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_DISARM_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
        qnode.Set_Arm_uavs(false, i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : DISARM ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_TAKEOFF_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("AUTO.TAKEOFF", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : TAKEOFF ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_LAND_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("AUTO.LAND", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : LAND ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_Button_Move_All_clicked(bool check){
    for (const auto &i : avail_uavind){
        qnode.Update_Move(i, true);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : MOVE ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_Button_Stop_All_clicked(bool check){
    for (const auto &i : avail_uavind){
        qnode.Update_Move(i, false);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : STOP ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_MODE_RTL_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("AUTO.RTL", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : RTL ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_MODE_LOITER_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("AUTO.LOITER", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : LOITER ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_MODE_POSCTL_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("POSCTL", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : POSCTL ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_MODE_OFFBOARD_ALL_clicked(bool check){
    for (const auto &i : avail_uavind){
	    qnode.Set_Mode_uavs("OFFBOARD", i);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : OFFBOARD ALL uavs command sent!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_Button_Flock_Param_clicked(bool check){
    float param[6];
    param[0] = ui.c1_input->text().toFloat();
    param[1] = ui.c2_input->text().toFloat();
    param[2] = ui.rho_input->text().toFloat();
    param[3] = ui.r_alpha_input->text().toFloat();
    param[4] = ui.acc_input->text().toFloat();
    param[5] = ui.vel_input->text().toFloat();
    qnode.Update_Flock_Param(param);
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Flock params are updated!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_Button_ORCA_Param_clicked(bool check){
    float param[4];
    param[0] = ui.tau_input->text().toFloat();
    param[1] = ui.pvel_input->text().toFloat();
    param[2] = ui.r_input->text().toFloat();
    param[3] = ui.NDist_input->text().toFloat();
    qnode.Update_ORCA_Param(param);
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : ORCA params are updated!");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
}
void MainWindow::on_Button_SetInit_clicked(bool check){
    /* read values from line edit */
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();

        float pathplan_pos[3];
        pathplan_pos[0] =  ui.x_input_all->text().toFloat();
        pathplan_pos[1] =  ui.y_input_all->text().toFloat();
        pathplan_pos[2] =  ui.z_input_all->text().toFloat();
        /*----------------determine whether the input is in safe range ------------------*/
        bool input_is_valid = true;
        if(pathplan_pos[0] < -20.0 || pathplan_pos[0] > 20.0) {
            input_is_valid = false;
        }
        if(pathplan_pos[1] < -20.0 || pathplan_pos[1] > 20.0) {
            input_is_valid = false;
        }
        if(pathplan_pos[2] < -1.20 || pathplan_pos[2] > 30.0) {
            input_is_valid = false;
        }

        /*----------------send input ------------------*/
        if(input_is_valid){
            for (const auto &i : avail_uavind){
                if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                    qnode.Update_PathPlan_Pos(i, pathplan_pos, true);
                    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Init of uav " + QString::number(i+1) + " is set! ");
                    int item_index = ui.notice_logger->count()-1;
                    ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
                    break;
                }
            }
        } else {
            ui.notice_logger->addItem(QTime::currentTime().toString() + " : Input location is out of range!");
            int item_index = ui.notice_logger->count()-1;
            ui.notice_logger->item(item_index)->setForeground(Qt::red);
        };

    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to assign initial location!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_Button_SetFin_clicked(bool check){
    /* read values from line edit */
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();

        float pathplan_pos[3];
        pathplan_pos[0] =  ui.x_input_all->text().toFloat();
        pathplan_pos[1] =  ui.y_input_all->text().toFloat();
        pathplan_pos[2] =  ui.z_input_all->text().toFloat();
        /*----------------determine whether the input is in safe range ------------------*/
        bool input_is_valid = true;
        if(pathplan_pos[0] < -20.0 || pathplan_pos[0] > 20.0) {
            input_is_valid = false;
        }
        if(pathplan_pos[1] < -20.0 || pathplan_pos[1] > 20.0) {
            input_is_valid = false;
        }
        if(pathplan_pos[2] < -1.20 || pathplan_pos[2] > 30.0) {
            input_is_valid = false;
        }

        /*----------------send input ------------------*/
        if(input_is_valid){
            for (const auto &i : avail_uavind){
                if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                    qnode.Update_PathPlan_Pos(i, pathplan_pos, false);
                    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Fin of uav " + QString::number(i+1) + " is set! ");
                    int item_index = ui.notice_logger->count()-1;
                    ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
                    break;
                }
            }
        } else {
            ui.notice_logger->addItem(QTime::currentTime().toString() + " : Input location is out of range!");
            int item_index = ui.notice_logger->count()-1;
            ui.notice_logger->item(item_index)->setForeground(Qt::red);
        };

    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to assign final location!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_Button_GoInit_clicked(bool check){
    all_arrive = false;
    start_time = ros::Time::now();
    init_fin = 1;
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                qnode.Update_PathPlan_Des(i, true);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired location of uav " + QString::number(i+1) + " is set to Init! ");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to GoInit!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_Button_GoFin_clicked(bool check){
    all_arrive = false;
    start_time = ros::Time::now();
    init_fin = 2;
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                qnode.Update_PathPlan_Des(i, false);
                ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired location of uav " + QString::number(i+1) + " is set to Fin! ");
                int item_index = ui.notice_logger->count()-1;
                ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to GoInit!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_Button_GoInit_ALL_clicked(bool check){
    all_arrive = false;
    start_time = ros::Time::now();
    init_fin = 1;
    for (const auto &i : avail_uavind){
        qnode.Update_PathPlan_Des(i, true);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired location of all uavs is set to Init! ");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
}
void MainWindow::on_Button_GoFin_ALL_clicked(bool check){
    all_arrive = false;
    start_time = ros::Time::now();
    init_fin = 2;
    for (const auto &i : avail_uavind){
        qnode.Update_PathPlan_Des(i, false);
    }
    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Desired location of all uavs is set to Fin! ");
    int item_index = ui.notice_logger->count()-1;
    ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
}
void MainWindow::on_Button_uavitem_clicked(bool check){
    if (checkbox_stat.uav_item == 1){
        checkbox_stat.uav_item = 2;
        ui.Button_uavitem->setText("uav");
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Print by item! ");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    } else{
        checkbox_stat.uav_item = 1;
        ui.Button_uavitem->setText("item");
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Print by uav! ");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    }
}


// void MainWindow::on_InfoLogger_Clear_clicked(bool check){
//     ui.info_logger->clear();
// }

// CheckBox //
void MainWindow::on_checkBox_rtcm_stateChanged(int){
    if (ui.checkBox_rtcm -> isChecked()){ 
        qnode.Update_RTCM(true);
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Send RTCM signal!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::darkGreen);
    } else{ 
        qnode.Update_RTCM(false);
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Stop sending RTCM signal!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_checkBox_square_stateChanged(int){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                if (ui.checkBox_square -> isChecked()){
                    ui.checkBox_circle -> setChecked(false);
                    float size_time[2];
                    size_time[0] =  ui.size_input->text().toFloat();
                    size_time[1] =  ui.time_input->text().toFloat();
                    qnode.Set_Square_Circle(i, size_time);
                    qnode.Update_Planning_Dim(i, 10);
                    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Square path of uav " + QString::number(i+1) + " is set!");
                    int item_index = ui.notice_logger->count()-1;
                    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                }
                else{
                    qnode.Update_Planning_Dim(i, 0);
                }
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to assign path!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_checkBox_circle_stateChanged(int){
    if (ui.uav_detect_logger->selectedItems().count()!=0){
        QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
        for (const auto &i : avail_uavind){
            if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
                if (ui.checkBox_circle -> isChecked()){
                    ui.checkBox_square -> setChecked(false);
                    float size_time[2];
                    size_time[0] =  ui.size_input->text().toFloat();
                    size_time[1] =  ui.time_input->text().toFloat();
                    qnode.Set_Square_Circle(i, size_time);
                    qnode.Update_Planning_Dim(i, 11);
                    ui.notice_logger->addItem(QTime::currentTime().toString() + " : Circle path of uav " + QString::number(i+1) + " is set!");
                    int item_index = ui.notice_logger->count()-1;
                    ui.notice_logger->item(item_index)->setForeground(Qt::blue);
                }
                else{
                    qnode.Update_Planning_Dim(i, 0);
                }
                break;
            }
        }
    } else{
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please select an uav to assign path!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
}
void MainWindow::on_checkBox_Flock_2D_stateChanged(int){
    if (ui.checkBox_Flock_2D -> isChecked()){
        ui.checkBox_Flock_3D -> setChecked(false);
        ui.checkBox_ORCA_2D -> setChecked(false);
        ui.checkBox_ORCA_3D -> setChecked(false);
        qnode.Update_Planning_Dim(99, 2); // 99 as all agents
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : 2D Flock Planning Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_Flock_3D_stateChanged(int){
    if (ui.checkBox_Flock_3D -> isChecked()){
        ui.checkBox_Flock_2D -> setChecked(false);
        ui.checkBox_ORCA_2D -> setChecked(false);
        ui.checkBox_ORCA_3D -> setChecked(false);
        qnode.Update_Planning_Dim(99, 3);
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : 3D Flock Planning Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_Flock_DW2_stateChanged(int){
    if (ui.checkBox_Flock_DW2 -> isChecked()){
        ui.checkBox_Flock_2D -> setChecked(false);
        ui.checkBox_Flock_3D -> setChecked(false);
        ui.checkBox_Flock_DW3 -> setChecked(false);
        ui.checkBox_ORCA_2D -> setChecked(false);
        ui.checkBox_ORCA_3D -> setChecked(false);
        qnode.Update_Planning_Dim(99, 6);
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Flock Planning with Downwash 2D Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_Flock_DW3_stateChanged(int){
    if (ui.checkBox_Flock_DW3 -> isChecked()){
        ui.checkBox_Flock_2D -> setChecked(false);
        ui.checkBox_Flock_3D -> setChecked(false);
        ui.checkBox_Flock_DW2 -> setChecked(false);
        ui.checkBox_ORCA_2D -> setChecked(false);
        ui.checkBox_ORCA_3D -> setChecked(false);
        qnode.Update_Planning_Dim(99, 7);
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Flock Planning with Downwash 3D Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_ORCA_2D_stateChanged(int){
    if (ui.checkBox_ORCA_2D -> isChecked()){
        ui.checkBox_Flock_2D -> setChecked(false);
        ui.checkBox_Flock_3D -> setChecked(false);
        ui.checkBox_ORCA_3D -> setChecked(false);
        // ui.checkBox_Plan_ -> setChecked(false);
        qnode.Update_Planning_Dim(99, 4); // 99 as all agents
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : 2D ORCA Planning Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_ORCA_3D_stateChanged(int){
    if (ui.checkBox_ORCA_3D -> isChecked()){
        ui.checkBox_Flock_2D -> setChecked(false);
        ui.checkBox_Flock_3D -> setChecked(false);
        ui.checkBox_ORCA_2D -> setChecked(false);
        qnode.Update_Planning_Dim(99, 5); // 99 as all agents
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : 3D ORCA Planning Set!");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::blue);
    }else{
        qnode.Update_Planning_Dim(99, 0);
    }
}
void MainWindow::on_checkBox_PPPrint_stateChanged(int){
    if (ui.checkBox_PPPrint -> isChecked()){
        checkbox_stat.print_pathplan = true;
    }else{
        checkbox_stat.print_pathplan = false;
    }
}
void MainWindow::on_checkBox_imu_stateChanged(int){
    if (ui.checkBox_imu -> isChecked()){
        checkbox_stat.print_imu = true;
    }else{
        checkbox_stat.print_imu = false;
    }
}
void MainWindow::on_checkBox_mode_stateChanged(int){
    if (ui.checkBox_mode -> isChecked()){
        checkbox_stat.print_state = true;
    }else{
        checkbox_stat.print_state = false;
    }
}
void MainWindow::on_checkBox_gps_stateChanged(int){
    if (ui.checkBox_gps -> isChecked()){
        checkbox_stat.print_gps = true;
    }else{
        checkbox_stat.print_gps = false;
    }
}
void MainWindow::on_checkBox_local_stateChanged(int){
    if (ui.checkBox_local -> isChecked()){
        checkbox_stat.print_local = true;
    }else{
        checkbox_stat.print_local = false;
    }
}
void MainWindow::on_checkBox_des_stateChanged(int){
    if (ui.checkBox_des -> isChecked()){
        checkbox_stat.print_des = true;
    }else{
        checkbox_stat.print_des = false;
    }
}
void MainWindow::on_checkBox_clear_stateChanged(int){
    if (ui.checkBox_clear -> isChecked()){
        checkbox_stat.clear_each_print = true;
    }else{
        checkbox_stat.clear_each_print = false;
    }
}


////////////////////////// Rostopic button /////////////////////////
void MainWindow::on_Rostopic_Update_clicked(bool check){
    ui.rostopic_logger->clear();
    all_topics = qnode.lsAllTopics();
    QString filter_word = ui.topic_filter->text();
    QStringList filtered_topics = all_topics.filter(filter_word);
    ui.rostopic_logger->addItems(filtered_topics);
    ui.rostopic_count->setText("Count: " +QString::number(filtered_topics.count()));
}


////////////////////////// Update signals /////////////////////////
void MainWindow::updateuavs(){
    if (avail_uavind.size()==0){
        ui.notice_logger->clear();
        ui.notice_logger->addItem(QTime::currentTime().toString() + " : Please update available uavs");
        int item_index = ui.notice_logger->count()-1;
        ui.notice_logger->item(item_index)->setForeground(Qt::red);
    }
    ui.notice_logger->scrollToBottom();

    all_arrive = true;
    for (const auto &i : avail_uavind){
        UAVs[i] = qnode.Get_UAV_info(i);
        if (!UAVs[i].arrive){ all_arrive = false; }
        // if (ui.checkBox_Offboard -> isChecked()){
	    //     qnode.Set_Mode_uavs("OFFBOARD", i);
        // }
    }
    if (!all_arrive){ arrive_time = ros::Time::now(); }
}

void MainWindow::updateInfoLogger(){
    if (checkbox_stat.clear_each_print){
        ui.info_logger->clear();
    }
    ui.info_logger->addItem(QTime::currentTime().toString());
    int item_index = ui.info_logger->count()-1;
    ui.info_logger->item(item_index)->setForeground(Qt::white);
    ui.info_logger->item(item_index)->setBackground(Qt::black);
    if (checkbox_stat.print_pathplan){
        QString word_to_be_print = "Go ";
        if (init_fin == 1){ word_to_be_print += "Init! "; } 
        else if (init_fin == 2){ word_to_be_print += "Final! "; } 
        else {word_to_be_print += "-- ";}
        if (all_arrive) {word_to_be_print += "Arrived! "; }
        else {word_to_be_print += "On the way! "; }
        word_to_be_print += "Time: ";
        word_to_be_print += QString::number(arrive_time.toSec() - start_time.toSec(), 'f', 1);
        ui.info_logger->addItem(word_to_be_print);

        ui.info_logger->addItem("Flock Param: c1: " + QString::number(qnode.GetFlockParam(0), 'f', 1) +
                                ", c2: " + QString::number(qnode.GetFlockParam(1), 'f', 1) + 
                                ", rho: " + QString::number(qnode.GetFlockParam(2), 'f', 1));
        ui.info_logger->addItem("             r_alpha: " + QString::number(qnode.GetFlockParam(3), 'f', 1) +
                                ", a_max: " + QString::number(qnode.GetFlockParam(4), 'f', 1) +
                                ", v_max: " + QString::number(qnode.GetFlockParam(5), 'f', 1));
        ui.info_logger->addItem("ORCA Param: tau: " + QString::number(qnode.GetORCAParam(0), 'f', 1) +
                                ", pref_v: " + QString::number(qnode.GetORCAParam(1), 'f', 1) +
                                ", r: " + QString::number(qnode.GetORCAParam(2), 'f', 1) +
                                ", NDist: " + QString::number(qnode.GetORCAParam(3), 'f', 1));
        ui.info_logger->addItem("----------------------------------------------------------------------------------------");
    }

    if (checkbox_stat.uav_item == 1){ // Print by uav

        for (const auto &it : avail_uavind){

            if (checkbox_stat.print_imu || checkbox_stat.print_state || checkbox_stat.print_gps ||
                checkbox_stat.print_local || checkbox_stat.print_des){
                ui.info_logger->addItem("uav " + QString::number(it+1) + " (id = " + QString::number(UAVs[it].id) + ") info: ");
                int item_index = ui.info_logger->count()-1;
                ui.info_logger->item(item_index)->setForeground(Qt::blue);
            } else{ continue; }

            if (checkbox_stat.print_imu){
                // Imu imu_data = qnode.GetImu_uavs(it);
                // float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
                // outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
                ui.info_logger->addItem("Roll: " + QString::number(UAVs[it].ang_cur[0], 'f', 2) + ". Pitch: " + 
                                QString::number(UAVs[it].ang_cur[1], 'f', 2) + ". Yaw: " + QString::number(UAVs[it].ang_cur[2], 'f', 2));
                if (!UAVs[it].imuReceived){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            if (checkbox_stat.print_state){
                mavros_msgs::State state_data = qnode.GetState_uavs(it);
                QString state_to_be_print = "State: ";
                if (state_data.connected){
                    state_to_be_print += "Connected! ";
                } else{
                    state_to_be_print += "UNConnected! ";
                }
                if (state_data.mode.empty()){
                    state_to_be_print += "Mode Empty! ";
                } else{
                    state_to_be_print += QString::fromStdString(state_data.mode) + "! ";
                }
                if (state_data.armed){
                    state_to_be_print += "ARMED!";
                } else{
                    state_to_be_print += "DISARMED!";
                }
                ui.info_logger->addItem(state_to_be_print);
                if (!state_data.connected){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            if (checkbox_stat.print_gps){
                outdoor_gcs::GPSRAW gps_data = qnode.GetGPS_uavs(it);
                ui.info_logger->addItem("Num: " + QString::number(gps_data.satellites_visible) + ". Lat: " + QString::number(gps_data.lat*1e-7, 'f', 7) +
                                        ". Lon: " + QString::number(gps_data.lon*1e-7, 'f', 7) + ". Alt: " + QString::number(gps_data.alt*1e-3, 'f', 3));
                if (!UAVs[it].gpsReceived){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            if (checkbox_stat.print_local){
                ui.info_logger->addItem("Local Position: X: " + QString::number(UAVs[it].pos_cur[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_cur[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_cur[2], 'f', 3));
                ui.info_logger->addItem("Local Velocity: X: " + QString::number(UAVs[it].vel_cur[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].vel_cur[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].vel_cur[2], 'f', 3));
                if (!UAVs[it].gpsLReceived){
                    int item_index = ui.info_logger->count()-1;
                    int item_index2 = ui.info_logger->count()-2;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                    ui.info_logger->item(item_index2)->setForeground(Qt::red);
                }
            }
            if (checkbox_stat.print_des){
                outdoor_gcs::Topic_for_log log = qnode.GetLog_uavs(it);
                ui.info_logger->addItem("Throttle: X: " + QString::number(log.Control_Output.Throttle[0], 'f', 3) +
                                        ". Y: " + QString::number(log.Control_Output.Throttle[1], 'f', 3) + 
                                        ". Z: " + QString::number(log.Control_Output.Throttle[2], 'f', 3));
                ui.info_logger->addItem("Desired Position: X: " + QString::number(UAVs[it].pos_des[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_des[1], 'f', 3) + 
                                        ". Z: " + QString::number(UAVs[it].pos_des[2], 'f', 3));
                if(UAVs[it].move){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::darkGreen);
                }
            }
            if (checkbox_stat.print_pathplan){
                ui.info_logger->addItem("PP Init Position: X: " + QString::number(UAVs[it].pos_ini[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_ini[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_ini[2], 'f', 3));
                ui.info_logger->addItem("PP Next Position: X: " + QString::number(UAVs[it].pos_nxt[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_nxt[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_nxt[2], 'f', 3));
                ui.info_logger->addItem("PP Final Position: X: " + QString::number(UAVs[it].pos_fin[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_fin[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_fin[2], 'f', 3));
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
    } else{ // Print by item
        if (checkbox_stat.print_imu){
            for (const auto &it : avail_uavind){
                // Imu imu_data = qnode.GetImu_uavs(it);
                // float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
                // outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
                // ui.info_logger->addItem("uav " + QString::number(it+1) + ": Roll: " + QString::number(uav_euler.roll*180/3.14159, 'f', 2) + ". Pitch: " + 
                //                 QString::number(uav_euler.pitch*180/3.14159, 'f', 2) + ". Yaw: " + QString::number(uav_euler.yaw*180/3.14159, 'f', 2));
                ui.info_logger->addItem("Roll: " + QString::number(UAVs[it].ang_cur[0], 'f', 2) + ". Pitch: " + 
                                QString::number(UAVs[it].ang_cur[1], 'f', 2) + ". Yaw: " + QString::number(UAVs[it].ang_cur[2], 'f', 2));
                if (!UAVs[it].imuReceived){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
        if (checkbox_stat.print_state){
            for (const auto &it : avail_uavind){
                mavros_msgs::State state_data = qnode.GetState_uavs(it);
                QString state_to_be_print = "uav " + QString::number(it+1) + ": ";
                if (state_data.connected){
                    state_to_be_print += "Connected! ";
                } else{
                    state_to_be_print += "UNConnected! ";
                }
                if (state_data.mode.empty()){
                    state_to_be_print += "Mode Empty! ";
                } else{
                    state_to_be_print += QString::fromStdString(state_data.mode) + "! ";
                }
                if (state_data.armed){
                    state_to_be_print += "ARMED!";
                } else{
                    state_to_be_print += "DISARMED!";
                }
                ui.info_logger->addItem(state_to_be_print);
                if (!state_data.connected){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
        if (checkbox_stat.print_gps){
            for (const auto &it : avail_uavind){
                outdoor_gcs::GPSRAW gps_data = qnode.GetGPS_uavs(it);
                ui.info_logger->addItem("uav " + QString::number(it+1) + 
                                        ": Num: " + QString::number(gps_data.satellites_visible) + ". Lat: " + QString::number(gps_data.lat*1e-7, 'f', 7) +
                                        ". Lon: " + QString::number(gps_data.lon*1e-7, 'f', 7) + ". Alt: " + QString::number(gps_data.alt*1e-3, 'f', 3));
                if (!UAVs[it].gpsReceived){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                }
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
        if (checkbox_stat.print_local){
            for (const auto &it : avail_uavind){
                ui.info_logger->addItem("uav " + QString::number(it+1) + 
                                        ": Local Position: X: " + QString::number(UAVs[it].pos_cur[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_cur[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_cur[2], 'f', 3));
                // ui.info_logger->addItem("Local Velocity: X: " + QString::number(UAVs[it].vel_cur[0], 'f', 3) +
                //                         ". Y: " + QString::number(UAVs[it].vel_cur[1], 'f', 3) +
                //                         ". Z: " + QString::number(UAVs[it].vel_cur[2], 'f', 3));
                int item_index = ui.info_logger->count()-1;
                if (UAVs[it].arrive){
                    ui.info_logger->item(item_index)->setForeground(Qt::darkGreen);
                }
                if (!UAVs[it].gpsLReceived){
                    // int item_index2 = ui.info_logger->count()-2;
                    ui.info_logger->item(item_index)->setForeground(Qt::red);
                    // ui.info_logger->item(item_index2)->setForeground(Qt::red);
                }
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
        if (checkbox_stat.print_des){
            for (const auto &it : avail_uavind){
                // outdoor_gcs::Topic_for_log log = qnode.GetLog_uavs(it);
                // ui.info_logger->addItem("Throttle: X: " + QString::number(log.Control_Output.Throttle[0], 'f', 3) +
                //                         ". Y: " + QString::number(log.Control_Output.Throttle[1], 'f', 3) + 
                //                         ". Z: " + QString::number(log.Control_Output.Throttle[2], 'f', 3));
                ui.info_logger->addItem("uav " + QString::number(it+1) + 
                                        ": Desired Position: X: " + QString::number(UAVs[it].pos_des[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_des[1], 'f', 3) + 
                                        ". Z: " + QString::number(UAVs[it].pos_des[2], 'f', 3));
                if(UAVs[it].move){
                    int item_index = ui.info_logger->count()-1;
                    ui.info_logger->item(item_index)->setForeground(Qt::darkGreen);
                }
            }
            ui.info_logger->addItem("----------------------------------------------------------------------------------------");
        }
        if (checkbox_stat.print_pathplan){
            for (const auto &it : avail_uavind){
                ui.info_logger->addItem("uav " + QString::number(it+1) + 
                                        ": PP Init Position: X: " + QString::number(UAVs[it].pos_ini[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_ini[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_ini[2], 'f', 3));
                ui.info_logger->addItem("           PP Next Position: X: " + QString::number(UAVs[it].pos_nxt[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_nxt[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_nxt[2], 'f', 3));
                ui.info_logger->addItem("           PP Final Position: X: " + QString::number(UAVs[it].pos_fin[0], 'f', 3) +
                                        ". Y: " + QString::number(UAVs[it].pos_fin[1], 'f', 3) +
                                        ". Z: " + QString::number(UAVs[it].pos_fin[2], 'f', 3));
            }
        }
    }
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "outdoor_gcs");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "outdoor_gcs");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace outdoor_gcs

