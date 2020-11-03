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

void MainWindow::on_Button_Set_clicked(bool check){
    /* read values from line edit */
    float target_state[3];

    target_state[0] =  ui.x_input->text().toFloat();
    target_state[1] =  ui.y_input->text().toFloat();
    target_state[2] =  ui.z_input->text().toFloat();
    /*----------------determine whether the input is in safe range ------------------*/
    bool input_is_valid = true;

    if(target_state[0] < -10.0 || target_state[0] > 10.0) {
        input_is_valid = false;
    }

    if(target_state[1] < -10.0 || target_state[1] > 10.0) {
        input_is_valid = false;
    }

    if(target_state[2] < 0|| target_state[2] > 30.0) {
        input_is_valid = false;
    }

    /*----------------send input ------------------*/

    if(input_is_valid){
        /*  update the ENU target label */
        ui.des_x->setText(QString::number(target_state[0], 'f', 2));
        ui.des_y->setText(QString::number(target_state[1], 'f', 2));
        ui.des_z->setText(QString::number(target_state[2], 'f', 2));

        qnode.move_uav(target_state, 0.0);
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
    ui.x_input->setText(QString::number(gpsL_data.pose.pose.position.x, 'f', 2));
    ui.y_input->setText(QString::number(gpsL_data.pose.pose.position.y, 'f', 2));
    ui.z_input->setText(QString::number(gpsL_data.pose.pose.position.z, 'f', 2));
}



////////////////////////// Multi-uav button /////////////////////////

void MainWindow::on_Update_UAV_List_clicked(bool check){
    ui.uav_detect_logger->clear();
    all_topics = qnode.lsAllTopics();
    UAV_Detected.clear();
    avail_uavind.clear();
    for(int i = 0; i < DroneNumber ; i++) {
        UAVs[i] = qnode.Get_UAV_info(i);
        UAVs[i].rosReceived = false;
        QString filter_word = "uav" + QString::number(i+1);
        QStringList filtered_topics = all_topics.filter(filter_word);
        if (filtered_topics.count() != 0){
            UAV_Detected += filter_word;
            UAVs[i].rosReceived = true;
            avail_uavind.push_back(i);
        }
        qnode.Update_UAV_info(UAVs[i], i);
    }
    ui.uav_detect_logger->addItems(UAV_Detected);
    // for (const auto &it : avail_uavind){
	//     std::cout << std::to_string(it) << std::endl;
    // }

}

void MainWindow::on_Set_GPS_Origin_clicked(bool check){
    QList<QListWidgetItem *> selected_uav = ui.uav_detect_logger->selectedItems();
    for(int i = 0; i < DroneNumber ; i++) {
        if (selected_uav[0]->text() == "uav" + QString::number(i+1)){
            ui.info_logger->addItem("uav " + QString::number(i+1) + " selected to set for origin!");
            break;
        }
    }
}

void MainWindow::on_InfoLogger_Clear_clicked(bool check){
    ui.info_logger->clear();
}

void MainWindow::on_Print_IMU_clicked(bool check){
    for(int i = 0; i < DroneNumber ; i++) {
        UAVs[i] = qnode.Get_UAV_info(i);
        if(UAVs[i].rosReceived){
            Imu imu_data = qnode.GetImu_uavs(i);
            float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
            outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
            ui.info_logger->addItem("uav " + QString::number(i+1) + ": roll: " + QString::number(uav_euler.roll, 'f', 3) +
                                ", pitch: " + QString::number(uav_euler.pitch, 'f', 3) + ", yaw: " + QString::number(uav_euler.yaw, 'f', 3));
        }
    }
}

// CheckBox //
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
void MainWindow::updateuav(){

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
        ui.roll->setText(QString::number(uav_euler.roll, 'f', 3));
        ui.pitch->setText(QString::number(uav_euler.pitch, 'f', 3));
        ui.yaw->setText(QString::number(uav_euler.yaw, 'f', 3));

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
        qnode.Do_Plan();
		ui.Enable_Planning->setText("Disable Planning");
    }else{
		ui.Enable_Planning->setText("Enable_Planning");
    }

}

void MainWindow::updateInfoLogger(){
    if (checkbox_stat.clear_each_print){
        ui.info_logger->clear();
    }
    for (const auto &it : avail_uavind){

        if (checkbox_stat.print_imu || checkbox_stat.print_state || checkbox_stat.print_gps ||
            checkbox_stat.print_local || checkbox_stat.print_des){
            UAVs[it] = qnode.Get_UAV_info(it);
            ui.info_logger->addItem("uav " + QString::number(it+1) + " (id = " + QString::number(UAVs[it].id) + ") info: ");
            int item_index = ui.info_logger->count()-1;
            ui.info_logger->item(item_index)->setForeground(Qt::blue);
        } else{ continue; }

        if (checkbox_stat.print_imu){
            Imu imu_data = qnode.GetImu_uavs(it);
            if (UAVs[it].imuReceived){
                float quat[4] = {imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z};
                outdoor_gcs::Angles uav_euler = qnode.quaternion_to_euler(quat);
                ui.info_logger->addItem("Roll: " + QString::number(uav_euler.roll, 'f', 3) + ". Pitch: " + 
                                QString::number(uav_euler.pitch, 'f', 3) + ". Yaw: " + QString::number(uav_euler.yaw, 'f', 3));
            }else{
                ui.info_logger->addItem("Imu empty");
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
        }
        if (checkbox_stat.print_gps){
	        outdoor_gcs::GPSRAW gps_data = qnode.GetGPS_uavs(it);
            if (UAVs[it].gpsReceived){
                ui.info_logger->addItem("Num: " + QString::number(gps_data.satellites_visible) + ". Lat: " + QString::number(gps_data.lat*1e-7, 'f', 7)
                                         + ". Lon: " + QString::number(gps_data.lon*1e-7, 'f', 7) + ". Alt: " + QString::number(gps_data.alt*1e-3, 'f', 3));
            }else{
                ui.info_logger->addItem("GPS empty");
                int item_index = ui.info_logger->count()-1;
                ui.info_logger->item(item_index)->setForeground(Qt::red);
            }
        }
        if (checkbox_stat.print_local){
            Gpslocal gpsL_data = qnode.GetGPSL_uavs(it);
            if (UAVs[it].gpsLReceived){
                ui.info_logger->addItem("Local Position: X: " + QString::number(gpsL_data.pose.pose.position.x, 'f', 3) +
                                        ". Y: " + QString::number(gpsL_data.pose.pose.position.y, 'f', 3) +
                                        ". Z: " + QString::number(gpsL_data.pose.pose.position.z, 'f', 3));
                ui.info_logger->addItem("Local Velocity: X: " + QString::number(gpsL_data.twist.twist.linear.x, 'f', 3) +
                                        ". Y: " + QString::number(gpsL_data.twist.twist.linear.y, 'f', 3) +
                                        ". Z: " + QString::number(gpsL_data.twist.twist.linear.z, 'f', 3));
            }else{
                ui.info_logger->addItem("Local empty");
                int item_index = ui.info_logger->count()-1;
                ui.info_logger->item(item_index)->setForeground(Qt::red);
            }
        }
        if (checkbox_stat.print_des){
            ui.info_logger->addItem("Desired Position: X: " + QString::number(UAVs[it].pos_des[0], 'f', 3) +
                                     ". Y: " + QString::number(UAVs[it].pos_des[1], 'f', 3) + 
                                     ". Z: " + QString::number(UAVs[it].pos_des[2], 'f', 3));
        }

        ui.info_logger->addItem("----------------------------------------------------------------------------------------");
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

