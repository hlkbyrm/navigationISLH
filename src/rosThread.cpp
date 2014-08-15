#include <tf/tf.h>


#include "rosThread.h"

#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>
#include <QtNetwork/QHostInfo>

RosThread::RosThread()
{
    shutdown = false;
}

void RosThread::startModule()
{
    //pt.start();
    //ct.start();

    //bin[robot.robotID][1] = this->robot.initialX;
    //bin[robot.robotID][2] = this->robot.initialY;

//    QString fileName = QDir::homePath();
//    fileName.append("/ISL_workspace/src/pose.txt");

//    poseFile.setFileName(fileName);

//    poseFile.open(QFile::WriteOnly);

//    QTextStream stream(&poseFile);

//    stream<<QDateTime::currentDateTime().toTime_t()<<" "<<bin[robot.robotID][1]<<" "<<bin[robot.robotID][2]<<" "<<robot.radius<<"\n";

//    poseFile.close();

    /*geometry_msgs::PoseWithCovarianceStamped initialpose;

    initialpose.pose.pose.position.x = this->robot.initialX/100;

    initialpose.pose.pose.position.y = this->robot.initialY/100;

    initialpose.pose.pose.position.z = 0.1;

    initialpose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    amclInitialPosePublisher.publish(initialpose);*/

    velocityVector.linear.x = 0.0;
}
void RosThread::work(){

    startNavigation = true;

    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");

    QString fileName = QDir::homePath();
    fileName.append("/ISL_workspace/src/pose.txt");
    poseFile.setFileName(fileName);

    if(!readConfigFile(path)){
        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();

        return;
    }

    radYaw = 0.0;
    vel[0] = 0.0;
    vel[1] = 0.0;
    isFinished = false;

    if(!ros::ok()){

        qDebug()<< "ROS Init failed!!!";

        ros::shutdown();

        emit rosFinished();

        return;
    }

    emit rosStarted();
    this->poseListSub = n.subscribe("pose_list",1,&RosThread::poseListCallback,this);
    //this->neighborInfoSubscriber = n.subscribe("communicationISL/neighborInfo",5,&RosThread::neighborInfoCallback,this);
    //this->turtlebotOdometrySubscriber = n.subscribe("odom",2,&RosThread::turtlebotOdometryCallback,this);
    this->targetPoseListSub = n.subscribe("targetPoseList",2,&RosThread::targetPoseListCallback,this);
    //publisher değiştirildi safety controller eklendi. minimal launchera ek olarak safe_keyop.launch çalıştırılması gerekiyor
    if(!isKobuki)
        this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    else
        this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("/keyop_vel_smoother/raw_cmd_vel",1);
    this->turtlebotGyroSub = n.subscribe("/mobile_base/sensors/imu_data",1,&RosThread::turtlebotGyroCallback,this);
    this->turtlebotOdomSub = n.subscribe("/odom",1,&RosThread::turtlebotOdomCallback,this);
    //this->amclInitialPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);
    //this->coordinatorUpdatePublisher = n.advertise<navigationISLH::robotInfo>("navigationISLH/coordinatorUpdate",1,true);

    this->robotPosePublisher = n.advertise<navigationISLH::robotPose>("robot_position_info",1,true);

    if(feedbackToServer){
        timer = n.createTimer(ros::Duration(0.25),&RosThread::timerTick,this);
        timer.start();
    }

    //this->robotinfoPublisher = n.advertise<navigationISLH::robotInfo>("navigationISLH/robotInfo",1);
    //ros::AsyncSpinner spinner(2);
    //pt = n.createTimer(ros::Duration(poseUpdatePeriod), &RosThread::poseUpdate,this);
    //ct = n.createTimer(ros::Duration(coordinatorUpdatePeriod),&RosThread::coordinatorUpdate,this);
    //ct.stop();
    //pt.stop();

    ros::Rate loop(10);

    velocityVector.linear.x = 0;

    while(ros::ok())
    {

        if(startNavigation) {
            std::vector<std::vector<double> > empty;
            double notnormalized[1];
            NavigationController::robotContoller(vel, notnormalized, numrobots, /*obstacles.size()*/0, /*partDist*/0, bin, bt, b_rs, empty, ro, kkLimits, robot.robotID);

            qDebug()<<"notnormalized[0]"<<notnormalized[0];
            qDebug()<<"vel[0]"<<vel[0]<<"vel[1]"<<vel[1];

            /*if(notnormalized[0] < stoppingThreshold)
                isFinished = true;
            else
                isFinished = false;*/

            this->sendVelocityCommand();
        }
        ros::spinOnce();
        loop.sleep();


    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;
}
void RosThread::targetPoseListCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    for(int i=0;i<msg->poses.size();i++){
        bt[i+1][1] = msg->poses[i].position.x;
        bt[i+1][2] = msg->poses[i].position.y;
    }

    robot.targetX = bt[robot.robotID][1];
    robot.targetY = bt[robot.robotID][2];

    qDebug() << "TargetX: " << bt[robot.robotID][1] << " Y: " << bt[robot.robotID][2];
}
bool turning = false; // Flag for not using camera data when turtlebot is turning
bool firstDataCame = false; // Flag for first data because we must not use gyro or odom before first data
ros::Time current_timeO, last_timeO; // Used for converting velocity to distance or angle
ros::Time current_timeG, last_timeG; // Used for converting velocity to distance or angle
double enSonAlinanYolX = 0.0; //Used for correction of data from camera
double enSonAlinanYolY = 0.0; //Used for correction of data from camera
double enSonDonulenAci = 0.0; //Used for correction of data from camera
void RosThread::turtlebotOdomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    if(!firstDataCame) return;
    current_timeO = ros::Time::now();
    double diffx = msg->twist.twist.linear.x * 100*((current_timeO - last_timeO).toSec());
    qDebug() << "twist X: " << diffx << " +X: " << diffx*cos(radYaw) << " +Y: " << diffx*sin(radYaw);
    bin[robot.robotID][1] += diffx*cos(radYaw);
    bin[robot.robotID][2] += diffx*sin(radYaw);
    enSonAlinanYolX += diffx*cos(radYaw);
    enSonAlinanYolY += diffx*sin(radYaw);
    last_timeO = current_timeO;
}
void RosThread::turtlebotGyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!firstDataCame) return;
    current_timeG = ros::Time::now();
    qDebug() << "Gyro X:" << msg->angular_velocity.x << "Y:" << msg->angular_velocity.y << "Z:" << msg->angular_velocity.z;
    radYaw += msg->angular_velocity.z*((current_timeG - last_timeG).toSec());
    enSonDonulenAci += msg->angular_velocity.z*((current_timeG - last_timeG).toSec());
    last_timeG = current_timeG;

    /*double d1,d2,radYaw1,radYaw2;
    tf::Matrix3x3(oldq).getEulerYPR(radYaw1,d1,d2);
    tf::Quaternion odomquatNew(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf::Matrix3x3(odomquatNew).getEulerYPR(radYaw2,d1,d2);
    radYaw1 = radYaw1*M_PI/180.0;
    radYaw2 = radYaw2*M_PI/180.0;
    qDebug() << "radYaw1 : " << radYaw1 << " radYaw2 : " << radYaw2;
    radYaw += radYaw2 - radYaw1;*/

    double calYaw = atan2(vel[1],vel[0]);

    if(calYaw < 0)
    {
        calYaw += M_PI*2;
    }
    if(radYaw < 0)
    {
        radYaw += M_PI*2;
    }
    if(radYaw > M_PI*2)
    {
        radYaw -= M_PI*2;
    }


    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    velocityVector = twist;
    if(isFinished && fabs(robot.targetX-bin[robot.robotID][1]) <= distanceThreshold &&
            fabs(robot.targetY-bin[robot.robotID][2]) <= distanceThreshold)
        return;

    if(!poseFile.exists())
        poseFile.open(QFile::WriteOnly);
    else
        poseFile.open(QFile::Append);
    QTextStream stream(&poseFile);
    stream<<"G " << QDateTime::currentDateTime().toTime_t()<<" "<<bin[robot.robotID][1]<<" "<<bin[robot.robotID][2]<<" "<<robot.radius<<" ";
    stream << radYaw << "\n";
    poseFile.close();

    qDebug() << "radYaw : " << radYaw << " calYaw : " << calYaw;
    qDebug() << "fabs(radYaw-calYaw)" << fabs(radYaw-calYaw);
    qDebug() << "angleThreshold*M_PI/180.0" << angleThreshold*M_PI/180.0;
    qDebug() << "turning : " << (turning?"true":"false");
    double diff = fabs(radYaw - calYaw);
    if(diff > M_PI){
        diff -= 2*M_PI;
        diff = fabs(diff);
    }
    if((diff > angleThreshold*M_PI/180.0 && !turning) ||
            (diff > angleThreshold/2*M_PI/180.0 && turning))
    {
        turning = true;
        calculateTurn(calYaw,radYaw);

        //qDebug() << "numrobots           : " << numrobots;
        //qDebug() << "robot.robotID         : " << robot.robotID;
        qDebug() << "bin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        //qDebug() << "ro                    : " << ro;
        qDebug()<<"LinearVel:X:"<<velocityVector.linear.x<<"Y:"<<velocityVector.linear.y<<"Z:"<<velocityVector.linear.z;
        qDebug()<<"AngularVel:X:"<<velocityVector.angular.x<<"Y:"<<velocityVector.angular.y<<"Z:"<<velocityVector.angular.z;

        return;
    }
    else
    {
        turning = false;
        if(/*!isFinished || */fabs(robot.targetX-bin[robot.robotID][1]) > distanceThreshold || fabs(robot.targetY-bin[robot.robotID][2]) > distanceThreshold){
           // qDebug()<<"Linear";
            velocityVector.linear.x = linearVelocity;
            if((diff > angleThreshold/5*M_PI/180.0 && !turning))
                calculateTurn(calYaw,radYaw);
        }
        else
            velocityVector.linear.x = 0;

        //qDebug() << "numrobots           : " << numrobots;
        //qDebug() << "robot.robotID         : " << robot.robotID;
        qDebug() << "bin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        //qDebug() << "ro                    : " << ro;
        qDebug()<<"LinearVel:X:"<<velocityVector.linear.x<<"Y:"<<velocityVector.linear.y<<"Z:"<<velocityVector.linear.z;
        qDebug()<<"AngularVel:X:"<<velocityVector.angular.x<<"Y:"<<velocityVector.angular.y<<"Z:"<<velocityVector.angular.z;
        //   turtlebotVelPublisher.publish(twist);
    }
}
bool turning2 = false;
u_int64_t lastPoseCallbackTime;
void RosThread::poseListCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if(!firstDataCame) {
        lastPoseCallbackTime = ros::Time::now().toNSec();
        current_timeG = ros::Time::now();
        last_timeG = ros::Time::now();
        current_timeO = ros::Time::now();
        last_timeO = ros::Time::now();
    }
    firstDataCame = true;
    qDebug() <<"data input";
    for(int i=1;i<msg->poses.size();i++){
        bin[i][1] = msg->poses[i].position.x;
        bin[i][2] = msg->poses[i].position.y;
        bin[i][3] = robot.radius;
        if(i != robot.robotID){
            bt[i][1] = bin[i][1];
            bt[i][2] = bin[i][2];
        }
    }

    u_int64_t newPoseCallbackTime = ros::Time::now().toNSec();
    ros::Time receivedTime;
    double d = msg->poses[0].position.x;
    receivedTime.fromSec(d);
    int _diff1 = (newPoseCallbackTime/1000000 - lastPoseCallbackTime/1000000);
    int _diff2 = (newPoseCallbackTime/1000000 - (msg->poses[0].position.x*1000.0));
    qDebug() << newPoseCallbackTime;
    qDebug() << msg->poses[0].position.x*1000000000.0;
    qDebug() << newPoseCallbackTime - (msg->poses[0].position.x*1000000000.0);
    qDebug() << QString::number(msg->poses[0].position.x);
    if(_diff1 != 0){
        qDebug() << "AddedToX: " << enSonAlinanYolX * _diff2 / _diff1 << " Y: " << enSonAlinanYolY * _diff2 / _diff1;
        qDebug() << "diff1: " << _diff1 << " 2: " << _diff2;
        if(_diff2 > _diff1) _diff1 = _diff2;
        bin[robot.robotID][1] += enSonAlinanYolX * _diff2 / _diff1;
        bin[robot.robotID][2] += enSonAlinanYolY * _diff2 / _diff1;
        qDebug() << "BASARILI";
    }

    enSonAlinanYolX = 0.0;
    enSonAlinanYolY = 0.0;
    lastPoseCallbackTime = newPoseCallbackTime;
    qDebug() << "BASARILI";

    /*tf::Quaternion odomquat(msg->poses[robot.robotID-1].orientation.x,msg->poses[robot.robotID-1].orientation.y,msg->poses[robot.robotID-1].orientation.z,msg->poses[robot.robotID-1].orientation.w);
    double d1,d2,radYaw;
    tf::Matrix3x3(odomquat).getEulerYPR(radYaw,d1,d2);*/
    //double radYaw = tf::getYaw(odomquat);

    if(!poseFile.exists())
    {
        poseFile.open(QFile::WriteOnly);
    }
    else
    {
        poseFile.open(QFile::Append);
    }
    QTextStream stream(&poseFile);

    qDebug() << "BASARILI";

    stream<<QDateTime::currentDateTime().toTime_t()<<" "<<bin[robot.robotID][1]<<" "<<bin[robot.robotID][2]<<" "<<robot.radius<<" ";

    if(!turning){
        if(!turning2){
            radYaw = msg->poses[robot.robotID].position.z + enSonDonulenAci ;
            //oldq = tf::Quaternion(msg->poses[robot.robotID-1].orientation.x,msg->poses[robot.robotID-1].orientation.y,msg->poses[robot.robotID-1].orientation.z,msg->poses[robot.robotID-1].orientation.w);
            qDebug() << "MRad yaw: " << radYaw;
            stream << "M ";
        }
        turning2 = false;
    }else turning2 = true;
    enSonDonulenAci = 0.0;

    double calYaw = atan2(vel[1],vel[0]);

    if(calYaw < 0)
    {
        calYaw += M_PI*2;
    }
    if(radYaw < 0)
    {
        radYaw += M_PI*2;
    }

    stream << radYaw << " " << calYaw << " " << (turning?"Y":"N") << "\n";

    poseFile.close();



    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    velocityVector = twist;
    if(isFinished && fabs(robot.targetX-bin[robot.robotID][1]) <= distanceThreshold &&
            fabs(robot.targetY-bin[robot.robotID][2]) <= distanceThreshold)
        return;

    qDebug() << "radYaw : " << radYaw << " calYaw : " << calYaw;
    qDebug() << "fabs(radYaw-calYaw)" << fabs(radYaw-calYaw);
    qDebug() << "angleThreshold*M_PI/180.0" << angleThreshold*M_PI/180.0;
    qDebug() << "turning : " << (turning?"true":"false");
    double diff = fabs(radYaw - calYaw);
    if(diff > M_PI){
        diff -= 2*M_PI;
        diff = fabs(diff);
    }
    if((diff > angleThreshold*M_PI/180.0 && !turning) ||
            (diff > angleThreshold/2*M_PI/180.0 && turning) )
    {
        turning = true;
        calculateTurn(calYaw,radYaw);

        //qDebug() << "numrobots           : " << numrobots;
        //qDebug() << "robot.robotID         : " << robot.robotID;
        qDebug() << "Mbin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        //qDebug() << "ro                    : " << ro;
        qDebug()<<"LinearVel:X:"<<velocityVector.linear.x<<"Y:"<<velocityVector.linear.y<<"Z:"<<velocityVector.linear.z;
        qDebug()<<"AngularVel:X:"<<velocityVector.angular.x<<"Y:"<<velocityVector.angular.y<<"Z:"<<velocityVector.angular.z;

        return;
    }
    else
    {
        turning = false;
        if(/*!isFinished || */fabs(robot.targetX-bin[robot.robotID][1]) > distanceThreshold || fabs(robot.targetY-bin[robot.robotID][2]) > distanceThreshold){
           // qDebug()<<"Linear";
            velocityVector.linear.x = linearVelocity;
            if((diff > angleThreshold/5*M_PI/180.0 && !turning))
                calculateTurn(calYaw,radYaw);
        }
        else
            velocityVector.linear.x = 0;

        //qDebug() << "numrobots           : " << numrobots;
        //qDebug() << "robot.robotID         : " << robot.robotID;
        qDebug() << "Mbin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        //qDebug() << "ro                    : " << ro;
        qDebug()<<"LinearVel:X:"<<velocityVector.linear.x<<"Y:"<<velocityVector.linear.y<<"Z:"<<velocityVector.linear.z;
        qDebug()<<"AngularVel:X:"<<velocityVector.angular.x<<"Y:"<<velocityVector.angular.y<<"Z:"<<velocityVector.angular.z;
        //   turtlebotVelPublisher.publish(twist);
    }
    // ROS_INFO("position x %4.2f position y %4.2f orientation %4.2f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.orientation.z*180/3.14);

}
// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {

        numrobots = result["numrobots"].toInt();

        qDebug()<<result["numrobots"].toString();

        bin = std::vector<std::vector<double> >(numrobots+1,std::vector<double>(4));
        bt = std::vector<std::vector<double> >(numrobots+1,std::vector<double>(3));
        rr = std::vector<double>(numrobots+1);
        b_rs = std::vector<std::vector<double> >(numrobots+1,std::vector<double>(4));

        this->robot.radius = result["radius"].toDouble();

        qDebug()<<result["radius"].toString();

        this->linearVelocity = result["linearVelocity"].toDouble();

        qDebug()<<result["linearVelocity"].toDouble();

        this->isKobuki = result["isKobuki"].toInt() == 1;

        this->angularVelocity = result["angularVelocity"].toDouble();

        this->angleThreshold = result["angleThreshold"].toDouble();

        this->distanceThreshold = result["distanceThreshold"].toInt();

        qDebug()<<distanceThreshold;

        this->stoppingThreshold = result["stoppingThreshold"].toDouble();

        qDebug()<<stoppingThreshold;

        ro = result["ro"].toInt();

        qDebug()<<ro;

        kkLimits[0] = result["kMin"].toInt();

        qDebug()<<kkLimits[0];

        kkLimits[1] = result["kMax"].toInt();

        qDebug()<<kkLimits[1];

        partDist = result["partDist"].toInt();

        qDebug()<<partDist;

        robot.robotID = result["robotID"].toInt();

        qDebug()<<result["robotID"].toString();

        feedbackToServer = result["feedbackToServer"].toInt() == 1;

        qDebug() << "feedbackToServer: " << (feedbackToServer?"true":"false");

        IP = result["IP"].toString();

        qDebug() << "IP: " << IP;

        int iscoord =   result["iscoordinator"].toInt();
        if(iscoord == 1) this->robot.isCoordinator = true;

        //this->robot.initialX = result["initialX"].toDouble();

        //this->robot.initialY = result["initialY"].toDouble();

        this->robot.targetX = result["targetX"].toDouble();

        qDebug()<<result["targetX"].toString();

        this->robot.targetY = result["targetY"].toDouble();

        qDebug()<<result["targetY"].toString();


        /*QVariantMap nestedMap = result["Obstacles"].toMap();

        //  this->obstacles.resize(4);

        int count = 0;
        foreach (QVariant plugin, nestedMap["Obstacle"].toList()) {

            Obstacle obstacle;

            obstacle.id = plugin.toMap()["id"].toInt();

            qDebug()<<obstacle.id;

            obstacle.radius = plugin.toMap()["radius"].toDouble();

            obstacle.x= plugin.toMap()["x"].toDouble();

            obstacle.y = plugin.toMap()["y"].toDouble();

            //  if(coord == 1) robot->setCoordinator(true);

            obstacles.push_back(obstacle);

            count++;
            // qDebug() << "\t-" << plugin.toMap()["ip"].toString();
        }
        for(int i = 0 ; i < obstacles.size(); i++)
        {

            bp[i+1][1] = obstacles[i].x;
            bp[i+1][2] = obstacles[i].y;
            bp[i+1][3] = obstacles[i].radius;

        }*/
        for(int i = 1; i <= numrobots; i++)
        {
            if(i  != robot.robotID){
                bin[i][1] = 0;
                bin[i][2] = 0;
                bin[i][3] = 0;

                bt[i][1] = 0;
                bt[i][2] = 0;
            }
            else{
                bt[i][1] = robot.targetX;
                bt[i][2] = robot.targetY;
            }

            b_rs[i][1] = 0;
            b_rs[i][2] = 0;
            b_rs[i][3] = 0;
        }

        qDebug()<<"file read finished";
    }
    file.close();

    qDebug()<<"DONE!!!";
    return true;
}
// Sends the velocity command to robot
void RosThread::sendVelocityCommand() {
    turtlebotVelPublisher.publish(velocityVector);
}
void RosThread::calculateTurn(double desired, double current) {
    int dir = 0;

    /* if(desired < 0)
    {
        desired += M_PI*2;
    }

    if(current < 0)
    {
        current += M_PI*2;
    }*/

    double diff = desired-current;

    if(diff > 0 )
    {
        dir = 1;
    }
    else
    {
        dir = -1;
    }

    if(fabs(M_PI*2 - fabs(diff)) < fabs(diff))
    {
        dir = dir*-1;
    }

    velocityVector.angular.z = dir*angularVelocity;

}
void RosThread::timerTick(const ros::TimerEvent&){
    navigationISLH::robotPose rp;

    qDebug()<<"entered";

    if(!firstDataCame) return;

    double calYaw = atan2(vel[1],vel[0]);
    if(calYaw < 0)
        calYaw += M_PI*2;

    rp.id = robot.robotID;
    rp.position.x = bin[robot.robotID][1];
    rp.position.y = bin[robot.robotID][2];
    rp.target.x = bt[robot.robotID][1];
    rp.target.y = bt[robot.robotID][2];
    rp.calYaw = calYaw;
    rp.radYaw = radYaw;

    this->robotPosePublisher.publish(rp);
}

// The odometry feedback from the robot (redundant)
/*void RosThread::turtlebotOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    bin[robot.robotID][1] = msg->pose.pose.position.x*100;
    bin[robot.robotID][2] = msg->pose.pose.position.y*100;
    bin[robot.robotID][3] = robot.radius;

    tf::Quaternion odomquat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z->msg.pose.pose.orientation.w);

    double d1,d2,radYaw;
    tf::Matrix3x3(odomquat).getEulerYPR(radYaw,d1,d2);
    // double radYaw = tf::getYaw(odomquat);

    qDebug()<<"Rad yaw: "<<radYaw;

    double cradyaw = cos(radYaw);
    double sradyaw = sin(radYaw);

    //vel[1] = -0.5;

    // vel[0] = 0.5;

    double calYaw = atan2((robot.targetY-bin[robot.robotID][2]),(robot.targetX-bin[robot.robotID][1]));

    qDebug()<<"Bin: "<<bin[robot.robotID][1]<<"Bin 2: "<<bin[robot.robotID][2];
    qDebug()<<"Cal yaw: "<<calYaw;

    //  double ccalyaw = cos(calYaw);
    //double scalyaw = sin(calYaw);

    //double diffYaw = radYaw-calYaw;

    //qDebug()<<"Diff yaw: "<<diffYaw;

    geometry_msgs::Twist twist;

    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    velocityVector = twist;

//    if((fabs(ccalyaw - cradyaw) > (double)0.5 && ccalyaw*cradyaw < 0 )|| (fabs(scalyaw-sradyaw) > (double)0.5 && scalyaw*sradyaw < 0))
//    {

//        if(calYaw >= 0)
//        {

//            twist.angular.z = 0.3;

//            //turtlebotVelPublisher.publish(twist);

//        }
//        else
//        {

//            twist.angular.z = -0.3;



//        }

//      //  turtlebotVelPublisher.publish(twist);

//    }
    if(fabs(radYaw-calYaw) >= angleThreshold*M_PI/180)
    {

        calculateTurn(calYaw,radYaw);

        return;
    }
    else
    {


        if(fabs(robot.targetX-bin[robot.robotID][1]) > distanceThreshold || fabs(robot.targetY-bin[robot.robotID][2]) > distanceThreshold){
            qDebug()<<"Linear";
            velocityVector.linear.x = linearVelocity;
        }
        else
            velocityVector.linear.x = 0;

        //   turtlebotVelPublisher.publish(twist);


    }

    //  velocityVector = twist;


}*/
// Received neighbor info
/*void RosThread::neighborInfoCallback(navigationISLH::neighborInfo neighborInfo)
{
    QString str = QString::fromStdString(neighborInfo.name);

    if(str == "start")
    {
        this->startModule();

        this->startNavigation = true;

        return;

    }
    else if(str.contains("Neighbors"))
    {
        QStringList list = str.split(";");

        QVector<int> ids;

        for(int i = 0; i < list.size(); i++)
        {

            if(list.at(i).contains("IRobot"))
            {
                QString ss = list.at(i);

                ss.remove("IRobot");

                ids.push_back(ss.toInt());
            }
            else if(list.at(i) == "0")
            {
                for(int j= 1; j <= numOfRobots; j++)
                {
                    if(j != this->robot.robotID)
                    {
                        bin[j][1] = 0;
                        bin[j][2] = 0;
                        bin[j][3] = 0;
                    }

                }
                break;
            }
        }
        if(ids.size() > 0)
        {
            for(int i = 1; i <= numOfRobots; i++)
            {

                for(int j = 0; j < ids.size(); j++)
                {
                    if(i != this->robot.robotID && i != ids.at(j))
                    {
                        bin[i][1] = 0;
                        bin[i][2] = 0;
                        bin[i][3] = 0;
                    }

                }

            }
        }

        for(int k = 1; k <= numOfRobots; k++)
        {
            qDebug()<<"Bin value "<<k<<" "<<bin[k][3];
        }

        return;

    }

    str.remove("IRobot");

    int num = str.toInt();

    if(num > 0 && num < numOfRobots){

        bin[num][1] = neighborInfo.posX;
        bin[num][2] = neighborInfo.posY;
        bin[num][3] = neighborInfo.radius;

        bt[num][1] = neighborInfo.targetX;
        bt[num][2] = neighborInfo.targetY;
        qDebug()<<"robot number "<<num;
    }
    else qDebug()<<"Unknown robot id number";
}*/
// Tc saniyede Komsulara kendi bilgisini gonderiyor
/*void RosThread::poseUpdate(const ros::TimerEvent&)
{

    navigationISLH::robotInfo info;

    info.neighbors.resize(1);

    info.posX = bin[robot.robotID][1];

    info.posY = bin[robot.robotID][2];

    info.targetX = robot.targetX;

    info.targetY = robot.targetY;

    info.radius = robot.radius;

    qDebug()<<"posUpdate:"<<info.posX<<" "<<info.posY;

    robotinfoPublisher.publish(info);



}*/
// Tg saniyede Coordinator a kendi konum bilgisini gonderiyor
/*void RosThread::coordinatorUpdate(const ros::TimerEvent&)
{
    pt.stop();

    navigationISLH::robotInfo info;

    info.neighbors.resize(1);
    info.neighbors[0] = "";
    info.posX = bin[robot.robotID][1];

    info.posY = bin[robot.robotID][2];

    info.radius = 0;
    info.targetX = 0;
    info.targetY = 0;

    this->coordinatorUpdatePublisher.publish(info);

    pt.start();

}*/
