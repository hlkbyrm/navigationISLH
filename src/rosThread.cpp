#include <tf/tf.h>
#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>
#include <QtNetwork/QHostInfo>

RosThread::RosThread()
{
}
void RosThread::startModule()
{
}
void RosThread::work(){
    startNavigation = false;
    firstTargetCame = false;
    //it will be false when first target came
    targetReached = true;

    //First initializations
    turning = false;
    turning2 = false;
    firstDataCame =false;
    velocityVector.linear.x = 0.0;
    radYaw = 0.0;
    vel[0] = 0.0;
    vel[1] = 0.0;
    isFinished = false;

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

    if(!ros::ok()){
        qDebug()<< "ROS Init failed!!!";
        ros::shutdown();
        emit rosFinished();
        return;
    }

    emit rosStarted();
    this->poseListSub = n.subscribe("localizationISLH/poseList",1,&RosThread::poseListCallback,this);
    this->navigationOKSub = n.subscribe("taskHandlerISLH/navigationOK",queueSize,&RosThread::navigationOKCallback,this);
    this->targetPoseListSub = n.subscribe("messageDecoderISLH/targetPoseList",2,&RosThread::targetPoseListCallback,this);
    this->targetPoseListFromMonitoringSub = n.subscribe("monitoringISLH/targetPoseList",2,&RosThread::targetPoseListCallback,this);
    this->targetPoseSub = n.subscribe("taskHandlerISLH/targetPose",2,&RosThread::targetPoseCallback,this);
    //publisher değiştirildi safety controller eklendi. minimal launchera ek olarak safe_keyop.launch çalıştırılması gerekiyor
    if(!isKobuki){
        if (!isSafety)
            this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
        else
            this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("/raw_velocity",1);
    }
    else
        this->turtlebotVelPublisher = n.advertise<geometry_msgs::Twist>("/keyop_vel_smoother/raw_cmd_vel",1);
    this->turtlebotGyroSub = n.subscribe("/mobile_base/sensors/imu_data",1,&RosThread::turtlebotGyroCallback,this);
    this->turtlebotOdomSub = n.subscribe("/odom",1,&RosThread::turtlebotOdomCallback,this);
    this->robotPosePublisher = n.advertise<ISLH_msgs::robotPose>("navigationISLH/robotPositionInfo",queueSize,true);
    this->targetReachedPublisher = n.advertise<std_msgs::UInt8>("navigationISLH/targetReached",queueSize,true);
    this->currentPosePublisher = n.advertise<geometry_msgs::Pose2D>("navigationISLH/currentPose",1,true);

    //Send robot pose and target info 4 times each second
    timer = n.createTimer(ros::Duration(0.25),&RosThread::timerTick,this);
    timer.start();

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(startNavigation && firstTargetCame) {
            // Use empty vector for obstacles since we don't have any obstacles
            std::vector<std::vector<double> > empty;
            double lengthOfVel;

            std::vector<std::vector<double> > bin_;
            bin_ = findRobotsInRange();

            NavigationController::robotContoller(vel, &lengthOfVel, numrobots, 0, 0, bin_, bt, b_rs, empty, ro, kkLimits, robot.robotID);

            qDebug()<<"lengthOfVel: "<<lengthOfVel;
            qDebug()<<"vel[0]"<<vel[0]<<"vel[1]"<<vel[1];

            //If length is smaller than threshold then stop the robot
            //Otherwise don't stop it even it reaches to its target
            if(lengthOfVel < stoppingThreshold)
                isFinished = true;
            else
                isFinished = false;

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
}
// find the robots in its range. These robots are taken into account in the navigation controller
std::vector<std::vector<double> >  RosThread::findRobotsInRange()
{

    std::vector<std::vector<double> > bin_;

    bin_ = std::vector<std::vector<double> >(numrobots+1,std::vector<double>(4));

    int myRID = robot.robotID;
    for(int rID=1;rID<=numrobots;rID++)
    {
        bin_[rID][1] = bin[rID][1];
        bin_[rID][2] = bin[rID][2];
        bin_[rID][3] = bin[rID][3];

        double dist = sqrt((bin[myRID][1]-bin[rID][1])*(bin[myRID][1]-bin[rID][1]) + (bin[myRID][2]-bin[rID][2])*(bin[myRID][2]-bin[rID][2]));

        if (dist>rs)
            bin_[rID][3] = 0;
    }

    return bin_;

}

void RosThread::navigationOKCallback(const std_msgs::UInt8::ConstPtr &msg){
    if(msg->data == 0)
        startNavigation = false;
    else
        startNavigation = true;
}
//Get targets of all robots
void RosThread::targetPoseListCallback(const ISLH_msgs::targetPoseListMessage::ConstPtr& msg){
    for(int i=0;i<msg->targetPoses.size();i++){
        bt[msg->robotIDs[i]][1] = msg->targetPoses[i].x;
        bt[msg->robotIDs[i]][2] = msg->targetPoses[i].y;
    }

    robot.targetX = bt[robot.robotID][1];
    robot.targetY = bt[robot.robotID][2];
}
//Get target of robot
void RosThread::targetPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg){
    firstTargetCame = true;
    targetReached = false;
    bt[robot.robotID][1] = msg->x;
    bt[robot.robotID][2] = msg->y;
    robot.targetX = msg->x;
    robot.targetY = msg->y;

    qDebug() << "TargetX: " << bt[robot.robotID][1] << " Y: " << bt[robot.robotID][2];
}
//Use odom data to correct position between two poseListCallback
void RosThread::turtlebotOdomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    if(!firstDataCame) return;
    current_timeO = ros::Time::now();
    double diffx = msg->twist.twist.linear.x * 100*((current_timeO - last_timeO).toSec());
    qDebug() << "twist X: " << diffx << " +X: " << diffx*cos(radYaw) << " +Y: " << diffx*sin(radYaw);
    bin[robot.robotID][1] += diffx*cos(radYaw);
    bin[robot.robotID][2] += diffx*sin(radYaw);
    last_timeO = current_timeO;
}
//Use gyro data to correct angle between two poseListCallback
void RosThread::turtlebotGyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(!firstDataCame) return;
    current_timeG = ros::Time::now();
    qDebug() << "Gyro X:" << msg->angular_velocity.x << "Y:" << msg->angular_velocity.y << "Z:" << msg->angular_velocity.z;
    radYaw += msg->angular_velocity.z*((current_timeG - last_timeG).toSec());
    last_timeG = current_timeG;

    double calYaw = atan2(vel[1],vel[0]);
    if(calYaw < 0)
        calYaw += M_PI*2;
    if(radYaw < 0)
        radYaw += M_PI*2;
    if(radYaw > M_PI*2)
        radYaw -= M_PI*2;

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    //if we reached to our target send targetReached message and return from function
    double dist2Target = sqrt((robot.targetX-bin[robot.robotID][1])*(robot.targetX-bin[robot.robotID][1]) + (robot.targetY-bin[robot.robotID][2])*(robot.targetY-bin[robot.robotID][2]));
    if (dist2Target <= distanceThreshold){
        if(!targetReached){
            std_msgs::UInt8 msg;
            msg.data = 1;
            targetReachedPublisher.publish(msg);
            targetReached = true;
        }
    }

    if(isFinished && dist2Target <= distanceThreshold){
 /*       if(!targetReached){
            std_msgs::UInt8 msg;
            msg.data = 1;
            targetReachedPublisher.publish(msg);
            targetReached = true;
        }
        */
        velocityVector = twist;
        return;
    }

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
    //if difference between our angle and target angle is too big then stop and turn
    if((diff > angleThreshold*M_PI/180.0 && !turning) ||
            (diff > angleThreshold/2*M_PI/180.0 && turning))
    {
        turning = true;
        calculateTurn(calYaw,radYaw,&twist);

        qDebug() << "bin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        qDebug()<<"LinearVel:X:"<<twist.linear.x<<"Y:"<<twist.linear.y<<"Z:"<<twist.linear.z;
        qDebug()<<"AngularVel:X:"<<twist.angular.x<<"Y:"<<twist.angular.y<<"Z:"<<twist.angular.z;
    }
    else
    {
        turning = false;
        // if robot is not at target position then continue
        double dist2Target = sqrt((robot.targetX-bin[robot.robotID][1])*(robot.targetX-bin[robot.robotID][1]) + (robot.targetY-bin[robot.robotID][2])*(robot.targetY-bin[robot.robotID][2]));
        if(!isFinished || dist2Target > distanceThreshold){
            twist.linear.x = linearVelocity;
            // if robots direction is not correct then fix it
            if((diff > angleThreshold/5*M_PI/180.0 && !turning))
                calculateTurn(calYaw,radYaw,&twist);
        }
        // if robot is at target position then stop
        else
            twist.linear.x = 0;

        qDebug() << "bin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        qDebug()<<"LinearVel:X:"<<twist.linear.x<<"Y:"<<twist.linear.y<<"Z:"<<twist.linear.z;
        qDebug()<<"AngularVel:X:"<<twist.angular.x<<"Y:"<<twist.angular.y<<"Z:"<<twist.angular.z;
    }
    velocityVector = twist;
}
//Get pose data from main computer and correct the data using odom and gyro
void RosThread::poseListCallback(const ISLH_msgs::robotPositions::ConstPtr& msg)
{
    // If its first data then reset time variables
    if(!firstDataCame) {
        current_timeG = ros::Time::now();
        last_timeG = ros::Time::now();
        current_timeO = ros::Time::now();
        last_timeO = ros::Time::now();
    }
    firstDataCame = true;

    //change pose datas of robots
    for(int i=0;i<msg->positions.size();i++){
        bin[i+1][1] = msg->positions[i].x;
        bin[i+1][2] = msg->positions[i].y;
        bin[i+1][3] = robot.radius;
    }

    if(!poseFile.exists())
        poseFile.open(QFile::WriteOnly);
    else
        poseFile.open(QFile::Append);
    QTextStream stream(&poseFile);
    stream<<QDateTime::currentDateTime().toTime_t()<<" "<<bin[robot.robotID][1]<<" "<<bin[robot.robotID][2]<<" "<<robot.radius<<" ";
    if(!turning){
        if(!turning2){
            //if robot is not turning then get angle from main computer(msg)
            radYaw = msg->directions[robot.robotID-1];
            qDebug() << "MRad yaw: " << radYaw;
            stream << "M ";
        }
        turning2 = false;
    }else turning2 = true;
    double calYaw = atan2(vel[1],vel[0]);

    if(calYaw < 0)
        calYaw += M_PI*2;
    if(radYaw < 0)
        radYaw += M_PI*2;

    stream << radYaw << " " << calYaw << " " << (turning?"Y":"N") << "\n";

    poseFile.close();

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    //if we reached to our target send targetReached msg and return from function
    double dist2Target = sqrt((robot.targetX-bin[robot.robotID][1])*(robot.targetX-bin[robot.robotID][1]) + (robot.targetY-bin[robot.robotID][2])*(robot.targetY-bin[robot.robotID][2]));
    if (dist2Target <= distanceThreshold){
            if(!targetReached){
                std_msgs::UInt8 _msg;
                _msg.data = 1;
                targetReachedPublisher.publish(_msg);
                targetReached = true;
            }
    }

    if(isFinished && dist2Target <= distanceThreshold){
/*        if(!targetReached){
            std_msgs::UInt8 _msg;
            _msg.data = 1;
            targetReachedPublisher.publish(_msg);
            targetReached = true;
        }
*/
        velocityVector = twist;
        return;
    }

    qDebug() << "radYaw : " << radYaw << " calYaw : " << calYaw;
    qDebug() << "fabs(radYaw-calYaw)" << fabs(radYaw-calYaw);
    qDebug() << "angleThreshold*M_PI/180.0" << angleThreshold*M_PI/180.0;
    qDebug() << "turning : " << (turning?"true":"false");
    double diff = fabs(radYaw - calYaw);
    if(diff > M_PI){
        diff -= 2*M_PI;
        diff = fabs(diff);
    }
    //if difference between our angle and target angle is too big then stop and turn
    if((diff > angleThreshold*M_PI/180.0 && !turning) ||
            (diff > angleThreshold/2*M_PI/180.0 && turning) )
    {
        turning = true;
        calculateTurn(calYaw,radYaw,&twist);

        qDebug() << "Mbin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        qDebug()<<"LinearVel:X:"<<twist.linear.x<<"Y:"<<twist.linear.y<<"Z:"<<twist.linear.z;
        qDebug()<<"AngularVel:X:"<<twist.angular.x<<"Y:"<<twist.angular.y<<"Z:"<<twist.angular.z;
    }
    else
    {
        turning = false;
        // if robot is not at target position then continue
        double dist2Target = sqrt((robot.targetX-bin[robot.robotID][1])*(robot.targetX-bin[robot.robotID][1]) + (robot.targetY-bin[robot.robotID][2])*(robot.targetY-bin[robot.robotID][2]));
        if(!isFinished || dist2Target > distanceThreshold){
            twist.linear.x = linearVelocity;
            // if robots direction is not correct then fix it
            if((diff > angleThreshold/5*M_PI/180.0 && !turning))
                calculateTurn(calYaw,radYaw,&twist);
        }
        // if robot is at target position then stop
        else
            twist.linear.x = 0;

        qDebug() << "Mbin[robot.RobotID][1] : " << bin[robot.robotID][1] << "[2] : " << bin[robot.robotID][2] << "[3] : " << bin[robot.robotID][3];
        qDebug() << "bt[robot.RobotID][1]  : " << bt[robot.robotID][1]  << "[2] : " << bt[robot.robotID][2];
        qDebug()<<"LinearVel:X:"<<twist.linear.x<<"Y:"<<twist.linear.y<<"Z:"<<twist.linear.z;
        qDebug()<<"AngularVel:X:"<<twist.angular.x<<"Y:"<<twist.angular.y<<"Z:"<<twist.angular.z;
    }
    velocityVector = twist;
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

        this->robot.radius = result["robotRadius"].toDouble();

        qDebug()<<result["robotRadius"].toString();

        this->linearVelocity = result["linearVelocity"].toDouble();

        qDebug()<<result["linearVelocity"].toDouble();

        this->isKobuki = result["isKobuki"].toInt() == 1;

        this->isSafety = result["isSafety"].toInt() == 1;

        this->angularVelocity = result["angularVelocity"].toDouble();

        this->angleThreshold = result["angleThreshold"].toDouble();

        this->distanceThreshold = result["distanceThreshold"].toInt();

        qDebug()<<distanceThreshold;

        this->stoppingThreshold = result["stoppingThreshold"].toDouble();

        qDebug()<<stoppingThreshold;

        //ro = result["ro"].toInt();
        ro = result["ro4Navigation"].toDouble();

        qDebug()<<"ro4Navigation: "<<ro;

        rs = result["rs"].toDouble();

        kkLimits[0] = result["kMin"].toInt();

        qDebug()<<kkLimits[0];

        kkLimits[1] = result["kMax"].toInt();

        qDebug()<<kkLimits[1];

        robot.robotID = result["robotID"].toInt();

        qDebug()<<result["robotID"].toString();

        feedbackToServer = result["feedbackToServer"].toInt() == 1;

        qDebug() << "feedbackToServer: " << (feedbackToServer?"true":"false");

        IP = result["IP"].toString();

        qDebug() << "IP: " << IP;

        int coordinatorRobotID =   result["taskCoordinatorRobotID"].toInt();
        if(coordinatorRobotID == robot.robotID) this->robot.isCoordinator = true;

        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();

        this->robot.targetX = result["targetX"].toDouble();

        qDebug()<<result["targetX"].toString();

        this->robot.targetY = result["targetY"].toDouble();

        qDebug()<<result["targetY"].toString();

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

    return true;
}
// Sends the velocity command to robot
void RosThread::sendVelocityCommand() {
    turtlebotVelPublisher.publish(velocityVector);
}
// calculate the direction of turning
void RosThread::calculateTurn(double desired, double current, geometry_msgs::Twist *twist) {
    int dir = 0;

    double diff = desired-current;

    if(diff > 0 )
        dir = 1;
    else
        dir = -1;

    if(fabs(M_PI*2 - fabs(diff)) < fabs(diff))
        dir = dir*-1;

    (*twist).angular.z = dir*angularVelocity;
}
// publish position of robot and target of robot to monitoringISLH
void RosThread::timerTick(const ros::TimerEvent&){
    if(feedbackToServer){
        ISLH_msgs::robotPose rp;
    
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

    geometry_msgs::Pose2D pose;
    pose.x = bin[robot.robotID][1];
    pose.y = bin[robot.robotID][2];
    this->currentPosePublisher.publish(pose);
}
