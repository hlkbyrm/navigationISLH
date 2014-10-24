#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include "navigationController.h"
#include "ISLH_msgs/robotPose.h"
#include "ISLH_msgs/robotPositions.h"
#include "ISLH_msgs/targetPoseListMessage.h"
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QFile>
#include <QDateTime>
#include <QtNetwork/QtNetwork>
#include <QXmlStreamReader>
#include <QtCore/QByteArray>
#include <vector>

class Robot
{
public:
    int robotID;
    bool isCoordinator;
    double radius;
    double targetX;
    double targetY;
};

class RosThread:public QObject
{
    Q_OBJECT

public:
    RosThread();

    Robot robot;

public:

     bool readConfigFile(QString filename);

private:
     bool startNavigation;
     bool targetReached;

     void startModule();

     ros::NodeHandle n;

     ros::Subscriber poseListSub;
     ros::Subscriber navigationOKSub;
     ros::Subscriber targetPoseListSub;
     ros::Subscriber targetPoseListFromMonitoringSub;
     ros::Subscriber targetPoseSub;
     ros::Subscriber turtlebotGyroSub;
     ros::Subscriber turtlebotOdomSub;
     ros::Subscriber turtlebotOdometrySubscriber;
     ros::Publisher turtlebotVelPublisher;
     ros::Publisher robotPosePublisher;
     ros::Publisher targetReachedPublisher;
     ros::Publisher currentPosePublisher;

     QTcpSocket* socket;
     ros::Timer timer;

     void poseListCallback(const ISLH_msgs::robotPositions::ConstPtr &msg);
     void navigationOKCallback(const std_msgs::UInt8::ConstPtr &msg);
     void targetPoseListCallback(const ISLH_msgs::targetPoseListMessage::ConstPtr &msg);
     void targetPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
     void turtlebotOdometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
     void turtlebotGyroCallback(const sensor_msgs::Imu::ConstPtr & msg);
     void turtlebotOdomCallback(const nav_msgs::Odometry::ConstPtr & msg);

     void robotContoller(double [], int , double [][4], double [][3], double [][4], double, double []);

     void calculateTurn(double desired, double current, geometry_msgs::Twist *twist);

     void sendVelocityCommand();

     std::vector<std::vector<double> > findRobotsInRange();

     void timerTick(const ros::TimerEvent&);


     double vel[2]; // velocity vector
     std::vector<std::vector<double> > bin;
     std::vector<std::vector<double> > bt;
     std::vector<double> rr;
     std::vector<std::vector<double> > b_rs;
     double ro; // workspace radius
     double rs; // sensing range
     double kkLimits[2]; // upper and lower bounds of parameters in navigation function
     bool isKobuki;
     double stoppingThreshold;
     bool isFinished;
     bool turning; // Flag for not using camera data when turtlebot is turning
     bool turning2;// Flag for not using camera data when turtlebot is turning
     bool firstDataCame; // Flag for first data because we must not use gyro or odom before first data
     bool firstTargetCame; // Flag for first target because we must not start navigating before first target
     ros::Time current_timeO, last_timeO; // Used for converting velocity to distance or angle
     ros::Time current_timeG, last_timeG; // Used for converting velocity to distance or angle

     double radYaw;

     QFile poseFile;

     // The robot's angle threshold while rotating in degrees
     double angleThreshold;

     // The robot's distance threshold for goal achievement in cms
     int distanceThreshold;

     // in m/sec linear velocity
     double linearVelocity;

     // in rad/sec rotational velocity
     double angularVelocity;

     int numrobots;

     int queueSize;

     bool feedbackToServer;

     QString IP;

     geometry_msgs::Twist velocityVector;

public slots:
     void work();
     void shutdownROS();

signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
