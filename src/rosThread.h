
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "navigationController.h"
#include "navigationISLH/robotInfo.h"
#include "navigationISLH/neighborInfo.h"
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QFile>
#include <QDateTime>
#include <QtNetwork/QtNetwork>
#include <QXmlStreamReader>
#include <QtCore/QByteArray>


#define numOfRobots 2

class Robot
{
public:
    int robotID;
    bool isCoordinator;
    double radius;
    double targetX;
    double targetY;
    //double initialX;
    //double initialY;

};
/*class Obstacle
{
public:
    int id;
    double radius;
    double x;
    double y;


};*/

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    Robot robot;

    //QVector<Obstacle> obstacles;

   // RosThread(int argc, char **argv, std::string nodeName);

public:

     bool readConfigFile(QString filename);

private:
     bool shutdown;

  //   navigationISL::robotInfo currentStatus;

     bool startNavigation;

     void startModule();

     ros::NodeHandle n;

     ros::Subscriber poseListSub;

     ros::Subscriber targetPoseListSub;
     ros::Subscriber turtlebotGyroSub;
     ros::Subscriber turtlebotOdomSub;

     //ros::Subscriber neighborInfoSubscriber;

     ros::Subscriber turtlebotOdometrySubscriber;

     //ros::Publisher robotinfoPublisher;

     //ros::Publisher coordinatorUpdatePublisher;

     ros::Publisher turtlebotVelPublisher;

     QTcpSocket* socket;
     ros::Timer timer;

     //ros::Publisher amclInitialPosePublisher;

     void poseListCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

     void targetPoseListCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

     void turtlebotOdometryCallback(const nav_msgs::Odometry::ConstPtr & msg);

     void turtlebotGyroCallback(const sensor_msgs::Imu::ConstPtr & msg);

     void turtlebotOdomCallback(const nav_msgs::Odometry::ConstPtr & msg);

     //void neighborInfoCallback(navigationISLH::neighborInfo neighborInfo);

     //void poseUpdate(const ros::TimerEvent&);

     //void coordinatorUpdate(const ros::TimerEvent&);

     void robotContoller(double [], int , double [][4], double [][3], double [][4], double, double []);

     void calculateTurn(double desired, double current);

     void sendVelocityCommand();

     void timerTick(const ros::TimerEvent&);


     double vel[2]; // velocity vector
     double bin[numOfRobots+1][4];// positions including itself
     double bt[numOfRobots+1][3]; // goal positions
     double rr[numOfRobots+1]; // radii of the robots
     double b_rs[numOfRobots+1][4]; // robots' positions within sensing range
     double ro;
     double kkLimits[2]; // upper and lower bounds of parameters in navigation function
     //double bp[5][4];

     double radYaw;

     QFile poseFile;

     //int poseUpdatePeriod;
     //int coordinatorUpdatePeriod;

     // The robot's angle threshold while rotating in degrees
     double angleThreshold;

     // The robot's distance threshold for goal achievement in cms
     int distanceThreshold;

     // in m/sec linear velocity
     double linearVelocity;

     // in rad/sec rotational velocity
     double angularVelocity;

     int numrobots;

     int partDist;

     bool feedbackToServer;

     QString IP;

     // Pose update timer
     //ros::Timer pt;

     // Coordinator update timer
     //ros::Timer ct;

     geometry_msgs::Twist velocityVector;

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
