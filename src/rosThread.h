
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "navigationController.h"
#include "navigationISLH/robotInfo.h"
#include "navigationISLH/neighborInfo.h"
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QFile>
#include <QDateTime>


#define numOfRobots 5

class Robot
{
public:
    int robotID;
    bool isCoordinator;
    double radius;
    double targetX;
    double targetY;
    double initialX;
    double initialY;

};
class Obstacle
{
public:
    int id;
    double radius;
    double x;
    double y;


};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();

    Robot robot;

    QVector<Obstacle> obstacles;

   // RosThread(int argc, char **argv, std::string nodeName);

public:

     bool readConfigFile(QString filename);

private:
     bool shutdown;

  //   navigationISL::robotInfo currentStatus;

     bool startNavigation;

     void startModule();

     ros::NodeHandle n;

     ros::Subscriber amclSub;

     ros::Subscriber neighborInfoSubscriber;

     ros::Subscriber turtlebotOdometrySubscriber;

     ros::Publisher robotinfoPublisher;

     ros::Publisher coordinatorUpdatePublisher;

     ros::Publisher turtlebotVelPublisher;

     ros::Publisher amclInitialPosePublisher;

     void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

     void turtlebotOdometryCallback(const nav_msgs::Odometry & msg);

     void neighborInfoCallback(navigationISLH::neighborInfo neighborInfo);

     void poseUpdate(const ros::TimerEvent&);

     void coordinatorUpdate(const ros::TimerEvent&);

     void robotContoller(double [], int , double [][4], double [][3], double [][4], double, double []);

     void calculateTurn(double desired, double current);

     void sendVelocityCommand();


     double vel[2]; // velocity vector
     double bin[numOfRobots+1][4];// positions including itself
     double bt[numOfRobots+1][3]; // goal positions
     double rr[numOfRobots+1]; // radii of the robots
     double b_rs[numOfRobots+1][4]; // robots' positions within sensing range
     double ro;
     double kkLimits[2]; // upper and lower bounds of parameters in navigation function
     double bp[5][4];

     QFile poseFile;

     int poseUpdatePeriod;
     int coordinatorUpdatePeriod;

     // The robot's angle threshold while rotating in degrees
     int angleThreshold;

     // The robot's distance threshold for goal achievement in cms
     int distanceThreshold;

     // in m/sec linear velocity
     double linearVelocity;

     // in rad/sec rotational velocity
     double angularVelocity;

     int numrobots;

     int partDist;

     // Pose update timer
     ros::Timer pt;

     // Coordinator update timer
     ros::Timer ct;

     geometry_msgs::Twist velocityVector;

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void  rosStarted();
   void  rosStartFailed();

};
