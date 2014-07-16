#include <ros/ros.h>
#include <QApplication>
#include <rosThread.h>
#include <QThread>



int main(int argc,char** argv){

    QApplication app(argc,argv);

    ros::init(argc,argv,"navigationISLH");

    RosThread* rosthread  = new RosThread;

    QThread* worker = new QThread(&app);

    rosthread->moveToThread(worker);

   // QObject(&app,SIGNAL(aboutToQuit()),rosthread,SLOT(shutdownROS()));

    QObject::connect(rosthread,SIGNAL(rosFinished()),worker,SLOT(quit()));
    QObject::connect(worker,SIGNAL(finished()),&app,SLOT(quit()));

    QObject::connect(worker,SIGNAL(finished()),rosthread,SLOT(deleteLater()));

    QObject::connect(worker,SIGNAL(started()),rosthread,SLOT(work()));


    worker->start();


    //ros::spin();

  //  ros::Rate loop_rate(10);

   // QTimer* time = new QTimer(0);

  //  int count = 0;
  /*  while (ros::ok())
    {

        robotContoller(vel, numOfRobots, bin, bt, b_rs, ro, kkLimits);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;*/

    return app.exec();
}


