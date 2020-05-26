#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include "thread_class.h"
thread_class *th_p;

using namespace std;

//******************************************
//
// callback functions
//
//******************************************
#include <std_srvs/SetBool.h>
bool process(std_srvs::SetBool::Request &req,
             std_srvs::SetBool::Response &res);
#include <std_msgs/Int32.h>
void callback(const std_msgs::Int32::ConstPtr& msg);

//******************************************
//
// signal handler
//
//******************************************
#include <signal.h>
void sig_handler(int signo)
{
 	cerr << "program closing..." << endl;
  exit(1);
}
//******************************************
//
// main function
//
//******************************************
int main (int argc, char **argv)
{
  // init ROS node
  ros::init (argc, argv, "main_node");
  ros::NodeHandle node;

  // set spin rate
  ros::Rate loop_rate(30);

	// set signal handler
  if (signal(SIGINT, sig_handler) == SIG_ERR)
    cerr << "can't catch SIGINT" << endl;
  if (signal(SIGHUP, sig_handler) == SIG_ERR)
    cerr << "can't catch SIGHUP" << endl;

	// service server
  ros::ServiceServer srv = node.advertiseService("/process", process);

	// subscriber
	ros::Subscriber sub = node.subscribe("/task", 1, callback);

	// thread class
	thread_class th;
	th_p = &th;

  while (ros::ok ())
  {
//		cerr << "main program" << endl;

		ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//******************************************
//
// Service server callback function
//
//******************************************
bool process(std_srvs::SetBool::Request &req,
             std_srvs::SetBool::Response &res)
{
	// スレッドの生成・終了
  if (req.data)
    res.success = th_p->run_process(0.0035);	// period [sec]
  else
    res.success = th_p->kill_process();

	res.message = "";
	return true;
}
//******************************************
//
// Subscriber callback function
//
//******************************************
void callback(const std_msgs::Int32::ConstPtr& msg)
{
	cerr << "callback" << endl;
	
	// 前処理
	th_p->setExecTask(msg->data);
	th_p->run_exec(15);		// スレッド数を指定して実行
	
	// タスク設定
	th_p->setProcessTask(msg->data);

}
