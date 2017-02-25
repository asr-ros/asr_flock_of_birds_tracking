/**

Copyright (c) 2016, Engelmann Stephan, Heller Florian, Meißner Pascal, Stöckle Patrick, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <asr_msgs/AsrObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <cv.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <termios.h>

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (1 / RAD_TO_DEG)

class PTUTracker {
public:
        ros::NodeHandle nh;

        // Publications / Subscriptions
        ros::Subscriber fob_sub;
        ros::Subscriber ptu_sub;
        ros::Publisher ptu_pub;

        //Param names
        std::string ptu_state_cmd, ptu_state, fob_topic, fob_tracking_point_left, fob_tracking_point_right;
        int panAmount, tiltAmount;

        //States
        std::vector<double> ptuPosition;

        geometry_msgs::Pose fob_pose_left;
        geometry_msgs::Pose fob_pose_right;

        geometry_msgs::PointStamped ptuPoint;
        geometry_msgs::PointStamped fobPoint;

        struct termios oldt;


        tf::TransformListener tf;
        tf::TransformBroadcaster tb;

        //FOB Callback
        void fobCB(const asr_msgs::AsrObject& fob_msg) {

	  if(!fob_msg.sampledPoses.size()){
        std::cerr << "Got a AsrObject without poses." << std::endl;
	    std::exit(1);    
	  }

	  if (!fob_msg.identifier.empty() && strcmp(fob_msg.identifier.c_str(), fob_tracking_point_left.c_str()) == 0) {
	    ROS_DEBUG("fob left message recieved");
	    fob_pose_left = fob_msg.sampledPoses.front().pose;
	  }else if (!fob_msg.identifier.empty() && strcmp(fob_msg.identifier.c_str(), fob_tracking_point_right.c_str()) == 0) {
	    ROS_DEBUG("fob right message recieved");
	    fob_pose_right = fob_msg.sampledPoses.front().pose;
	  }
        }

        //PTU Callback
        void ptuCB(const sensor_msgs::JointState& ptu_msg) {
            ptuPosition = ptu_msg.position;
        }

        //Initializations and configure
        PTUTracker(ros::NodeHandle nh){
            this->nh = nh;
            nh.getParam("ptu_pub_topic", ptu_state_cmd);
            nh.getParam("ptu_sub_topic", ptu_state);
            nh.getParam("fob_sub_topic", fob_topic);
            nh.getParam("fob_tracking_point_left", fob_tracking_point_left);
            nh.getParam("fob_tracking_point_right", fob_tracking_point_right);

            nh.getParam("panAmount", panAmount);
            nh.getParam("tiltAmount", tiltAmount);

            std::string fob_frame, ptu_frame;
            nh.getParam("tracker_frame", fob_frame);
            nh.getParam("ptu_base_frame", ptu_frame);

            fob_sub = nh.subscribe(fob_topic, 1, &PTUTracker::fobCB, this);
            ptu_sub = nh.subscribe("/asr_flir_ptu_driver/state", 1, &PTUTracker::ptuCB, this);
            ptu_pub = nh.advertise<sensor_msgs::JointState>(ptu_state_cmd, 5);

            fobPoint.header.frame_id = fob_frame;
            fobPoint.header.stamp = ros::Time(0);
            ptuPoint.header.frame_id = ptu_frame;
            ptuPoint.header.stamp = ros::Time(0);

            boost::thread trackerThread(spin, this);
            ros::spin();
            trackerThread.interrupt();

            //reapply the old Terminal settings
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        }

        cv::Point2f toSphereCoords(geometry_msgs::PointStamped p) {
            return toSphereCoords(cv::Point3f(p.point.x, p.point.y, p.point.z));
        }

        cv::Point2f toSphereCoords(cv::Point3f pt) {
            cv::Point2f pantilt;
            cv::Point3f p;
            p.x = -pt.y;
            p.y = -pt.x;
            p.z = -pt.z;
            double panAngle = (p.x < 0)? 1 : -1;
            panAngle *= acos(-p.y / (sqrt(p.x * p.x + p.y * p.y))) * RAD_TO_DEG;

            double tiltAngle = (p.z > 0)? -1 : 1;
            tiltAngle *= acos(-p.y / (sqrt(p.y * p.y + p.z * p.z))) * RAD_TO_DEG;

            pantilt.x = panAngle;
            pantilt.y = tiltAngle;

            return pantilt;
        }

        void movePTU(char c){
            if(c!='l' && c !='r'){
                    return;
            }

            //transformation into sphere coords is only possible if the point is in the PTU frame
            if(c == 'l'){
                fobPoint.point.x = fob_pose_left.position.x;
                fobPoint.point.y = fob_pose_left.position.y;
                fobPoint.point.z = fob_pose_left.position.z;
            }else if(c == 'r'){
                fobPoint.point.x = fob_pose_right.position.x;
                fobPoint.point.y = fob_pose_right.position.y;
                fobPoint.point.z = fob_pose_right.position.z;
            }
            tf.transformPoint(ptuPoint.header.frame_id, fobPoint, ptuPoint);
            cv::Point2f pantiltTarget = toSphereCoords(ptuPoint);
            rotPTUAbsolute(pantiltTarget.x, pantiltTarget.y);

        }
        void rotPTUAbsolute(double pan, double tilt){
            ROS_INFO("Target pan tilt angle: ([%f  %f])", pan, tilt);

            sensor_msgs::JointState pantiltState;
            pantiltState.header.stamp = ros::Time::now();

            pantiltState.name.push_back("pan");
            pantiltState.name.push_back("tilt");

            pantiltState.position.push_back(pan);
            pantiltState.position.push_back(tilt);

            pantiltState.velocity.push_back(0);
            pantiltState.velocity.push_back(0);

            ptu_pub.publish(pantiltState);
        }
        void rotPTURelative(double pan, double tilt){
            pan = pan + ptuPosition[0];
            tilt = tilt + ptuPosition[1];
            rotPTUAbsolute(pan, tilt);
        }

        int getch(void)
        {
            //overlays echo from Terminal
            int ch;
            struct termios newt;
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            ch = getchar();
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return ch;
        }

        int getKey()
        {
           int result=getch();
           if(result==0){
              result=256+getch();
           }
           return result;
        }

        static void spin(PTUTracker* tracker){
            std::string input;
            ROS_INFO("Press <l> to move PTU to left tracker.");
            ROS_INFO("Press <r> to move PTU to right tracker.");
            ROS_INFO("Use Arrowkeys to move PTU (do not hold key down).");

            while (true){
               char c = tracker->getKey();

               if(c == 'l' || c == 'r'){
                    tracker->movePTU(c);
               }else if(c == 65){
                    tracker->rotPTURelative(0,tracker->tiltAmount);
               }else if(c == 66){
                   tracker->rotPTURelative(0,-tracker->tiltAmount);
               }else if(c == 67){
                   tracker->rotPTURelative(-tracker->panAmount,0);
               }else if(c == 68){
                   tracker->rotPTURelative(tracker->panAmount,0);
               }
               boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ptu_tracker");
  ros::NodeHandle nh("~");
  new ::PTUTracker(nh);
  return 0;
}
