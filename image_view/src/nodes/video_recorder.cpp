/****************************************************************************
 * Software License Agreement (Apache License)
 *
 *     Copyright (C) 2012-2013 Open Source Robotics Foundation
 *
 *     Licensed under the Apache License, Version 2.0 (the "License");
 *     you may not use this file except in compliance with the License.
 *     You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 *
 *****************************************************************************/

#include "ros/service_client.h"
#include "ros/service_server.h"
#include "sensor_msgs/Image.h"
#include <climits>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#if CV_MAJOR_VERSION == 3
#include <opencv2/videoio.hpp>
#endif

#include "std_srvs/Empty.h"
#include "image_view/video_controller.h"

cv::VideoWriter outputVideo;
int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;

sensor_msgs::ImageConstPtr img_ptr = boost::make_shared<sensor_msgs::Image>();


// TODO Sequence
// "Start" -> create file + record (bool new_file = false)
// "Stop" -> bool new_file = true 


// Marcus add
bool service_start = false;
bool first_frame = false; // to get dimensions
// 1 = start
// 2 = stop + save
// 3 = save screen
bool service_processor(image_view::video_controller::Request &req,
				image_view::video_controller::Response &res){
		if(req.cmd == 1){
				ROS_INFO("START VIDEO");
				service_start = true;
		}
		if(req.cmd == 0){
				ROS_INFO("STOP + SAVE VIDEO");
				service_start = false;


				//File Record TODO bolar uma forma de criar arquivo dps do primeiro frame
				std::size_t found = filename.find_last_of("/\\");
				std::string path = filename.substr(0, found + 1);
				std::string basename = filename.substr(found + 1);
				std::stringstream ss;
				ss << ros::Time::now().toNSec() << basename;
				std::string filename_now = path + ss.str();
				ROS_INFO("Video recording to %s ...", filename_now.c_str());

				cv::Size size(img_ptr->width, img_ptr->height);

				outputVideo.open(filename_now, 
#if CV_MAJOR_VERSION >= 3
								cv::VideoWriter::fourcc(codec.c_str()[0],
#else
										CV_FOURCC(codec.c_str()[0],
#endif
												codec.c_str()[1],
												codec.c_str()[2],
												codec.c_str()[3]), 
										fps,
										size,
										true);

								if (!outputVideo.isOpened())
								{
								ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
								exit(-1);
								}
								g_count = 0;
								ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

		}



		return true;
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
		img_ptr = image_msg;
		first_frame = true;

		if(service_start == false)
				return;

		//		if (!outputVideo.isOpened()) {

		if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1.0 / fps))
		{
				// Skip to get video with correct fps
				return;
		}

		try
		{
				cv_bridge::CvtColorForDisplayOptions options;
				options.do_dynamic_scaling = use_dynamic_range;
				options.min_image_value = min_depth_range;
				options.max_image_value = max_depth_range;
				options.colormap = colormap;
				const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
				if (!image.empty() ) {
						outputVideo << image;
						ROS_INFO_STREAM("Recording frame " << g_count << "\x1b[1F");
						g_count++;
						g_last_wrote_time = image_msg->header.stamp;
				} else {
						ROS_WARN("Frame skipped, no data!");
				}
		} catch(cv_bridge::Exception)
		{
				ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
				return;
		}
}

int main(int argc, char** argv)
{
		ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
		ros::NodeHandle nh;
		ros::NodeHandle local_nh("~");
		local_nh.param("filename", filename, std::string("output.avi"));
		bool stamped_filename;
		// local_nh.param("stamped_filename", stamped_filename, false); //always stamped
		local_nh.param("fps", fps, 15);
		local_nh.param("codec", codec, std::string("MJPG"));
		local_nh.param("encoding", encoding, std::string("bgr8"));
		// cv_bridge::CvtColorForDisplayOptions
		local_nh.param("min_depth_range", min_depth_range, 0.0);
		local_nh.param("max_depth_range", max_depth_range, 0.0);
		local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
		local_nh.param("colormap", colormap, -1);

		//		if (stamped_filename) {
		//				std::size_t found = filename.find_last_of("/\\");
		//				std::string path = filename.substr(0, found + 1);
		//				std::string basename = filename.substr(found + 1);
		//				std::stringstream ss;
		//				ss << ros::Time::now().toNSec() << basename;
		//				filename = path + ss.str();
		//				ROS_INFO("Video recording to %s", filename.c_str());
		//		}

		if (codec.size() != 4) {
				ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
				exit(-1);
		}

		image_transport::ImageTransport it(nh);
		std::string topic = nh.resolveName("image");
		image_transport::Subscriber sub_image = it.subscribe(topic, 10, callback);

		ROS_INFO_STREAM("Waiting for topic " << topic << "...");
		// New Service
		while(first_frame == false && ros::ok()){
				ros::spinOnce();
		}

		ROS_INFO("Topic read! starting ...");
		ros::ServiceServer service = local_nh.advertiseService("video_controller",service_processor);

		ros::spin();
}
