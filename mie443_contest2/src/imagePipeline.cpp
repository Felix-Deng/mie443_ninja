#include <imagePipeline.h>
//added library and values
#include <iostream>
#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

const char* keys =
       "{ help h |                  | Print help message. }"
       "{ input1 | box.png          | Path to input image 1. }"
       "{ input2 | box_in_scene.png | Path to input image 2. }";
//added library and values

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
//#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
#define IMAGE_TOPIC "camera/rgb/image_raw"
// rostopic hz camera/rgb/image_raw



ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
   image_transport::ImageTransport it(n);
   sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
   isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
   try {
       if(isValid) {
           img.release();
       }
       img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
       isValid = true;
   } catch (cv_bridge::Exception& e) {
       std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                 << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
       isValid = false;
   }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
   int template_id = -1;
   if(!isValid) {
       std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    
   } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
       std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
       std::cout << "img.empty():" << img.empty() << std::endl;
       std::cout << "img.rows:" << img.rows << std::endl;
       std::cout << "img.cols:" << img.cols << std::endl;
   }
     else 
   {
       /***YOUR CODE HERE***/
       // Use: boxes.templates
       std::vector<int> Matches{0, 0, 0}; //initialize array to store good matches for three templates
       Mat img_scene = img; //this is the scanned image, call this once only
       for(int count = 0; count<5; count++)
       {
            for(int j = 0; j<3; j++)
            {
                Mat img_object = boxes.templates[j]; //reference image, from templates

                //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
                int minHessian = 400;
                Ptr<SURF> detector = SURF::create( minHessian );
                std::vector<KeyPoint> keypoints_object, keypoints_scene;
                Mat descriptors_object, descriptors_scene;
                detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
                detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
                //-- Step 2: Matching descriptor vectors with a FLANN based matcher
                // Since SURF is a floating-point descriptor NORM_L2 is used
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
                std::vector< std::vector<DMatch> > knn_matches;
                matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
                //-- Filter matches using the Lowe's ratio test
                const float ratio_thresh = 0.75f;
                std::vector<DMatch> good_matches;
                for (size_t i = 0; i < knn_matches.size(); i++)
                {
                    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                    {
                        good_matches.push_back(knn_matches[i][0]);
                    }
                }
                
                //end of copy paste
                Matches[j] = Matches[j] + good_matches.size(); //store good matches number into array
                //std::cout << "Template ID:" << j+1 << std::endl;
                //std::cout << "Good matches:" << Matches[j] << std::endl;
            }
       }

       if(Matches[0]<300 && Matches[1]<300 && Matches[2]<300) {
        template_id = 3;
       }
       else {
       std::vector<int>::iterator max_num;
       max_num = std::max_element(Matches.begin(), Matches.end());
       template_id = std::distance(Matches.begin(), max_num);
       cv::imshow("view", img);
       std::cout << "Template 1:" << Matches[0] << std::endl;
       std::cout << "Template 2:" << Matches[1] << std::endl;
       std::cout << "Template 3:" << Matches[2] << std::endl;
       cv::waitKey(10);
       }
   }
   return (template_id+1); //0->invalid, 1->template1, 2->template2, 3->template3, 4->blank page
}