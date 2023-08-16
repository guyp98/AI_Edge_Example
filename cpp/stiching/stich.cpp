/**
 * Copyright (c) 2020-2022 Hailo Technologies Ltd. All rights reserved.
 * Distributed under the MIT license (https://opensource.org/licenses/MIT)
 **/
/**
 * @file raw_async_streams_single_thread_example
 * This example demonstrates using low level async streams using single thread over c++.
 **/

#include "hailo/hailort.hpp"
#include "common.h"

#include <thread>
#include <iostream>
#include <chrono>
#include <mutex>
#include <future>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include <iostream>
using namespace std;
using namespace cv;
#include <queue>
#include <condition_variable>

#if defined(__unix__)
#include <sys/mman.h>
#endif

std::mutex mutx;
using namespace hailort;

using AlignedBuffer = std::shared_ptr<uint8_t>;

cv::Mat g_frame[10300];
uint32_t g_TotalFrame = 0xFFFFFFFF;
statistics stats;

std::chrono::time_point<std::chrono::system_clock> end_time;
std::chrono::duration<double> inference_time;
std::chrono::time_point<std::chrono::system_clock> start_time;

void print_inference_statistics(std::chrono::duration<double> inference_time,
                                std::string hef_file, double frame_count) { 
    std::cout << BOLDGREEN << "\n-I-----------------------------------------------" << std::endl;
    std::cout << "-I- " << hef_file.substr(0, hef_file.find(".")) << std::endl;
    std::cout << "-I-----------------------------------------------" << std::endl;
    std::cout << "\n-I-----------------------------------------------" << std::endl;
    std::cout << "-I- Inference c4i yolo model                       " << std::endl;
    std::cout << "-I-----------------------------------------------" << std::endl;
    std::cout << "-I- Average FPS:  " << frame_count / (inference_time.count()) << std::endl;
    std::cout << "-I- Total time:   " << inference_time.count() << " sec" << std::endl;
    std::cout << "-I- Latency:      " << 1.0 / (frame_count / (inference_time.count()) / 1000) << " ms" << std::endl;
    std::cout << "-I-----------------------------------------------" << std::endl;
}


void print_inference_statistics2(statistics* stats) {
    double start_time_secs = (double)stats->start_time.tv_sec + ((double)stats->start_time.tv_nsec / NSEC_IN_SEC);
    double end_time_secs = (double)stats->end_time.tv_sec + ((double)stats->end_time.tv_nsec / NSEC_IN_SEC);
    double infer_time_secs = end_time_secs - start_time_secs;
    //static float mbit_per_byte = 8.0f / 1024.0f / 1024.0f;
    //uint32_t send_frame_size = stats->output_vstream_infos[0].hw_frame_size;
    //uint32_t recv_frame_size = 0;

    printf(BOLDGREEN);
    printf("-I-----------------------------------------------\n");
    printf("-I- Total time:      %4.2lf sec\n", infer_time_secs);
    printf("-I- Average FPS:     %4.2lf\n", g_TotalFrame / infer_time_secs);
    //printf("-I- Send data rate:  %-4.2lf Mbit/s\n",
    //    (double)(stats->num_images) * send_frame_size * mbit_per_byte / infer_time_secs);
    //for (int i = 0; i < (int)stats->output_stream_cnt; i++) {
    //    recv_frame_size = stats->output_vstream_infos[i].hw_frame_size;
    //    printf("-I- Recv[%d] data rate: %-4.2lf Mbit/s\n", i,
    //        (double)(stats->num_images) * recv_frame_size * mbit_per_byte / infer_time_secs);
    //    printf("-I-----------------------------------------------\n");
    //}
    printf(RESET);
}

#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>

#include <stdio.h>
#include <iostream>
 
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
using namespace cv;
using namespace cv::xfeatures2d;
 
#include <iostream>
#include "opencv2/core.hpp"



#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
int main( int argc, char** argv )
{
    //CommandLineParser parser( argc, argv);
 
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
    std::vector<cv::KeyPoint> lastFramekeypoints1, lastFramekeypoints2;
    cv::Mat lastFrameDescriptors1, lastFrameDescriptors2;
    std::vector<cv::DMatch > matches;
    int movementDirection = 0;
    //std::string image_path1 = samples::findFile("Hill1.jpg");
    //std::string image_path2 = samples::findFile("Hill2.jpg");
    Mat image1= imread( "out1.png" );
    Mat image2= imread( "out2.png"  );

   
    imshow("first image",image2);
    imshow("second image",image1);
 
    (void) clock_gettime(CLOCK_MONOTONIC, &(stats.start_time));

    g_TotalFrame = 100;

    for(uint32_t t=0; t<g_TotalFrame; t++)
    {
   // finds keypoints and their disriptors
    detector->detectAndCompute(image1, noArray(), lastFramekeypoints1, lastFrameDescriptors1);
    detector->detectAndCompute(image2, noArray(), lastFramekeypoints2, lastFrameDescriptors2);

    // match the descriptor between two images
    matcher->match(lastFrameDescriptors1, lastFrameDescriptors2, matches);

    std::vector<cv::Point2d> good_point1, good_point2;
    good_point1.reserve(matches.size());
    good_point2.reserve(matches.size());

    //calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;
    for (const auto& m : matches)
    {
        double dist = m.distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    // filter out good points
    // distance which is less than or equals the min_dist*1.5
    // if this value is increased more keypoits are detected.
    for (const auto& m : matches)
    {
        if (m.distance <= 1.5 * min_dist)
        {
            // the matches variable holds the index values of x-y positions of the keypoints in both images.
            // queryIdx gives key points index which has a match and trainIdx gives its corrosponding matched key point position index 
            // these inde values then can be used to find the key points in the key points arrays.
            // e.g. array1[(1,20),(3,40)] array2[(5,40),(6,20)] 
            // m.queryIdx=0 m.trainIdx=1 => array1[0] array2[1] has a match. array2 co-ordinates are found at array2 given position.

            good_point1.push_back(lastFramekeypoints1.at(m.queryIdx).pt);
            good_point2.push_back(lastFramekeypoints2.at(m.trainIdx).pt);
        }
    }

    // crop rectangle constructor.
    cv::Rect croppImg1(0, 0, image1.cols, image1.rows);
    cv::Rect croppImg2(0, 0, image2.cols, image2.rows); 

    // find minimum horizontal value for image 1 to crop
    //e.g. img1 size = 200 first keypoint having match found at position 100 crop img1 to 0-100
    // crop image2 to from corresponding x value to the width. 
    //e.g. img2 width 200 point found at 50 crop image  50-200

    // movementDirection tells us are both the images aligned or not if not adjust the images accordingly.
    int imgWidth = image1.cols;
    for (int i = 0; i < good_point1.size(); ++i)
    {
        if (good_point1[i].x < imgWidth)
        {
            croppImg1.width = good_point1.at(i).x;
            croppImg2.x = good_point2[i].x;
            croppImg2.width = image2.cols - croppImg2.x;
            movementDirection = good_point1[i].y - good_point2[i].y;
            imgWidth = good_point1[i].x;
        }
    }
    image1 = image1(croppImg1);
    image2 = image2(croppImg2);
    }

    (void) clock_gettime(CLOCK_MONOTONIC, &(stats.end_time));

    print_inference_statistics2(&stats);

    int maxHeight = image1.rows > image2.rows ? image1.rows : image2.rows;
    int maxWidth = image1.cols + image2.cols;
    cv::Mat result=cv::Mat::zeros(cv::Size(maxWidth, maxHeight + abs(movementDirection)), CV_8UC3);
    if (movementDirection > 0)
    {
        cv::Mat half1(result, cv::Rect(0, 0, image1.cols, image1.rows));
        image1.copyTo(half1);
        cv::Mat half2(result, cv::Rect(image1.cols, abs(movementDirection),image2.cols, image2.rows));
        image2.copyTo(half2);
    }
    else
    {
        cv::Mat half1(result, cv::Rect(0, abs(movementDirection), image1.cols, image1.rows));
        image1.copyTo(half1);
        cv::Mat half2(result, cv::Rect(image1.cols,0 ,image2.cols, image2.rows));
        image2.copyTo(half2);
    }
    
    imshow("Stitched Image", result);
    
    int k = waitKey(0); // Wait for a keystroke in the window
    if (k == 's')
    {
        imwrite("StitchedImage.png", result);
    }
    return 0;
}

#endif