/** This custom class keeps track of a target image in a video feed.
    Just initialize it with the target image, and each time you give a frame with computeHomography it returns the homography.
    If you just need the transformed coordinates of a set of points, use findInFrame.
    WARNING : it has not been checked whether the computed homography is likely to be correct or not. This class does not ensure the object
              is actually in the frame.
**/

#ifndef OPENCV_MATCHER_H
#define OPENCV_MATCHER_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class OpenCvMatcher
{
public:
    OpenCvMatcher(cv::Mat img, int min_hessian = 800, float threshold = 0.1);
    ~OpenCvMatcher();

    /// Save a set of points in the target image which you want to compute their transformed coordinates in each frame
    void setPoints(const std::vector<cv::Point2f>& points);
    /// The most important function !
    cv::Mat computeHomography(cv::Mat& frame);
    /** Transforms the points into transformed_points according to the homography found in frame
        @return Returns true if the object was found in the frame, false otherwise */
    bool findInFrame(const std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& transformed_points, cv::Mat& frame);
    /** Transforms the saved points into transformed_points according to the homography found in frame (see setPoints for how to save points)
        @return Returns true if the object was found in the frame, false otherwise */
    bool findInFrame(std::vector<cv::Point2f>& transformed_points, cv::Mat& frame);

private:
    cv::Mat target_image;
    cv::SurfFeatureDetector detector;
    std::vector<cv::KeyPoint> kp_target;
    cv::SurfDescriptorExtractor extractor;
    cv::Mat desc_target;
    cv::FlannBasedMatcher matcher;
    std::vector<cv::Point2f> saved_points;
    float distance_threshold;
};

#endif // OPENCV_MATCHER_H
