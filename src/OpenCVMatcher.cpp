#include "OpenCVMatcher.h"
#include <iostream>

using namespace cv;

OpenCvMatcher::OpenCvMatcher(Mat img, int min_hessian, float threshold) : target_image(img), detector(min_hessian), distance_threshold(threshold)
{
    detector.detect(target_image, kp_target);
    extractor.compute(target_image, kp_target, desc_target);
}

OpenCvMatcher::~OpenCvMatcher()
{
    // dtor
}

cv::Mat OpenCvMatcher::computeHomography(cv::Mat& frame)
{
    std::vector<KeyPoint> kp_frame;
    detector.detect(frame, kp_frame);

    Mat desc_frame;
    extractor.compute(frame, kp_frame, desc_frame);

    std::vector<DMatch> matches, good_matches;
    matcher.match(desc_target, desc_frame, matches);

    double min_dist = 100;
    for(int i = 0; i < desc_target.rows; i++)
        if(matches[i].distance < min_dist) min_dist = matches[i].distance;

    if(min_dist > distance_threshold)
        return Mat(); // returning an empty Mat means the object was not found

    // Take only good matches (those with a distance < 3*min_dist)
    for(int i = 0; i < desc_target.rows; i++)
        if(matches[i].distance < 3*min_dist) good_matches.push_back(matches[i]);

    vector<Point2f> pts_object, pts_frame;
    for(unsigned int i = 0; i < good_matches.size(); i++)
    {
        pts_object.push_back(kp_target[good_matches[i].queryIdx].pt);
        pts_frame.push_back(kp_frame[good_matches[i].trainIdx].pt);
    }

    return findHomography(pts_object, pts_frame, CV_RANSAC);
}

bool OpenCvMatcher::findInFrame(const std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& transformed_points, cv::Mat& frame)
{
    cv::Mat H = computeHomography(frame);
    if(H.empty())
        return false; // object was not found

    perspectiveTransform(points, transformed_points, H);
    return true;
}

bool OpenCvMatcher::findInFrame(std::vector<cv::Point2f>& transformed_points, cv::Mat& frame)
{
    cv::Mat H = computeHomography(frame);
    if(H.empty())
        return false; // object was not found

    perspectiveTransform(saved_points, transformed_points, H);
    return true;
}

void OpenCvMatcher::setPoints(const std::vector<cv::Point2f>& points)
{
    saved_points = points;
}
