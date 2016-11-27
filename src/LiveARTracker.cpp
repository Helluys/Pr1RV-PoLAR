#include "LiveARTracker.h"

LiveARTracker::LiveARTracker(LiveARViewer &viewer, int min_hessian, float threshold)
 : mViewer(viewer), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
}

LiveARTracker::LiveARTracker(LiveARViewer &viewer, const PoLAR::Image_uc &referenceImage, int min_hessian, float threshold)
 : mViewer(viewer), mReferenceImage(polarToCvImage(referenceImage)), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
}

LiveARTracker::~LiveARTracker()
{
    //dtor
}

void LiveARTracker::setReferenceImage(const PoLAR::Image_uc& referenceImage)
{
    mReferenceImage = polarToCvImage(referenceImage);
}

void LiveARTracker::newFrameReceived(unsigned char* data, int w, int h, int d)
{
    cv::Mat frame(h, w, CV_8UC1, data);
    cv::Mat H = computeHomography(frame);
    cv::Mat iH = H.inv();

    osg::Matrixd M;
    M.set(  iH.at<double>(0, 0), iH.at<double>(0, 1), iH.at<double>(0, 2), 0,
            iH.at<double>(1, 0), iH.at<double>(1, 1), iH.at<double>(1, 2), 0,
            iH.at<double>(2, 0), iH.at<double>(2, 1), iH.at<double>(2, 2), 0,
            0                  , 0                  , 0                  , 1);
    mViewer.setProjection(M);
}

cv::Mat LiveARTracker::computeHomography(cv::Mat &frame)
{
    using namespace cv;
    std::vector<KeyPoint> kpFrame;
    mDetector.detect(frame, kpFrame);

    Mat descFrame;
    mExtractor.compute(frame, kpFrame, descFrame);

    std::vector<DMatch> matches, goodMatches;
    mMatcher.match(mDescReference, descFrame, matches);

    double minDistance = 100;
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < minDistance) minDistance = matches[i].distance;

    if(minDistance > mDistanceThreshold)
        return Mat(); // returning an empty Mat means the object was not found

    // Take only good matches (those with a distance < 3*minDistance)
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < 3*minDistance) goodMatches.push_back(matches[i]);

    vector<Point2f> ptsObject, ptsFrame;
    for(unsigned int i = 0; i < goodMatches.size(); i++)
    {
        ptsObject.push_back(mKpReference[goodMatches[i].queryIdx].pt);
        ptsFrame.push_back(kpFrame[goodMatches[i].trainIdx].pt);
    }

    return findHomography(ptsObject, ptsFrame, CV_RANSAC);
}

cv::Mat LiveARTracker::polarToCvImage(const PoLAR::Image_uc& polarImage) const
{
    unsigned size = polarImage.getHeight() * polarImage.getWidth();
    unsigned char *data = new unsigned char [size];
    for(unsigned ii = 0; ii < size; ii++)
        data[ii] = polarImage.data()[ii];

    return cv::Mat(polarImage.getHeight(), polarImage.getWidth(), CV_8UC1, data);
}

PoLAR::Image_uc LiveARTracker::cvToPolarImage(const cv::Mat& cvImage) const
{
    unsigned size = cvImage.total();
    unsigned char *data = new unsigned char [size];
    for(unsigned ii = 0; ii < size; ii++)
        data[ii] = cvImage.data[ii];

    return PoLAR::Image_uc(cvImage.data, cvImage.rows, cvImage.cols, 1);
}
