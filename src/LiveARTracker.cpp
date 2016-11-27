#include "LiveARTracker.h"

LiveARTracker::LiveARTracker(LiveARViewer &viewer, int min_hessian, float threshold)
 : mViewer(viewer), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
}

LiveARTracker::LiveARTracker(LiveARViewer &viewer, PoLAR::Image_uc &referenceImage, int min_hessian, float threshold)
 : mViewer(viewer), mReferenceImage(polarToCvImage(referenceImage)), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
}

LiveARTracker::~LiveARTracker()
{
    //dtor
}

void LiveARTracker::setReferenceImage(PoLAR::Image_uc &referenceImage)
{
    mReferenceImage = polarToCvImage(referenceImage);
}

void LiveARTracker::newFrameReceived(unsigned char *data, int w, int h, int d)
{
    // Deep copy the image not to delete the camera data pool
    unsigned char *dataCopy = new unsigned char [w*h*d];
    for(int ii = 0; ii < w*h*d; ii++)
        dataCopy[ii] = data[ii];

    // Create OpenCV image
    cv::Mat frame(h, w, CV_8UC3);
    frame.data = dataCopy;
    cv::cvtColor(frame, frame, CV_BGR2RGB);

    // Look for homography
    cv::Mat H = computeHomography(frame);
    delete[] dataCopy;      // image not needed anymore
    if(H.empty())
        return;             // reference was not found in frame

    // Read projection Matrix in osg format
    osg::Matrixd M;
    M.set(  H.at<double>(0, 0), H.at<double>(0, 1), H.at<double>(0, 2), 0,
            H.at<double>(1, 0), H.at<double>(1, 1), H.at<double>(1, 2), 0,
            H.at<double>(2, 0), H.at<double>(2, 1), H.at<double>(2, 2), 0,
            0                  , 0                  , 0               , 1);

    // Apply the projection to the scene
    mViewer.setProjection(M);
}

cv::Mat LiveARTracker::computeHomography(cv::Mat &frame)
{
    using namespace cv;

    // Detect keypoints
    std::vector<KeyPoint> kpFrame;
    mDetector.detect(frame, kpFrame);

    // Extract descriptors
    Mat descFrame;
    mExtractor.compute(frame, kpFrame, descFrame);

    // Match frame descriptors with reference descriptors
    std::vector<DMatch> matches, goodMatches;
    mMatcher.match(mDescReference, descFrame, matches);

    // Look for best match distance
    double minDistance = 100;
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < minDistance)
            minDistance = matches[i].distance;
    std::cout << minDistance << std::endl;

    // Returning an empty Mat means the object was not found
    if(minDistance > mDistanceThreshold)
        return Mat();

    // Take only good matches (those with a distance < 3*minDistance)
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < 3*minDistance)
            goodMatches.push_back(matches[i]);

    // Generate matched point list from good matches
    vector<Point2f> ptsObject, ptsFrame;
    for(unsigned int i = 0; i < goodMatches.size(); i++)
    {
        ptsObject.push_back(mKpReference[goodMatches[i].queryIdx].pt);
        ptsFrame.push_back(kpFrame[goodMatches[i].trainIdx].pt);
    }

    // Look for a homography using these points
    return findHomography(ptsObject, ptsFrame, CV_RANSAC);
}

cv::Mat LiveARTracker::polarToCvImage(PoLAR::Image_uc& polarImage) const
{
    cv::Mat cvImage(polarImage.t(), polarImage.s(), CV_8UC3, polarImage.getRowStepInBytes());
    cvImage.data = (uchar*)polarImage.data();
    cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
    return cvImage;
}
