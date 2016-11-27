#include "LiveARTracker.h"

LiveARTracker::LiveARTracker(osg::ref_ptr<PoLAR::Object3D> trackedObject, int min_hessian, float threshold)
 : mTrackedObject(trackedObject), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mCameraIntrisics = cv::Mat::zeros(3, 3, CV_64FC1);
    double f = 55;                           // focal length in mm
    double sx = 22.3, sy = 14.9;             // sensor size
    double width = 1024, height = 768;        // image size
    mCameraIntrisics.at<double>(0, 0) = width*f/sx;     //      [ fx   0  cx ]
    mCameraIntrisics.at<double>(1, 1) = height*f/sy;    //      [  0  fy  cy ]
    mCameraIntrisics.at<double>(0, 2) = width/2;        //      [  0   0   1 ]
    mCameraIntrisics.at<double>(1, 2) = height/2;
    mCameraIntrisics.at<double>(2, 2) = 1;
}

LiveARTracker::LiveARTracker(osg::ref_ptr<PoLAR::Object3D> trackedObject, PoLAR::Image_uc &referenceImage, int min_hessian, float threshold)
 : mTrackedObject(trackedObject), mReferenceImage(polarToCvImage(referenceImage)), mDetector(min_hessian), mDistanceThreshold(threshold)
{
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
    mReferencePoints.push_back(cv::Point2f(0, 0));
    mReferencePoints.push_back(cv::Point2f(mReferenceImage.cols, 0));
    mReferencePoints.push_back(cv::Point2f(0, mReferenceImage.rows));
    mReferencePoints.push_back(cv::Point2f(mReferenceImage.cols, mReferenceImage.rows));
}

LiveARTracker::~LiveARTracker()
{
    //dtor
}

void LiveARTracker::setReferenceImage(PoLAR::Image_uc &referenceImage)
{
    mReferenceImage = polarToCvImage(referenceImage);
    mDetector.detect(mReferenceImage, mKpReference);
    mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
}

void LiveARTracker::newFrameReceived(unsigned char *data, int w, int h, int d)
{
    if(!mTrackedObject)
        return;

    // Deep copy the image not to delete the camera data pool
    unsigned char *dataCopy = new unsigned char [w*h*d];
    for(int ii = 0; ii < w*h*d; ii++)
        dataCopy[ii] = data[ii];

    // Create OpenCV image
    cv::Mat frame(h, w, CV_8UC3);
    frame.data = dataCopy;
    cv::cvtColor(frame, frame, CV_BGR2RGB);

    // Look for homography
    std::pair<cv::Mat, cv::Mat> Rt = computePose(frame);
    delete[] dataCopy;      // image not needed anymore
    if(Rt.first.empty())
    {
        mTrackedObject->setDisplayOff();
        return;             // reference was not found in frame
    }
    else
    {
        // Generate projection Matrix in osg format
        osg::Matrixd M = osg::Matrixd::identity();
        float angle = cv::norm(Rt.first);
        Rt.first /= angle;
        M.setRotate(osg::Quat(angle, cvToOsgVec3f(Rt.first)));
        M.translate(cvToOsgVec3f(Rt.second));

        // Apply the pose to the object
        mTrackedObject->setDisplayOn();
        mTrackedObject->setTransformationMatrix(M);
    }
}

std::pair<cv::Mat, cv::Mat> LiveARTracker::computePose(cv::Mat &frame)
{
    using namespace cv;

    // Detect keypoints
    std::vector<KeyPoint> kpFrame;
    mDetector.detect(frame, kpFrame);
    std::cout << "kpFrame : " << kpFrame.size() << '\t';

    // Extract descriptors
    Mat descFrame;
    mExtractor.compute(frame, kpFrame, descFrame);

    // Match frame descriptors with reference descriptors
    std::vector<DMatch> matches, goodMatches;
    mMatcher.match(mDescReference, descFrame, matches);
    std::cout << "matches : " << matches.size() << '\t';

    // Look for best match distance
    double minDistance = 100;
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < minDistance)
            minDistance = matches[i].distance;
    std::cout << "distance : " << minDistance << std::endl;

    // Returning an empty Mat means the object was not found
    if(minDistance > mDistanceThreshold)
        return std::make_pair(Mat(), Mat());

    // Take only good matches (those with a distance < 3*minDistance)
    for(int i = 0; i < mDescReference.rows; i++)
        if(matches[i].distance < 3*minDistance)
            goodMatches.push_back(matches[i]);

    // Cannot determine homography without at least 4 points
    if(goodMatches.size() < 4)
        return std::make_pair(Mat(), Mat());

    // Generate matched point list from good matches
    vector<Point3f> ptsObject;
    vector<Point2f> ptsFrame;
    for(unsigned int i = 0; i < goodMatches.size(); i++)
    {
        ptsObject.push_back(Point3f(mKpReference[goodMatches[i].queryIdx].pt.x, mKpReference[goodMatches[i].queryIdx].pt.y, 0.f));
        ptsFrame.push_back(kpFrame[goodMatches[i].trainIdx].pt);
    }

    // Vector of distortion coefficients
    Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);
    // Output Rotation - Translation vectors
    std::pair<Mat, Mat> Rt = std::make_pair(Mat::zeros(3, 1, CV_64FC1),  cv::Mat::zeros(3, 1, CV_64FC1));
    // Solve pose
    solvePnPRansac(ptsObject, ptsFrame, mCameraIntrisics, distCoeffs, Rt.first, Rt.second, false, 500, 2.0, 0.95);

    return Rt;
}

cv::Mat LiveARTracker::polarToCvImage(PoLAR::Image_uc& polarImage) const
{
    cv::Mat cvImage(polarImage.t(), polarImage.s(), CV_8UC3, polarImage.getRowStepInBytes());
    cvImage.data = (uchar*)polarImage.data();
    cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
    return cvImage;
}

osg::Vec3f LiveARTracker::cvToOsgVec3f(cv::Mat &mat) const
{
    osg::Vec3f vec;
    if(mat.rows == 3 && mat.cols == 1)
    {
        vec.x() = mat.at<float>(0);
        vec.y() = mat.at<float>(1);
        vec.z() = mat.at<float>(2);
    }
    return vec;
}

