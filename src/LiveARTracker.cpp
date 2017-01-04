#include "LiveARTracker.h"

LiveARTracker::LiveARTracker(osg::ref_ptr<PoLAR::Object3D> transformedObject)
 : mThread()
{
    QObject::connect(&mThread, SIGNAL(finished()), this, SLOT(updateObject()));
    setTransformedObject(transformedObject);
}

LiveARTracker::LiveARTracker(osg::ref_ptr<PoLAR::Object3D> transformedObject, PoLAR::Image_uc &referenceImage)
 : mThread(referenceImage)
{
    QObject::connect(&mThread, SIGNAL(finished()), this, SLOT(updateObject()));
    setTransformedObject(transformedObject);
}

LiveARTracker::~LiveARTracker()
{
    mThread.quit();
    mThread.wait();
}

void LiveARTracker::setReferenceImage(PoLAR::Image_uc &referenceImage)
{
    mThread.setReferenceImage(referenceImage);
}

void LiveARTracker::newFrameReceived(unsigned char *data, int w, int h, int d)
{
    if(!mTransformedObject || mThread.isRunning())
        return;

    // Deep copy the image not to delete the camera data pool
    unsigned char *dataCopy = new unsigned char [w*h*d];
    for(int ii = 0; ii < w*h*d; ii++)
        dataCopy[ii] = data[ii];

    // Create OpenCV image
    cv::Mat frame(h, w, CV_8UC3);
    frame.data = dataCopy;
    cv::cvtColor(frame, frame, CV_BGR2RGB);

    // Start the pose computation thread
    mThread.setFrame(frame);
    mThread.start();
}

void LiveARTracker::setTransformedObject(osg::ref_ptr<PoLAR::Object3D> transformedObject)
{
    mTransformedObject = transformedObject;
}

void LiveARTracker::updateObject()
{
    const std::pair<cv::Mat, cv::Mat> &Rt = mThread.getPose();
    if(Rt.first.empty()) // reference was not found in frame
    {
        std::cout << "Empty pose..." << std::endl;
        mTransformedObject->setDisplayOff();
    }
    else
    {
        // Generate projection Matrix in osg format
        float angle = cv::norm(Rt.first);
        Rt.first /= angle;
        mTransformedObject->setTransformationMatrix(osg::Matrixd::identity());

        std::cout << "Got Pose : " << Rt.first << std::endl
                  << Rt.second << std::endl;
        // TODO : correctly compute the transform (conversion from OpenCV to OSG)

        mTransformedObject->rotate(angle, Rt.first.at<double>(0), Rt.first.at<double>(1), Rt.first.at<double>(2));
        //mTransformedObject->translate(Rt.second.at<double>(0), Rt.second.at<double>(1), Rt.second.at<double>(2));

        // Apply the pose to the object
        mTransformedObject->setDisplayOn();
    }
}

osg::Vec3d LiveARTracker::cvToOsgVec3d(const cv::Mat &mat) const
{
    osg::Vec3d vec;
    if(mat.rows == 3 && mat.cols == 1)
    {
        vec.x() = mat.at<double>(0);
        vec.y() = mat.at<double>(1);
        vec.z() = mat.at<double>(2);
    }
    return vec;
}

LiveARTracker::TrackingThread::TrackingThread(int min_hessian, float threshold)
 : mDetector(min_hessian), mDistanceThreshold(threshold)
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
    mPose = std::make_pair(cv::Mat::zeros(3, 1, CV_64FC1),  cv::Mat::zeros(3, 1, CV_64FC1));
}

LiveARTracker::TrackingThread::TrackingThread(PoLAR::Image_uc &referenceImage, int min_hessian, float threshold) : TrackingThread(min_hessian, threshold)
{
    setReferenceImage(referenceImage);
}

void LiveARTracker::TrackingThread::setReferenceImage(PoLAR::Image_uc &referenceImage)
{
    mImageMutex.lock();
        mReferenceImage = polarToCvImage(referenceImage);
        mDetector.detect(mReferenceImage, mKpReference);
        mExtractor.compute(mReferenceImage, mKpReference, mDescReference);
    mImageMutex.unlock();
}

void LiveARTracker::TrackingThread::setFrame(cv::Mat &frame)
{
    mFrameMutex.lock();
        delete[] mFrame.data;
        mFrame = frame;
    mFrameMutex.unlock();
}

void LiveARTracker::TrackingThread::run()
{
    using namespace cv;
    if(mReferenceImage.empty())
    {   // output empty values to notify failure
        mPose = std::make_pair(Mat(), Mat());
        return;
    }

    mFrameMutex.lock();
        // Detect keypoints
        std::vector<KeyPoint> kpFrame;
        mDetector.detect(mFrame, kpFrame);

        // Extract descriptors
        Mat descFrame;
        mExtractor.compute(mFrame, kpFrame, descFrame);
    mFrameMutex.unlock();

    // Match frame descriptors with reference descriptors
    std::vector<DMatch> matches, goodMatches;
    mImageMutex.lock(); // protect the reference descriptors
        mMatcher.match(mDescReference, descFrame, matches);

        // Look for best match distance
        double minDistance = 100;
        for(int i = 0; i < mDescReference.rows; i++)
            if(matches[i].distance < minDistance)
                minDistance = matches[i].distance;

    //std::cout << "Distance : "<< minDistance << " ";
    // Returning an empty Mat means the object was not found
    if(minDistance > mDistanceThreshold)
    {
        mPose = std::make_pair(Mat(), Mat());
        mImageMutex.unlock();
        return;
    }

    // Take only good matches (those with a distance < 3*minDistance)
    for(unsigned i = 0; i < matches.size(); i++)
        if(matches[i].distance < 3*minDistance)
            goodMatches.push_back(matches[i]);

    // Cannot determine homography without at least 4 points
    if(goodMatches.size() < 4)
    {
        mPose = std::make_pair(Mat(), Mat());
        mImageMutex.unlock();
        return;
    }

    // Generate matched point list from good matches
    vector<Point3f> ptsObject;
    vector<Point2f> ptsFrame;
    for(unsigned int i = 0; i < goodMatches.size(); i++)
    {
        ptsObject.push_back(Point3f(mKpReference[goodMatches[i].queryIdx].pt.x, mKpReference[goodMatches[i].queryIdx].pt.y, 0.f));
        ptsFrame.push_back(kpFrame[goodMatches[i].trainIdx].pt);
    }
    mImageMutex.unlock();

    // Vector of distortion coefficients
    Mat distCoeffs = Mat::zeros(4, 1, CV_64FC1);
    // Solve pose
    mPoseMutex.lock();
        solvePnPRansac(ptsObject, ptsFrame, mCameraIntrisics, distCoeffs, mPose.first, mPose.second, false, 500, 2.0, 0.95);
    mPoseMutex.unlock();
}

cv::Mat LiveARTracker::TrackingThread::polarToCvImage(PoLAR::Image_uc& polarImage) const
{
    cv::Mat cvImage(polarImage.t(), polarImage.s(), CV_8UC3, polarImage.getRowStepInBytes());
    cvImage.data = (uchar*)polarImage.data();
    cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
    return cvImage;
}
