#ifndef LIVEARTRACKER_H
#define LIVEARTRACKER_H

#include <QObject>

#include <PoLAR/Image.h>
#include <PoLAR/Object3D.h>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class LiveARTracker : public QObject
{
    Q_OBJECT

    public:
        LiveARTracker(osg::ref_ptr<PoLAR::Object3D> trackedObject, int min_hessian = 1500, float threshold = 0.1);
        LiveARTracker(osg::ref_ptr<PoLAR::Object3D> trackedObject, PoLAR::Image_uc &referenceImage, int min_hessian = 1500, float threshold = 0.1);
        ~LiveARTracker();

        void setReferenceImage(PoLAR::Image_uc &referenceImage);
        void setTrackedObject(osg::ref_ptr<PoLAR::Object3D> trackedObject) {mTrackedObject = trackedObject;}

    public slots:
        void newFrameReceived(unsigned char *data, int w, int h, int d);

    protected:
        std::pair<cv::Mat, cv::Mat> computePose(cv::Mat &frame);

        cv::Mat polarToCvImage(PoLAR::Image_uc &polarImage) const;
        osg::Vec3f cvToOsgVec3f(cv::Mat &mat) const;

        osg::ref_ptr<PoLAR::Object3D> mTrackedObject;
        cv::Mat mReferenceImage;
        cv::SurfFeatureDetector mDetector;
        std::vector<cv::KeyPoint> mKpReference;
        cv::SurfDescriptorExtractor mExtractor;
        cv::Mat mDescReference;
        cv::FlannBasedMatcher mMatcher;
        float mDistanceThreshold;

        std::vector<cv::Point2f> mReferencePoints;
        cv::Mat mCameraIntrisics;
};

#endif // LIVEARTRACKER_H
