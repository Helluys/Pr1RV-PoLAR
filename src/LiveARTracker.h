#ifndef LIVEARTRACKER_H
#define LIVEARTRACKER_H

#include <QObject>

#include <PoLAR/Image.h>
#include <PoLAR/Object3D.h>

#include <QThread>
#include <QMutex>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

class LiveARTracker : public QObject
{
    Q_OBJECT

    public:
        LiveARTracker(osg::ref_ptr<PoLAR::Object3D> transformedObject);
        LiveARTracker(osg::ref_ptr<PoLAR::Object3D> transformedObject, PoLAR::Image_uc &referenceImage);
        ~LiveARTracker();

        void setReferenceImage(PoLAR::Image_uc &referenceImage);
        void setTransformedObject(osg::ref_ptr<PoLAR::Object3D> transformedObject);

    public slots:
        void newFrameReceived(unsigned char *data, int w, int h, int d);
        void updateObject();

    protected:
        osg::Vec3d cvToOsgVec3d(const cv::Mat &mat) const;

        class TrackingThread : public QThread
        {
            public:
                TrackingThread(int min_hessian = 1500, float threshold = 0.1);
                TrackingThread(PoLAR::Image_uc &referenceImage, int min_hessian = 1500, float threshold = 0.1);

                void run();
                void setReferenceImage(PoLAR::Image_uc &referenceImage);
                void setFrame(cv::Mat &frame);
                const std::pair<cv::Mat, cv::Mat>& getPose() const {return mPose;};

            private:
                cv::Mat polarToCvImage(PoLAR::Image_uc &polarImage) const;

                cv::Mat mReferenceImage, mFrame;
                cv::SurfFeatureDetector mDetector;
                std::vector<cv::KeyPoint> mKpReference;
                cv::SurfDescriptorExtractor mExtractor;
                cv::Mat mDescReference;
                cv::FlannBasedMatcher mMatcher;
                float mDistanceThreshold;

                std::vector<cv::Point2f> mReferencePoints;
                cv::Mat mCameraIntrisics;

                QMutex mPoseMutex, mImageMutex, mFrameMutex;
                std::pair<cv::Mat, cv::Mat> mPose;
        } mThread;

        osg::ref_ptr<PoLAR::Object3D> mTransformedObject;
};

#endif // LIVEARTRACKER_H
