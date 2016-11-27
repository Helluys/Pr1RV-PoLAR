#ifndef LIVEARTRACKER_H
#define LIVEARTRACKER_H

#include <QObject>

#include <PoLAR/Image.h>
#include <PoLAR/Object3D.h>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "LiveARViewer.h"

class LiveARTracker : public QObject
{
    Q_OBJECT

    public:
        LiveARTracker(LiveARViewer &viewer, int min_hessian = 800, float threshold = 0.1);
        LiveARTracker(LiveARViewer &viewer, const PoLAR::Image_uc &referenceImage, int min_hessian = 800, float threshold = 0.1);
        ~LiveARTracker();

        void setReferenceImage(const PoLAR::Image_uc &referenceImage);

    public slots:
        void newFrameReceived(unsigned char* data, int w, int h, int d);

    protected:
        cv::Mat computeHomography(cv::Mat &frame);

        cv::Mat polarToCvImage(const PoLAR::Image_uc &polarImage) const;
        PoLAR::Image_uc cvToPolarImage(const cv::Mat &cvImage) const;

        LiveARViewer &mViewer;
        cv::Mat mReferenceImage;
        cv::SurfFeatureDetector mDetector;
        std::vector<cv::KeyPoint> mKpReference;
        cv::SurfDescriptorExtractor mExtractor;
        cv::Mat mDescReference;
        cv::FlannBasedMatcher mMatcher;
        float mDistanceThreshold;
};

#endif // LIVEARTRACKER_H
