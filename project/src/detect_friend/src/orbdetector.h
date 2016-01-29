#include "opencv2/core/core.hpp"

class ORBDetector
{
    public:
        struct ORBMatchResult
        {
            cv::Point2f corners[4];
            double score;
            cv::Mat colorDiffImg;
        };
        
        ORBDetector();
        ORBMatchResult match(const cv::Mat& img, const cv::Mat& templ) const;
        cv::Mat drawResult(const cv::Mat& img, const ORBMatchResult& result) const;
        
    private:
        static cv::Mat binarizeImageKMeans(const cv::Mat& input);
        static cv::Mat removeIsolatedPixels(const cv::Mat& binInput, int nbMinNeighbours=1);
};