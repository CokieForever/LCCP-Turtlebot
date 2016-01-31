#include "opencv2/core/core.hpp"
using namespace cv;
class DetectFriend2
{
    public:

        
        DetectFriend2();
        std::vector<Rect> clusters(Mat img);

    private:
        Mat image_star;
	Mat image_mushroom;
	Mat image_coin;
};
