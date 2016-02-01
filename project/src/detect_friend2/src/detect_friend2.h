#include "opencv2/core/core.hpp"
using namespace cv;
class DetectFriend2
{
    public:

        static const double FRIEND_REF_DIST = 480 * 0.2 / 0.175;
        DetectFriend2();
        std::vector<Rect> clusters(Mat img);

    private:
        Mat image_star;
	Mat image_mushroom;
	Mat image_coin;
};
