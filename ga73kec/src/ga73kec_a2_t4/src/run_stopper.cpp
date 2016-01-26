#include	"stopper.h"
int	main(int	argc,	char	**argv)	{
        //	Initiate	new	ROS	node	named	"stopper"
        ros::init(argc,	argv,	"stopper");
        //	Create	new	stopper	object
        Stopper	stopper;
        ros::Rate	rate(100);
        //	Start	the	movement
        while (ros::ok()){
                stopper.Moving();
        }
};
