#include "Walker.h"

int	main(int	argc,	char	**argv)	{
        //	Initiate	new	ROS	node	named	"Walker"
        ros::init(argc,	argv,	"Walker");
        //	Create	new	stopper	object
        Walker	walker;
        ros::Rate	rate(100);
        //	Start	the	movement
        while (ros::ok()){
                walker.Moving();
        }
};
