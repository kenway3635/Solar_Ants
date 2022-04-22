#include "willy_base.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "willy_base");
	willy_base willy;
    willy.run();

	return 0;
}
