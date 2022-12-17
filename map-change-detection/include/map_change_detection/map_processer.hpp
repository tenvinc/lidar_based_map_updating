#include "grid.hpp"

#include <map_change_detection/ChangedCells.h>
#include <std_srvs/Empty.h>
#include <map_change_detection/GetCurrMap.h>

class MapProcesser
{
	public:
		MapProcesser(ros::NodeHandle* nodehandle);

	private:

		ros::NodeHandle nh_;
		ros::Subscriber	sub;
		ros::Publisher	pub;
		ros::ServiceServer get_curr_map_srv, update_init_map_srv;

		nav_msgs::OccupancyGrid initial_map;
		nav_msgs::OccupancyGrid online_map;   // current map that is most updated

		void subscriberCallback(const map_change_detection::ChangedCells& msg);

		bool getCurrMapSrv(map_change_detection::GetCurrMap::Request &req, map_change_detection::GetCurrMap::Response &response);

		bool updateInitMapSrv(std_srvs::Empty::Request &req, std_srvs::EmptyResponse &response);

};