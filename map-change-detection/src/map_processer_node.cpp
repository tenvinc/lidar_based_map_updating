#include "map_change_detection/map_processer.hpp"

MapProcesser::MapProcesser(ros::NodeHandle* nodehandle) :	nh_(*nodehandle) {
	std::string map_topic, map_processed_topic, cells_topic;

	nh_.getParam("/map_topic", map_topic);
	nh_.getParam("/processed_map_topic", map_processed_topic);
	nh_.getParam("/changed_cells_topic", cells_topic);

	sub = nh_.subscribe(cells_topic, 1, &MapProcesser::subscriberCallback, this);
	pub = nh_.advertise<nav_msgs::OccupancyGrid>(map_processed_topic, 5, true);
	get_curr_map_srv = nh_.advertiseService("get_curr_map", &MapProcesser::getCurrMapSrv, this);
	update_init_map_srv = nh_.advertiseService("update_initial_map", &MapProcesser::updateInitMapSrv, this);

	while(ros::ok()){
		boost::shared_ptr<nav_msgs::OccupancyGrid const> map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(4.0));
		if(map){
			initial_map = *map;
			online_map = initial_map;
			break;
		} else {
			ROS_INFO("Waiting for map...");
		}
	}
}

void MapProcesser::subscriberCallback(const map_change_detection::ChangedCells& msg) {
	online_map = initial_map;

	for(auto idx : msg.toOcc)
		online_map.data.at(idx) = 100;

	for(auto idx : msg.toFree)
		online_map.data.at(idx) = 0;

	for(auto idx : msg.toFree) {
		bool below = idx >= online_map.info.width;
		bool above = idx < (online_map.data.size() - online_map.info.width);
		int remainder = idx % online_map.info.width;
		bool right = remainder != online_map.info.width - 1;
		bool left = remainder != 0;
		if(below && above && right && left) {
			auto tmp_idx = idx - online_map.info.width; // below
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx++;	// below right
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx -= 2;	// below left
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx += online_map.info.width; // left
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx += 2; // right
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx += online_map.info.width; // above right
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx--;	// above
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
			tmp_idx--;	// above left
			if(online_map.data.at(tmp_idx) == -1)
				online_map.data.at(tmp_idx) = 100;
		} else {
			int64_t	tmp_idx;
			if(below) {
				tmp_idx = idx - online_map.info.width;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(above) {
				tmp_idx = idx + online_map.info.width;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(right) {
				tmp_idx = idx + 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(left) {
				tmp_idx = idx - 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(above && right) {
				tmp_idx = idx + online_map.info.width + 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(above && left) {
				tmp_idx = idx + online_map.info.width - 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(below && right) {
				tmp_idx = idx - online_map.info.width + 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
			if(below && left) {
				tmp_idx = idx - online_map.info.width - 1;
				if(online_map.data.at(tmp_idx) == -1)
					online_map.data.at(tmp_idx) = 100;
			}
		}
	}

	online_map.header.stamp = ros::Time::now();

	pub.publish(online_map);
}

bool MapProcesser::getCurrMapSrv(map_change_detection::GetCurrMap::Request &req, map_change_detection::GetCurrMap::Response &response) {
	response.online_map = online_map;
	return true;
}

bool MapProcesser::updateInitMapSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &response) {
	ROS_INFO("MapProcesser::Updating initial map");
	initial_map = online_map;
	return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "updated_map_processer_node");
	ros::NodeHandle nh("~");

	MapProcesser	node(&nh);

	ros::spin();

	return 0;
}