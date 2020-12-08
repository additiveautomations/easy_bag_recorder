#include <easy_bag_recorder/easy_bag_recorder.h>

namespace easy_bag_recorder {

    void RecorderNode::start() {
        for (auto &topic : topics_) {
            ros::Subscriber subscriber =
                private_nh_.subscribe(topic, 10, &RecorderNode::recordCallback, this);
            subscribers_.push_back(subscriber);
        }
        ROS_INFO("Subscribed to %lu topics", subscribers_.size());
        bag_.open(rosbag_path_, rosbag::bagmode::Write);
    }

    std::string RecorderNode::getBagPath() { return rosbag_path_; }

    void RecorderNode::stop() {
        ROS_INFO("Stopping recording");
        for (auto &sub : subscribers_) {
            sub.shutdown();
        }

        private_nh_.shutdown();
        bag_.close();
    };

    void RecorderNode::recordCallback(const ros::MessageEvent<topic_tools::ShapeShifter const> &event) {
        ROS_INFO("Recording message");
        ros::M_string &header = event.getConnectionHeader();
        topic_tools::ShapeShifter::ConstPtr message = event.getMessage();
        bag_.write(header["topic"], ros::Time::now(), message);
    }

    void EasyBagRecorder::preemptGoal() {
        recorder_->stop();
        as_.setPreempted(result_, recorder_->getBagPath());
    }

    void EasyBagRecorder::executeGoal() {
        auto goal = as_.acceptNewGoal();
        // TODO should reject as opposed to accept and reject
        if (goal->topics.empty()) {
            ROS_ERROR("No topics provided aborting");
            as_.setAborted(result_, "No topics provided");
            return;
        }

        auto now = std::time(nullptr);
        auto bag_path = bag_output_dir_ + "/" + std::to_string(now) + ".bag";
        ROS_INFO("Recording to %s", bag_path.c_str());
        recorder_.reset(new RecorderNode(nh_, goal->topics, bag_path));

        std::string topic_list;
        for (auto topic : goal->topics) topic_list += topic + ", ";

        ROS_INFO("Recording topics: %s", topic_list.c_str());
        recorder_->start();
    }
}  // namespace easy_bag_recorder


int error(std::string message) {
    std::cerr << message << std::endl;
    return EXIT_FAILURE;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "easy_bag_recorder");
    std::string bag_output_dir;

    ros::param::get("~bag_output_dir", bag_output_dir);

    if (bag_output_dir.empty())
        return error("No bag_output_dir not set");

    if (!std::filesystem::exists(bag_output_dir))
        return error(bag_output_dir + " does not exist");

    if (!std::filesystem::is_directory(bag_output_dir))
        return error(bag_output_dir + " is not a directory");

    easy_bag_recorder::EasyBagRecorder server{bag_output_dir};

    ROS_INFO("Easy Bag Recorder started");
    ROS_INFO("Writing bags to %s", bag_output_dir.c_str());

    ros::spin();

    return 0;
}