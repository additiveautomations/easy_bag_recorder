#pragma once

#include <filesystem>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <actionlib/server/simple_action_server.h>
#include <topic_tools/shape_shifter.h>
#include <easy_bag_recorder/RecordAction.h>

namespace easy_bag_recorder {

    class RecorderNode {
    private:
        ros::NodeHandle &private_nh_;
        std::vector<std::string> topics_;
        std::string rosbag_path_;
        std::vector<ros::Subscriber> subscribers_;
        rosbag::Bag bag_;

    public:
        RecorderNode(ros::NodeHandle &parent, std::vector<std::string> topics, std::string path)
                : private_nh_(parent), topics_(std::move(topics)), rosbag_path_(std::move(path)) {};

        void start();
        void stop();
        std::string getBagPath();
        void recordCallback(const ros::MessageEvent<topic_tools::ShapeShifter const> &event);
    };

    class EasyBagRecorder {
    private:
        ros::NodeHandle nh_;
        std::shared_ptr<RecorderNode> recorder_;
        actionlib::SimpleActionServer<easy_bag_recorder::RecordAction> as_;
        std::string bag_output_dir_;
        std::string topics_;
        easy_bag_recorder::RecordResult result_;

    public:
        EasyBagRecorder(std::string bag_dir)
                : as_(nh_, "record", false), bag_output_dir_(std::move(bag_dir)) {
            as_.registerGoalCallback(std::bind(&EasyBagRecorder::executeGoal, this)); // NOLINT
            as_.registerPreemptCallback(std::bind(&EasyBagRecorder::preemptGoal, this)); // NOLINT
            as_.start();
        }

        void executeGoal();
        void preemptGoal();
    };
}