
#define PROGRAM_NAME \
    "stereo_publisher"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(string, left_timestamps, "", "Left images timestamps file")                          \
    FLAG_CASE(string, right_timestamps, "", "Right images timestamps file")                        \
    FLAG_CASE(uint64, o, 0, "Sequence offset")                                                     \
    FLAG_CASE(uint64, s, 0, "Sequence decimation")                                                 \
    FLAG_CASE(uint64, n, 0, "Maximum sequence length")                                             \
    FLAG_CASE(double, rate, 20.0, "Minimum frame rate (Hz)")

#define ARGS_CASES                                                                                 \
    ARG_CASE(left_images_directory)                                                                \
    ARG_CASE(right_images_directory)                                                               \
    ARG_CASE(left_topic)                                                                                \
    ARG_CASE(right_topic)

#include <signal.h>

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <Eigen/Core>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sequence_publisher/args.hpp>
#include <sequence_publisher/sequence.hpp>
#include <sequence_publisher/timestamps.hpp>

#include <sequence_publisher/util/macros.h>
#include <sequence_publisher/util/version.h>

namespace fs = boost::filesystem;

std::atomic<bool> exit_requested;

void signal_handler(int sig) {
  exit_requested = true;
}

inline void ValidateFlags() {
    if (!FLAGS_left_timestamps.empty())
        RUNTIME_ASSERT(fs::is_regular_file(FLAGS_left_timestamps));
    if (!FLAGS_right_timestamps.empty())
        RUNTIME_ASSERT(fs::is_regular_file(FLAGS_right_timestamps));
    RUNTIME_ASSERT(FLAGS_rate > 0.0);
}

inline void ValidateArgs() {
    RUNTIME_ASSERT(fs::is_directory(ARGS_left_images_directory));
    RUNTIME_ASSERT(fs::is_directory(ARGS_right_images_directory));
}

template<typename Container>
Container decimate_sequence(const Container& full_sequence, std::size_t offset = 0, std::size_t step = 0, std::size_t nmax = 0) {

    if (nmax == 0)
        nmax = std::numeric_limits<std::size_t>::max();

    if (offset >= full_sequence.size())
        return Container();

    std::size_t n = std::min(nmax, static_cast<std::size_t>(std::ceil(static_cast<double>(full_sequence.size() - offset) / static_cast<double>(step + 1))));

    Container decimated_sequence(n);

    std::size_t i = offset;
    for (std::size_t ctr = 0; ctr < n; ++ctr, i += (step + 1))
        decimated_sequence.at(ctr) = full_sequence.at(i);

    return decimated_sequence;
}

int main(int argc, char* argv[]) {

    // Print build info
    std::cout << PROGRAM_NAME << " (" << GetBuildInfo() << ")" << std::endl;
    std::cout << std::endl;

    // Handle help flag
    if (args::HelpRequired(argc, argv)) {
        args::ShowHelp();
        return 0;
    }

    // Parse input flags
    args::ParseCommandLineNonHelpFlags(&argc, &argv, true);

    // Check number of args
    if (argc-1 != args::NumArgs()) {
        args::ShowHelp();
        return -1;
    }

    // Parse input args
    args::ParseCommandLineArgs(argc, argv);

    // Validate input arguments
    ValidateFlags();
    ValidateArgs();

    ros::init(argc, argv, PROGRAM_NAME, ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    exit_requested = false;
    signal(SIGINT, signal_handler);

    // topic
    image_transport::ImageTransport it(nh);
    image_transport::Publisher left_pub = it.advertise(ARGS_left_topic, 30);
    image_transport::Publisher right_pub = it.advertise(ARGS_right_topic, 30);

    std::cout << "Loading..." << '\r' << std::flush;

    // timestamps
    std::unordered_map<std::string, std::uint64_t> left_file2timestamp;
    if (!FLAGS_left_timestamps.empty())
        left_file2timestamp = timestamps::read(FLAGS_left_timestamps);

    std::unordered_map<std::string, std::uint64_t> right_file2timestamp;
    if (!FLAGS_right_timestamps.empty())
        right_file2timestamp = timestamps::read(FLAGS_right_timestamps);

    // base path
    fs::path left_base_path(ARGS_left_images_directory);
    fs::path right_base_path(ARGS_right_images_directory);

    // sequence
    std::vector<std::string> left_paths;
    for (const fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(left_base_path), {})) {
        std::string filename = entry.path().filename().string();
        if (fs::is_regular_file(entry.status()))
            left_paths.push_back(filename);
    }
    std::sort(left_paths.begin(), left_paths.end(), sort_by_sid());

    std::vector<std::string> right_paths;
    for (const fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(right_base_path), {})) {
        std::string filename = entry.path().filename().string();
        if (fs::is_regular_file(entry.status()))
            right_paths.push_back(filename);
    }
    std::sort(right_paths.begin(), right_paths.end(), sort_by_sid());

    if (left_paths.size() != right_paths.size())
        std::cout << "[Warning] Number of left and right images doesn't match!" << std::endl;

    left_paths = decimate_sequence(left_paths, FLAGS_o, FLAGS_s, FLAGS_n);
    right_paths = decimate_sequence(right_paths, FLAGS_o, FLAGS_s, FLAGS_n);

    size_t i = 0;
    const size_t N = std::min(left_paths.size(), right_paths.size());
    ros::Rate loop_rate(FLAGS_rate);
    while (!exit_requested && nh.ok() && i < N) {
        std::cout << i+1 << " / " << N << '\r' << std::flush;
        const std::string& left_image_name = left_paths.at(i);
        const std::string& right_image_name = right_paths.at(i);
        ++i;

        // left image
        std_msgs::Header left_header;
        if (!FLAGS_left_timestamps.empty()) {
            if (left_file2timestamp.find(left_image_name) == left_file2timestamp.end()) {
                std::cout << "[Warning] No timestamp associated with file (left): " << left_image_name << std::endl;
                continue;
            }
            left_header.stamp = ros::Time(static_cast<double>(left_file2timestamp.at(left_image_name)) * 1e-9);
        } else {
            left_header.stamp = ros::Time(sequence_id::get_sid(left_image_name));
        }

        cv::Mat left_image = cv::imread((left_base_path / left_image_name).string(), CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(left_header, "bgr8", left_image).toImageMsg();

        // right
        std_msgs::Header right_header;
        if (!FLAGS_right_timestamps.empty()) {
            if (right_file2timestamp.find(right_image_name) == right_file2timestamp.end()) {
                std::cout << "[Warning] No timestamp associated with file (right): " << right_image_name << std::endl;
                continue;
            }
            right_header.stamp = ros::Time(static_cast<double>(right_file2timestamp.at(right_image_name)) * 1e-9);
        } else {
            right_header.stamp = ros::Time(sequence_id::get_sid(right_image_name));
        }

        cv::Mat right_image = cv::imread((right_base_path / right_image_name).string(), CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(right_header, "bgr8", right_image).toImageMsg();

        // publish
        left_pub.publish(left_msg);
        right_pub.publish(right_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << std::endl;

    ros::shutdown();
    return 0;
}
