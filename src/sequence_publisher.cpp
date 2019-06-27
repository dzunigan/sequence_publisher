
#define PROGRAM_NAME \
    "sequence_publisher"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(string, timestamps, "", "Timestamps file")                                           \
    FLAG_CASE(uint64, o, 0, "Sequence offset")                                                     \
    FLAG_CASE(uint64, s, 0, "Sequence decimation")                                                 \
    FLAG_CASE(uint64, n, 0, "Maximum sequence length")                                             \
    FLAG_CASE(double, rate, 20.0, "Minimum frame rate (Hz)")

#define ARGS_CASES                                                                                 \
    ARG_CASE(images_directory)                                                                     \
    ARG_CASE(topic)

#include <signal.h>

#include <atomic>
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
    if (!FLAGS_timestamps.empty())
        RUNTIME_ASSERT(fs::is_regular_file(FLAGS_timestamps));
    RUNTIME_ASSERT(FLAGS_rate > 0.0);
}

inline void ValidateArgs() {
    RUNTIME_ASSERT(fs::is_directory(ARGS_images_directory));
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
    image_transport::Publisher pub = it.advertise(ARGS_topic, 30);

    std::cout << "Loading..." << '\r' << std::flush;

    // timestamps
    std::unordered_map<std::string, std::uint64_t> file2timestamp;
    if (!FLAGS_timestamps.empty())
        file2timestamp = timestamps::read(FLAGS_timestamps);

    // base path
    fs::path base_path(ARGS_images_directory);

    // sequence
    std::vector<std::string> paths;
    for (const fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(base_path), {})) {
        std::string filename = entry.path().filename().string();
        if (fs::is_regular_file(entry.status()))
            paths.push_back(filename);
    }
    std::sort(paths.begin(), paths.end(), sort_by_sid());

    paths = decimate_sequence(paths, FLAGS_o, FLAGS_s, FLAGS_n);

    size_t i = 0;
    ros::Rate loop_rate(FLAGS_rate);
    while (!exit_requested && nh.ok() && i < paths.size()) {
        std::cout << i+1 << " / " << paths.size() << '\r' << std::flush;
        const std::string& image_name = paths.at(i);
        ++i;

        // header
        std_msgs::Header header;
        if (!FLAGS_timestamps.empty()) {
            if (file2timestamp.find(image_name) == file2timestamp.end()) {
                std::cout << "[Warning] No timestamp associated with file: " << image_name << std::endl;
                continue;
            }
            header.stamp = ros::Time(static_cast<double>(file2timestamp.at(image_name)) * 1e-9);
        } else {
            header.stamp = ros::Time(sequence_id::get_sid(image_name));
        }

        cv::Mat image = cv::imread((base_path / image_name).string(), CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << std::endl;

    ros::shutdown();
    return 0;
}
