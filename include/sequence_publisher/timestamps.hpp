
#ifndef TIMESTAMPS_HPP_
#define TIMESTAMPS_HPP_

// STL
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <istream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace timestamps {

std::unordered_map<std::string, std::uint64_t> read(const std::string &path, char delim = ',') {
    std::unordered_map<std::string, std::uint64_t> file2timestamp;

    std::string line;
    std::ifstream input(path);
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        if (line.front() == '#') continue;

        std::uint64_t timestamp;
        std::string value, file;
        std::stringstream line_stream(line);
        if (std::getline(line_stream, value, delim) && std::getline(line_stream, file, delim)) {
            timestamp = static_cast<std::uint64_t>(std::stoll(value));
            file2timestamp[file] = timestamp;
        }
    }

    return file2timestamp;
}

} // namespace timestamps

#endif // TIMESTAMPS_HPP_
