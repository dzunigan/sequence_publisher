#ifndef SEQUENCE_HPP
#define SEQUENCE_HPP

// STL
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

// Boost
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

template <typename T>
T str2num ( const std::string &str )
{
     std::stringstream ss(str);
     T result;
     return ss >> result ? result : 0;
}

struct sequence_id
{
    bool operator ()(const std::string &a,const std::string &b)
    {
        return get_sid(a) < get_sid(b);
    }

    static bool match_regex(const std::string &name) {
        std::string rev_name(name);
        std::reverse(rev_name.begin(), rev_name.end());

        return std::regex_match(rev_name, rev_sid_regex);
    }

    static double get_sid(const std::string &name) {
        std::string rev_name(name);
        std::reverse(rev_name.begin(), rev_name.end());

        std::smatch matches;
        if (!std::regex_match(rev_name, matches, rev_sid_regex))
            throw std::runtime_error("Filename doesn't match sid regex!");
        if (matches.size() != 4)
            throw std::runtime_error("Unexpected number of sub-matches");

        std::string id = matches[2].str();
        std::reverse(id.begin(), id.end());

        return str2num<double>(id);
    }

    static std::regex rev_sid_regex;
};

using sort_by_sid = sequence_id;
std::regex sort_by_sid::rev_sid_regex{"^([[:alpha:]]{3})\\.([[:digit:]]*\\.?[[:digit:]]+)(.*$)", std::regex::extended};

std::vector<std::string> getSequence(const std::string &base_path, std::size_t offset = 0, std::size_t nmax = 0, std::size_t step = 0) {

    namespace fs = boost::filesystem;

    std::vector<std::string> full_paths;

    if (nmax == 0)
        nmax = std::numeric_limits<std::size_t>::max();

    fs::path base_dir(base_path);
    if (!fs::is_directory(base_dir)) return full_paths; // throw std::domain_error("[getSequence] Invalid path");

    std::vector<std::string> filenames;
    for (const fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(base_dir), {})) {
        std::string filename = entry.path().filename().string();
        if (fs::is_regular_file(entry.status()) && sequence_id::match_regex(filename))
            filenames.push_back(filename);
    }

    if (offset >= filenames.size()) return full_paths;
    std::sort(filenames.begin(), filenames.end(), sort_by_sid());

    std::size_t n = std::min(nmax, static_cast<std::size_t>(std::ceil(static_cast<double>(filenames.size() - offset) / static_cast<double>(step + 1))));

    full_paths.reserve(n);
    std::size_t i = offset;
    for (std::size_t ctr = 0; ctr < n; ++ctr, i += (step + 1))
        full_paths.push_back((base_dir / filenames.at(i)).string());

    return full_paths;
}

#endif // SEQUENCE_HPP
