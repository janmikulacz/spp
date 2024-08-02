/**
 * File:   find_file.cc
 *
 * Date:   12.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#include "visis/data/find_file.h"

#include <sstream>

#include <boost/filesystem.hpp>

#include "visis/log/log.h"

using namespace visis;
using namespace visis::data;

namespace fs = boost::filesystem;

boost::filesystem::path data::FindFile(
    const std::string &full_path_or_name,
    const std::optional<std::string> &extension,
    const std::optional<std::string> &directory
) {
    LOGF_INF("======== Finding file.");

    std::string name;
    std::string ext;
    std::string dir;
    if (directory) {
        name = full_path_or_name;
        ext = extension ? *extension : "";
        dir = *directory;
    } else {
        auto aux = fs::path(full_path_or_name);
        name = aux.replace_extension("").filename().string();
        ext = aux.extension().string();
        dir = aux.parent_path().string();
    }

    auto path_full_candidate = fs::path(dir + "/" + name + ext);

    if (fs::exists(path_full_candidate)) {
        LOGF_INF("Found file " << path_full_candidate << ".");
        LOGF_INF("==DONE== Finding file.");
        return path_full_candidate;
    }

    const auto path_name_ext = fs::path(name + ext);
    const auto path_dir = fs::path(dir);

    LOGF_INF("Searching for file " << path_name_ext << " in " << path_dir << ".");
    std::vector<fs::path> paths_full;
    std::for_each(fs::recursive_directory_iterator(path_dir), fs::recursive_directory_iterator{}, [&path_name_ext, &paths_full](const fs::directory_entry &e) {
        if (e.path().filename() == path_name_ext) {
            paths_full.push_back(e.path());
        }
    });

    if (paths_full.empty()) {
        LOGF_ERR("Could not find file " << path_name_ext << " in " << path_dir << "!");
        return fs::path{};
    }

    if (paths_full.size() > 1) {
        std::stringstream paths_full_str;
        for (int i = 0; i < paths_full.size(); ++i) {
            paths_full_str << (i == 0 ? "" : "\n") << paths_full[i];
        }
        LOGF_ERR("File " << path_name_ext << " is not unique in " << path_dir << "! Listing all found files:\n" << paths_full_str.str());
        return fs::path{};
    }

    LOGF_INF("Found unique file " << paths_full.front() << ".");
    LOGF_INF("==DONE== Finding file.");
    return paths_full.front();
}
