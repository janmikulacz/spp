/**
 * File:   find_file.h
 *
 * Date:   12.01.2023
 * Author: Jan Mikula
 * E-mail: jan.mikula@cvut.cz
 *
 */

#ifndef VISIS_DATA_FIND_FILE_H_
#define VISIS_DATA_FIND_FILE_H_

#include <string>
#include <optional>

#include <boost/filesystem.hpp>

namespace visis::data {

boost::filesystem::path FindFile(
    const std::string &full_path_or_name,
    const std::optional<std::string> &extension,
    const std::optional<std::string> &directory
);

}

#endif //VISIS_DATA_FIND_FILE_H_
