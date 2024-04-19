/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include <string>
#include <fstream>

// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
// necessary basic-type support
#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/utility.hpp"  // std::pair

#include "ad_common/config_utils/base/status.hpp"
#include "ad_log/ad_log.hpp"

namespace senseAD {
namespace common {
namespace utils {

template <class Property>
class CerealMethod {
 public:
    /**
     * @brief
     **/
    static confStatus_t LoadJSON(const std::string& file_path,
                                 Property* property) {
        if (nullptr == property) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader: null property";
        }
        std::ifstream fin(file_path);
        if (!fin.is_open()) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader failed to open json file: "
                << file_path;
        }
        try {
            cereal::JSONInputArchive json_archive(fin);
            json_archive(*property);
        } catch (const std::exception& ex) {
            AD_LWARN(CONFIGURATION_READER)
                << "Cereal-based reader failed to read " << file_path << ", "
                << ex.what();
            return CONF_INVALID_PARAM;
        }
        return CONF_SUCCESS;
    }
    static confStatus_t WriteJSON(const std::string& file_path,
                                  const Property& property) {
        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader failed to write json file: "
                << file_path;
        }
        try {
            cereal::JSONOutputArchive json_archive(fout);
            json_archive(property);
        } catch (const std::exception& ex) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader failed due to: " << ex.what();
            return CONF_INVALID_PARAM;
        }
        return CONF_SUCCESS;
    }
    static confStatus_t LoadXML(const std::string& file_path,
                                Property* property) {
        if (nullptr == property) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader: null property";
        }
        std::ifstream fin(file_path);
        if (!fin.is_open()) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader failed to load json file: "
                << file_path;
        }
        try {
            cereal::XMLInputArchive xml_archive(fin);
            xml_archive(*property);
        } catch (const std::exception& ex) {
            return CONF_INVALID_PARAM;
        }
        return CONF_SUCCESS;
    }
    static confStatus_t WriteXML(const std::string& file_path,
                                 const Property& property) {
        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            AD_LERROR(CONFIGURATION_READER)
                << "Cereal-based reader failed to write json file: "
                << file_path;
        }
        try {
            cereal::XMLOutputArchive xml_archive(fout);
            xml_archive(property);
        } catch (const std::exception& ex) {
            return CONF_INVALID_PARAM;
        }
        return CONF_SUCCESS;
    }
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
