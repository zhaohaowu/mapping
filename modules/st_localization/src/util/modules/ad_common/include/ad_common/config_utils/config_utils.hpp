/*
 * Copyright (C) 2017-2018 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */

#pragma once

#include <string>
#include "ad_common/config_utils/base/configuration_reader.hpp"

namespace senseAD {
namespace common {
namespace utils {

/*
 * @brief template method class to read/write property from/into files
 */
class ConfigurationReader {
 public:
    /*
     * @brief Read configuration from .json file
     * @param file_path [in] the path to the configuration file
     * @param property [out] property
     * @return error code
     */
    template <class Property, template <class> class Method = CerealMethod>
    static confStatus_t LoadJSON(const std::string& file_path,
                                 Property* property) {
        return Method<Property>::LoadJSON(file_path, property);
    }
    /*
     * @brief Write configuration into .json file
     * @param file_path [in] the path to the configuration file
     * @param property [in] property
     * @return error code
     */
    template <class Property, template <class> class Method = CerealMethod>
    static confStatus_t WriteJSON(const std::string& file_path,
                                  const Property& property) {
        return Method<Property>::WriteJSON(file_path, property);
    }
    /*
     * @brief Read configuration from .xml file
     * @param file_path [in] the path to the configuration file
     * @param property [out] property
     * @return error code
     */
    template <class Property, template <class> class Method = CerealMethod>
    static confStatus_t LoadXML(const std::string& file_path,
                                Property* property) {
        return Method<Property>::LoadXML(file_path, property);
    }
    /*
     * @brief Write configuration from .xml file
     * @param file_path [in] the path to the configuration file
     * @param property [in] property
     * @return error code
     */
    template <class Property, template <class> class Method = CerealMethod>
    static confStatus_t WriteXML(const std::string& file_path,
                                 const Property& property) {
        return Method<Property>::WriteXML(file_path, property);
    }

 private:
    /*
     * @brief Hide constructor to specify it's a pure method class
     */
    ConfigurationReader();
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
