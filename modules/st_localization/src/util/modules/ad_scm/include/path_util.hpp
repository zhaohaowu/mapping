/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan<weizhengyuan@sensetime.com>
 */
#pragma once

#include <unistd.h>
#include <sys/stat.h>  // stat
#include <string>
#include <vector>

namespace senseAD {
namespace common {
namespace utils {

#define JOIN_PATH(root, path) \
    (root.back() == '/' ? root + path : root + '/' + path)

class FileSystem {
 public:
    /*
     * @brief check the "file"
     * @param path[in] specify the "file" path
     * @return int flag to specify the status of "file".
     *  -1 means invalid;
     *   0 means Neither file nor directory
     *   1 means it's a regular file
     *   2 menas it's a directory
     */
    static int FileStatus(const std::string& path);
    /*
     * @brief check if a regular file
     * @param path[in] specify the "file" path
     * @return true if it's a regular file
     */
    static bool IsFile(const std::string& path) {
        return 1 == FileStatus(path);
    }
    /*
     * @brief check if a directory
     * @param path[in] specify the "file" path
     * @return true if it's a regular directory
     */
    static bool IsDir(const std::string& path) { return 2 == FileStatus(path); }
    /*
     * @brief List all the files
     * @param path[in] specify the folder
     * @param is_recursive [in] specify if search the sub-folder recursively
     * @param prefix [in] specify the match pattern
     * @return a vector containing the absolute paths
     */
    static std::vector<std::string> GetFileList(const std::string& folder,
                                                bool is_recursive = false,
                                                const std::string& prefix = "");

    /*
     * @brief List all the directories
     * @param path[in] specify the folder
     * @param is_recursive [in] specify if search the sub-folder recursively
     * @param prefix [in] specify the match pattern
     * @return a vector containing the absolute paths
     */
    static std::vector<std::string> GetDirectoryList(
        const std::string& folder,
        bool is_recursive = false,
        const std::string& prefix = "");

    /*
     * @brief List all the objects in the current directory
     * @param path[in] specify the folder
     * @param status[in] specify the object type(default as 'regular file')
     * @param prefix [in] specify the match pattern
     * @return a vector containing the object names
     */
    static std::vector<std::string> ListDirectory(
        const std::string& folder,
        int status = 1,
        const std::string& prefix = "");

    /*
     * @brief Get the current working directory
     * @return the current working directory
     */
    static std::string CurrentDir();

    /*
     * @brief Get the project root
     * @return the project root
     */
    static std::string ProjectDir();

    /*
     * @brief call 'mkdir -p' to create directory(ies)
     * @return true if succeed
     */
    static bool MakeDirs(const std::string& dir);

    /*
     * @brief get std::string from environment variable
     * @param name [in] Specify the environment name
     * @return the result string from environment variable,
     *         if environment variable not existed, return std::string()
     */
    static std::string GetEnv(const std::string& name);

 private:
    static std::vector<std::string> GetObjectList(
        const std::string& folder,
        bool is_recursive = false,
        const std::string& prefix = "",
        int status = 1);

 private:
    // DISALLOW_IMPLICIT_CONSTRUCTORS
    FileSystem();
    FileSystem(const FileSystem&);
    FileSystem& operator=(const FileSystem&);
};

}  // namespace utils
}  // namespace common
}  // namespace senseAD
