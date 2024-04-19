/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan<weizhengyuan@sensetime.com>
 * GUO Zhichong<guozhichong@sensetime.com>
 */
#include "path_util.hpp"

#include <dirent.h>  // DIR/opendir/dirent/readdir
#include <cstdlib>   // getenv
#include <string.h>

namespace senseAD {
namespace common {
namespace utils {

int FileSystem::FileStatus(const std::string& path) {
    struct stat sb;
    if (stat(path.c_str(), &sb) == -1) {
        return -1;
    }
    switch (sb.st_mode & S_IFMT) {
        case S_IFDIR:
            return 2;
        case S_IFREG:
            return 1;
        default:
            return 0;
    }
    return 0;
}

std::string FileSystem::CurrentDir() {
    char buff[1024];
    ssize_t len = readlink("/proc/self/exe", buff, sizeof(buff) - 1);
    if (len != -1) {
        buff[len] = '\0';
        std::string temp(buff);
        return temp.substr(0, temp.find_last_of('/'));
    }
    return std::string("");
}

std::string FileSystem::ProjectDir() {
    std::string current_dir = CurrentDir();
    const std::string project_name = "senseauto";  // git project name
    std::size_t found = current_dir.rfind("/" + project_name + "/");
    if (found != std::string::npos) {
        return current_dir.substr(0, found + project_name.length() + 1);
    }
    return std::string(".");
}

bool FileSystem::MakeDirs(const std::string& dir) {
    // A fastest version implemented first
    std::string command = std::string("mkdir -p ") + dir;
    int nerrno = system(command.c_str());
    return 0 == nerrno;
}

std::vector<std::string> FileSystem::GetFileList(const std::string& folder,
                                                 bool is_recursive,
                                                 const std::string& prefix) {
    return std::move(GetObjectList(folder, is_recursive, prefix, 1));
}

std::vector<std::string> FileSystem::GetDirectoryList(
    const std::string& folder, bool is_recursive, const std::string& prefix) {
    return std::move(GetObjectList(folder, is_recursive, prefix, 2));
}

std::vector<std::string> FileSystem::GetObjectList(const std::string& folder,
                                                   bool is_recursive,
                                                   const std::string& prefix,
                                                   int status) {
    DIR* dir = opendir(folder.c_str());
    std::vector<std::string> file_list;
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != NULL) {
            // Skip "." and ".."
            if (0 == strcmp(ent->d_name, ".") ||
                0 == strcmp(ent->d_name, "..")) {
                continue;
            }
            std::string&& temp = folder + "/" + std::string(ent->d_name);
            int ret = FileStatus(temp);
            if (ret == status) {
                // Skip unsatisfied file and directory
                if (!prefix.empty() &&
                    0 != std::string(ent->d_name).find(prefix)) {
                    continue;
                }
                file_list.emplace_back(temp);
            } else if (is_recursive && ret == 2) {
                std::vector<std::string> sub_files =
                    GetObjectList(temp, is_recursive, prefix, status);
                file_list.insert(file_list.begin(), sub_files.begin(),
                                 sub_files.end());
            }
        }
        closedir(dir);
    } else {
        printf("Cannot open the directory. %s\n", folder.c_str());
    }
    return std::move(file_list);
}

std::vector<std::string> FileSystem::ListDirectory(const std::string& folder,
                                                   int status,
                                                   const std::string& prefix) {
    DIR* dir = opendir(folder.c_str());
    std::vector<std::string> object_list;
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != NULL) {
            // Skip "." and ".."
            if (0 == strcmp(ent->d_name, ".") ||
                0 == strcmp(ent->d_name, "..")) {
                continue;
            }
            std::string&& object_name(ent->d_name);
            std::string&& temp = folder + "/" + object_name;
            int ret = FileStatus(temp);
            if (ret == status) {
                // Skip unsatisfied file and directory
                if (!prefix.empty() && 0 != object_name.find(prefix)) {
                    continue;
                }
                object_list.emplace_back(object_name);
            }
        }
        closedir(dir);
    } else {
        printf("Cannot open the directory. %s\n", folder.c_str());
    }
    return std::move(object_list);
}

std::string FileSystem::GetEnv(const std::string& name) {
    char* p = getenv(name.c_str());
    if (p != nullptr) {
        return std::move(std::string(p));
    } else {
        return std::string();
    }
}

}  // namespace utils
}  // namespace common
}  // namespace senseAD
