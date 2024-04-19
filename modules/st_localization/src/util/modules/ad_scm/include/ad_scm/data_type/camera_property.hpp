/*
 * Copyright (C) 2017-2019 by SenseTime Group Limited. All rights reserved.
 * LiangYu <liangyu@sensetime.com>
 * Deng Long <denglong@sensetime.com>
 */

#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "ad_common/data_type/base.hpp"

namespace senseAD {

class CameraControl {
 public:
    class Specification {
     public:
        std::string name;
        std::vector<int32_t> ids;
    };
    std::unordered_map<std::string, Specification> specification;
};

class CameraConfig {
 public:
    uint32_t number;
    bool slave;
    virtual int32_t Number() const { return number; }
    virtual bool IsSlave() const { return slave; }
};

struct FrameSkip {
    uint32_t frame_record_num;
    uint32_t frame_skip_num;
};

// SoA
struct VirtualAttr {
    std::string protocal;
    std::string datatype;
    std::string path;
    bool loop;
};
struct VirtualAttrs : public CameraConfig {
    std::vector<VirtualAttr> configs;
    int32_t Number() const override { return configs.size(); }
    bool IsSlave() const override { return false; }
};
struct VirtualConfig {
    std::string TYPE;
    VirtualAttrs ATTRIBUTE;
};

struct IMX290Attr : public CameraConfig {
    std::string port;
    std::string framerate_in;
    std::string camera_type;
    std::string mask;
    std::string write_path;
    std::string max9286_address;
    std::string max9271_address;
    std::string sensor_address;
    std::string board;
    uint32_t i2c_device;
    uint32_t csi_lanes;
    bool serialize;
    uint32_t embedded_lines_top;
    uint32_t embedded_lines_bottom;
    FrameSkip camera_frame_skip;
};

struct IMX290Config {
    std::string TYPE;
    IMX290Attr ATTRIBUTE;
};

struct NvdwAttr : public CameraConfig {
    std::string port;
    std::string datatype;
    std::string protocal;
    std::string framerate_in;
    std::string camera_type;
    std::string mask;
    std::string serialize;
    std::string write_path;
    FrameSkip camera_frame_skip;
};

struct NvdwConfig {
    std::string TYPE;
    NvdwAttr ATTRIBUTE;
};

struct IMXCustomAttr : public CameraConfig {
    std::string port;
    std::string rig_file_path;
    std::string isp_file_path;
    std::string serialize;
    std::string write_path;
    FrameSkip camera_frame_skip;
};

struct IMXCustomConfig {
    std::string TYPE;
    IMXCustomAttr ATTRIBUTE;
};

struct IPCCameraAttr : public CameraConfig {
    std::string port;
    std::string process_idx;
};

struct IPCCameraConfig {
    std::string TYPE;
    IPCCameraAttr ATTRIBUTE;
};

struct CameraPylonId {
    std::string camera_name;
    std::string device_user_id;
    std::string param_set;
};

struct CameraPylonAttr : public CameraConfig {
    uint32_t port_id;
    uint32_t width;
    uint32_t height;
    uint32_t buffer_size;
    float framerate_in;
    std::string write_path;
    std::vector<CameraPylonId> cameras;
};

struct CameraPylonConfig {
    std::string TYPE;
    CameraPylonAttr ATTRIBUTE;
};

struct CameraZMQAttrEach {
    uint8_t id;
    std::string publish_ip;
    uint32_t publish_port;
    // TODO(chenshengjie): need width*height? also fps?
};

struct CameraZMQAttr : public CameraConfig {
    std::string ip;
    // TODO(chenshengjie): support several-port cameras on DCU
    uint32_t port_id;
    // TODO(chenshengjie): unify to control followings or in each camera?
    uint32_t width;
    uint32_t height;
    float framerate_in;
    // TODO(chenshengjie): reserved for control recording path on DCU?
    std::string write_path;
    std::vector<CameraZMQAttrEach> cameras;
};

struct CameraZMQConfig {
    std::string TYPE;
    CameraZMQAttr ATTRIBUTE;
};

struct CameraOfflineAttrEach {
    uint8_t id;
    std::string camera_name;
    std::string file_path;
    uint32_t encoder_width;
    uint32_t encoder_height;
};

struct CameraOfflineAttr : public CameraConfig {
    uint32_t width;
    uint32_t height;
    uint32_t buffer_size;
    std::string write_path;
    std::vector<CameraOfflineAttrEach> cameras;
};

struct CameraOfflineConfig {
    std::string TYPE;
    CameraOfflineAttr ATTRIBUTE;
};

struct CameraTdaAttrEach {
    uint8_t id;
    std::string camera_name;
    std::string mesh_table_file;
    uint32_t channel;
    uint32_t encoder_width;
    uint32_t encoder_height;
};

struct CameraTdaAttr : public CameraConfig {
    uint32_t width;
    uint32_t height;
    float framerate_in;
    uint32_t buffer_size;
    uint32_t sensor_index;
    uint32_t inst_id;
    std::string write_path;
    std::vector<CameraTdaAttrEach> cameras;
};

struct CameraTdaConfig {
    std::string TYPE;
    CameraTdaAttr ATTRIBUTE;
};

}  // namespace senseAD
