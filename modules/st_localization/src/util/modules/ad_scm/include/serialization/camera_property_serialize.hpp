/*
 * Copyright (C) 2018~2020 by SenseTime Group Limited. All rights reserved.
 * Susichang <susichang@sensetime.com>
 * Deng Long <denglong@sensetime.com>
 */
#pragma once

#include <string>
#include <unistd.h>
// should always include following 4 cereal's archives
#include "cereal/archives/binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
// private user components header
#include "ad_scm/data_type/camera_property.hpp"
// public user components header
#include "ad_common/config_utils/macros.hpp"

namespace senseAD {

REGISTER_CEREAL_SERIALIZE(CameraControl::Specification& result) {  // NOLINT
    CEREAL_PAIR(result, name);
    CEREAL_PAIR(result, ids);
}

REGISTER_CEREAL_SERIALIZE(CameraControl& result) {  // NOLINT
    CEREAL_PAIR(result, specification);
}

// REGISTER_CEREAL_SERIALIZE(CameraConfig& result) {  // NOLINT
//    // TODO(someone): ...
//}

REGISTER_CEREAL_SERIALIZE(FrameSkip& result) {  // NOLINT
    CEREAL_PAIR(result, frame_record_num);
    CEREAL_PAIR(result, frame_skip_num);
}

REGISTER_CEREAL_SERIALIZE(IMX290Attr& result) {  // NOLINT
    CEREAL_PAIR(result, port);
    CEREAL_PAIR(result, framerate_in);
    CEREAL_PAIR(result, camera_type);
    CEREAL_PAIR(result, mask);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, max9286_address);
    CEREAL_PAIR(result, max9271_address);
    CEREAL_PAIR(result, sensor_address);
    CEREAL_PAIR(result, board);
    CEREAL_PAIR(result, i2c_device);
    CEREAL_PAIR(result, csi_lanes);
    CEREAL_PAIR(result, number);
    CEREAL_PAIR(result, serialize);
    CEREAL_PAIR(result, slave);
    CEREAL_PAIR(result, embedded_lines_top);
    CEREAL_PAIR(result, embedded_lines_bottom);
    CEREAL_PAIR(result, camera_frame_skip);
}

REGISTER_CEREAL_SERIALIZE(IMX290Config& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(NvdwAttr& result) {  // NOLINT
    CEREAL_PAIR(result, port);
    CEREAL_PAIR(result, datatype);
    CEREAL_PAIR(result, protocal);
    CEREAL_PAIR(result, framerate_in);
    CEREAL_PAIR(result, camera_type);
    CEREAL_PAIR(result, mask);
    CEREAL_PAIR(result, serialize);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, camera_frame_skip);
    CEREAL_PAIR(result, slave);
    CEREAL_PAIR(result, number);
}

REGISTER_CEREAL_SERIALIZE(NvdwConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(CameraPylonId& result) {  // NOLINT
    CEREAL_PAIR(result, camera_name);
    CEREAL_PAIR(result, device_user_id);
    CEREAL_PAIR(result, param_set);
}

REGISTER_CEREAL_SERIALIZE(CameraPylonAttr& result) {  // NOLINT
    CEREAL_PAIR(result, port_id);
    CEREAL_PAIR(result, width);
    CEREAL_PAIR(result, height);
    CEREAL_PAIR(result, buffer_size);
    CEREAL_PAIR(result, framerate_in);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, slave);
    CEREAL_PAIR(result, number);
    CEREAL_PAIR(result, cameras);
}

REGISTER_CEREAL_SERIALIZE(CameraPylonConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(CameraOfflineAttrEach& result) {  // NOLINT
    CEREAL_PAIR(result, id);
    CEREAL_PAIR(result, camera_name);
    CEREAL_PAIR(result, file_path);
    CEREAL_PAIR(result, encoder_width);
    CEREAL_PAIR(result, encoder_height);
}

REGISTER_CEREAL_SERIALIZE(CameraOfflineAttr& result) {  // NOLINT
    CEREAL_PAIR(result, width);
    CEREAL_PAIR(result, height);
    CEREAL_PAIR(result, buffer_size);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, cameras);
    CEREAL_PAIR(result, number);
}

REGISTER_CEREAL_SERIALIZE(CameraOfflineConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(CameraTdaAttrEach& result) {  // NOLINT
    CEREAL_PAIR(result, id);
    CEREAL_PAIR(result, camera_name);
    CEREAL_PAIR(result, mesh_table_file);
    CEREAL_PAIR(result, channel);
    CEREAL_PAIR(result, encoder_width);
    CEREAL_PAIR(result, encoder_height);
}

REGISTER_CEREAL_SERIALIZE(CameraTdaAttr& result) {  // NOLINT
    CEREAL_PAIR(result, width);
    CEREAL_PAIR(result, height);
    CEREAL_PAIR(result, framerate_in);
    CEREAL_PAIR(result, buffer_size);
    CEREAL_PAIR(result, sensor_index);
    CEREAL_PAIR(result, inst_id);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, cameras);
    CEREAL_PAIR(result, number);
}

REGISTER_CEREAL_SERIALIZE(CameraTdaConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(IMXCustomAttr& result) {  // NOLINT
    CEREAL_PAIR(result, port);
    CEREAL_PAIR(result, rig_file_path);
    CEREAL_PAIR(result, isp_file_path);
    CEREAL_PAIR(result, number);
    CEREAL_PAIR(result, serialize);
    CEREAL_PAIR(result, slave);
    CEREAL_PAIR(result, write_path);
    CEREAL_PAIR(result, camera_frame_skip);
}

REGISTER_CEREAL_SERIALIZE(IMXCustomConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

REGISTER_CEREAL_SERIALIZE(VirtualAttr& result) {  // NOLINT
    CEREAL_PAIR(result, protocal);
    CEREAL_PAIR(result, datatype);
    CEREAL_PAIR(result, path);
    CEREAL_PAIR(result, loop);
}

REGISTER_CEREAL_SERIALIZE(VirtualAttrs& result) {  // NOLINT
    CEREAL_PAIR(result, configs);
}

REGISTER_CEREAL_SERIALIZE(VirtualConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

template <class Archive>
void save(Archive& archive, const IPCCameraAttr& ipc_camera_attr) {  // NOLINT
    archive(cereal::make_nvp("port", ipc_camera_attr.port));
    auto output_process_idx = ipc_camera_attr.process_idx;
    auto pos = (output_process_idx).rfind('_');
    if (pos != std::string::npos) {
        output_process_idx = output_process_idx.substr(0, pos);
    }
    archive(cereal::make_nvp("process_idx", output_process_idx));
}

template <class Archive>
void load(Archive& archive, IPCCameraAttr& ipc_camera_attr) {  // NOLINT
    archive(cereal::make_nvp("port", ipc_camera_attr.port));
    std::string process_idx;
    archive(cereal::make_nvp("process_idx", process_idx));
    auto pid = getpid();
    std::stringstream ss;
    ss << process_idx << "_" << pid;
    ipc_camera_attr.process_idx = ss.str();
}

// REGISTER_CEREAL_SERIALIZE(IPCCameraAttr& result) {  // NOLINT
//    CEREAL_PAIR(result, port);
//    CEREAL_PAIR(result, process_idx);
//}

REGISTER_CEREAL_SERIALIZE(IPCCameraConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}

//--------------------------------- CameraZMQ ----------------------------------
REGISTER_CEREAL_SERIALIZE(CameraZMQAttrEach& attr_each) {  // NOLINT
    CEREAL_PAIR(attr_each, id);
    CEREAL_PAIR(attr_each, publish_ip);
    CEREAL_PAIR(attr_each, publish_port);
}

REGISTER_CEREAL_SERIALIZE(CameraZMQAttr& attr) {  // NOLINT
    CEREAL_PAIR(attr, ip);
    CEREAL_PAIR(attr, port_id);
    CEREAL_PAIR(attr, width);
    CEREAL_PAIR(attr, height);
    CEREAL_PAIR(attr, framerate_in);
    CEREAL_PAIR(attr, write_path);
    CEREAL_PAIR(attr, cameras);
    CEREAL_PAIR(attr, number);
}

REGISTER_CEREAL_SERIALIZE(CameraZMQConfig& result) {  // NOLINT
    CEREAL_PAIR(result, TYPE);
    CEREAL_PAIR(result, ATTRIBUTE);
}
//--------------------------------- CameraZMQ ----------------------------------

}  // namespace senseAD
