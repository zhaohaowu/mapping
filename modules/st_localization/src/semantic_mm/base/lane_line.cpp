/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/base/lane_line.hpp"

#include <utility>

#include "semantic_mm/base/line_segment.hpp"

namespace senseAD {
namespace localization {
namespace smm {

LaneLine::LaneLine(id_t id, const std::vector<LineSegment::Ptr>& line_segments)
    : BaseElement(id), line_segments_(line_segments) {}

LaneLine::LaneLine(const LineData& data) : BaseElement(data.id) {
  for (size_t i = 0; i < data.line_segments.size(); i++) {
    line_segments_.push_back(
        std::make_shared<LineSegment>(data.line_segments[i]));
  }
}

void LaneLine::SetLinkedId(id_t id) { linked_id_ = id; }

id_t LaneLine::GetLinkedId() const { return linked_id_; }

void LaneLine::AddLineSegment(const LineSegment::Ptr& line_segment) {
  line_segments_.push_back(line_segment);
}

void LaneLine::SetLineSegments(const std::vector<LineSegment::Ptr>& line_segs) {
  line_segments_ = line_segs;
}

void LaneLine::SetLineSegments(std::vector<LineSegment::Ptr> line_segs) {
  line_segments_ = std::move(line_segs);
}

std::vector<LineSegment::Ptr>& LaneLine::GetLineSegments() {
  return line_segments_;
}

const std::vector<LineSegment::Ptr>& LaneLine::GetLineSegments() const {
  return line_segments_;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
