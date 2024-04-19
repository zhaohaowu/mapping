/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/base/base_element.hpp"
#include "semantic_mm/base/line_segment.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class LaneLine : public BaseElement {
 public:
  DEFINE_SMART_PTR(LaneLine)
  DEFINE_PTR_CONTAINER(LaneLine)

  LaneLine() = default;
  LaneLine(id_t id, const std::vector<LineSegment::Ptr>& line_segments);
  explicit LaneLine(const LineData& data);

  ~LaneLine() override {}

  void SetLinkedId(id_t id);
  id_t GetLinkedId() const;

  void AddLineSegment(const LineSegment::Ptr& line_segment);
  void SetLineSegments(const std::vector<LineSegment::Ptr>& line_segs);
  void SetLineSegments(std::vector<LineSegment::Ptr> line_segs);
  std::vector<LineSegment::Ptr>& GetLineSegments();
  const std::vector<LineSegment::Ptr>& GetLineSegments() const;

 private:
  // line segments, for now, just 0-index segment is valid
  std::vector<LineSegment::Ptr> line_segments_;

  // linked laneline id, using for roadside line
  id_t linked_id_{-1};
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
