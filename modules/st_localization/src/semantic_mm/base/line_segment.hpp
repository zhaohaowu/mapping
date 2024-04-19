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
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/base/base_element.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class LineSegment {
 public:
  DEFINE_SMART_PTR(LineSegment)
  DEFINE_PTR_CONTAINER(LineSegment)

  LineSegment() = default;
  LineSegment(const id_t& id, float32_t confidence, const double width,
              const Color color, const LineType& line_type,
              const LineStyle& type, const std::vector<Point3D_t>& points);
  explicit LineSegment(const LineSegment& other);
  explicit LineSegment(const LineSegmentData& data);

  ~LineSegment() = default;

  void SetId(const id_t& id);
  id_t GetId() const;

  void SetConfidence(float32_t confidence);
  float32_t GetConfidence() const;

  void SetWidth(float32_t width);
  float32_t GetWidth() const;

  void SetPoints(const std::vector<Point3D_t>& points);
  void SetPoints(std::vector<Point3D_t>&& points);
  std::vector<Point3D_t>& GetPoints();
  const std::vector<Point3D_t>& GetPoints() const;

  void SetColor(const Color& color);
  Color GetColor() const;

  void SetLineType(const LineType& line_type);
  LineType GetLineType() const;

  void SetLineStyle(const LineStyle& line_style);
  LineStyle GetLineStyle() const;

 private:
  id_t id_ = 0;                 // element unique id
  float32_t confidence_ = 0.0;  // confidence

  float32_t width_ = 0.15;                     // line width
  Color color_ = Color::NONE;                  // line color property
  LineType line_type_ = LineType::Unknown;     // line type property
  LineStyle line_style_ = LineStyle::Unknown;  // line style property

  std::vector<Point3D_t> points_;  // sampled points in line segment
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
