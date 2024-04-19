/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/base/line_segment.hpp"

#include <utility>

namespace senseAD {
namespace localization {
namespace smm {

LineSegment::LineSegment(const id_t& id, float32_t confidence,
                         const double width, const Color color,
                         const LineType& line_type, const LineStyle& line_style,
                         const std::vector<Point3D_t>& points)
    : id_(id),
      confidence_(confidence),
      width_(width),
      color_(color),
      line_type_(line_type),
      line_style_(line_style),
      points_(points) {}

LineSegment::LineSegment(const LineSegment& other)
    : id_(other.id_),
      confidence_(other.confidence_),
      width_(other.width_),
      color_(other.color_),
      line_type_(other.line_type_),
      line_style_(other.line_style_),
      points_(other.points_) {}

LineSegment::LineSegment(const LineSegmentData& data)
    : id_(data.id),
      confidence_(data.confidence),
      width_(data.width),
      color_(data.color),
      line_type_(data.line_type),
      line_style_(data.line_style),
      points_(data.points) {}

void LineSegment::SetId(const id_t& id) { id_ = id; }

id_t LineSegment::GetId() const { return id_; }

void LineSegment::SetConfidence(float32_t confidence) {
  confidence_ = confidence;
}

float32_t LineSegment::GetConfidence() const { return confidence_; }

void LineSegment::SetWidth(float32_t width) { width_ = width; }

float32_t LineSegment::GetWidth() const { return width_; }

void LineSegment::SetPoints(const std::vector<Point3D_t>& points) {
  points_ = points;
}

void LineSegment::SetPoints(std::vector<Point3D_t>&& points) {
  points_ = std::move(points);
}

std::vector<Point3D_t>& LineSegment::GetPoints() { return points_; }

const std::vector<Point3D_t>& LineSegment::GetPoints() const { return points_; }

void LineSegment::SetColor(const Color& color) { color_ = color; }

Color LineSegment::GetColor() const { return color_; }

void LineSegment::SetLineType(const LineType& line_type) {
  line_type_ = line_type;
}

LineType LineSegment::GetLineType() const { return line_type_; }

void LineSegment::SetLineStyle(const LineStyle& line_style) {
  line_style_ = line_style;
}

LineStyle LineSegment::GetLineStyle() const { return line_style_; }

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
