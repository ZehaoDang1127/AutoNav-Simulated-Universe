// ROS-independent geometry shared by the line layer and Smac overlays.
// Copyright 2026 AutoNav contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace autonav_lane
{
struct Point {double x, y;};
struct Segment {Point a, b;};

inline std::vector<Segment> segments(const std::vector<double> & values)
{
  if (values.size() % 4 != 0) {throw std::invalid_argument("Lane segments require groups of four coordinates");}
  std::vector<Segment> result;
  for (size_t i = 0; i < values.size(); i += 4) {
    for (size_t j = i; j < i + 4; ++j) {
      if (!std::isfinite(values[j])) {throw std::invalid_argument("Non-finite lane coordinate");}
    }
    Segment line{{values[i], values[i+1]}, {values[i+2], values[i+3]}};
    if (std::hypot(line.b.x-line.a.x, line.b.y-line.a.y) < 1e-8) {
      throw std::invalid_argument("Zero-length lane segment");
    }
    result.push_back(line);
  }
  return result;
}

inline double distance(Point p, const Segment & s)
{
  const double dx = s.b.x-s.a.x, dy = s.b.y-s.a.y;
  const double t = std::max(0.0, std::min(1.0,
      ((p.x-s.a.x)*dx+(p.y-s.a.y)*dy)/(dx*dx+dy*dy)));
  return std::hypot(p.x-s.a.x-t*dx, p.y-s.a.y-t*dy);
}

inline bool near(Point p, const std::vector<Segment> & lines, double tolerance)
{
  return std::any_of(lines.begin(), lines.end(), [&](const Segment & s) {
      return distance(p, s) <= tolerance;
    });
}

// The hard classification wins where paint segments overlap or meet.
inline bool crossable(Point p, const std::vector<Segment> & dividers,
  const std::vector<Segment> & boundaries, double tolerance)
{
  return !near(p, boundaries, tolerance) && near(p, dividers, tolerance);
}

inline unsigned int crossings(Point a, Point b, const std::vector<Segment> & lines)
{
  std::vector<double> hits;
  for (const auto & s : lines) {
    const double sx = s.b.x-s.a.x, sy = s.b.y-s.a.y;
    const double sa = sx*(a.y-s.a.y)-sy*(a.x-s.a.x);
    const double sb = sx*(b.y-s.a.y)-sy*(b.x-s.a.x);
    // Assign a point exactly on the divider to the nonnegative side. This
    // charges a split crossing once, independent of primitive sample density.
    if ((sa >= 0) == (sb >= 0)) {continue;}
    const double t = sa/(sa-sb);
    const Point hit{a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)};
    const double u = ((hit.x-s.a.x)*sx+(hit.y-s.a.y)*sy)/(sx*sx+sy*sy);
    if (u >= -1e-9 && u <= 1+1e-9) {hits.push_back(t);}
  }
  std::sort(hits.begin(), hits.end());
  hits.erase(std::unique(hits.begin(), hits.end(), [](double a, double b) {
      return std::abs(a-b) < 1e-8;
    }), hits.end());
  return static_cast<unsigned int>(hits.size());
}

inline unsigned char quadratic_cost(double distance_m, double inscribed_m, double reach_m)
{
  if (distance_m <= 1e-12) {return 254;}
  if (distance_m <= inscribed_m) {return 253;}
  if (reach_m <= 0 || distance_m >= inscribed_m+reach_m) {return 0;}
  const double fraction = 1-(distance_m-inscribed_m)/reach_m;
  return static_cast<unsigned char>(252*fraction*fraction);
}
}  // namespace autonav_lane
