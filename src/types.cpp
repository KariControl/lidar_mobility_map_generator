#include "lidar_mobility_map_generator/types.hpp"

#include <cmath>
#include <stdexcept>

namespace lidar_mobility_map_generator
{

bool Quaternion::isFinite() const
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
}

double Quaternion::squaredNorm() const
{
  return x * x + y * y + z * z + w * w;
}

Quaternion Quaternion::normalized() const
{
  const double length = std::sqrt(squaredNorm());
  if (length < 1.0e-15) {
    return {};
  }
  return {x / length, y / length, z / length, w / length};
}

Quaternion Quaternion::inverse() const
{
  const double value = squaredNorm();
  if (value < 1.0e-15) {
    return {};
  }
  return {-x / value, -y / value, -z / value, w / value};
}

Quaternion Quaternion::operator*(const Quaternion & rhs) const
{
  return {
    w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
    w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
    w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
    w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z};
}

Vec3 Quaternion::rotate(const Vec3 & point) const
{
  const Quaternion q = normalized();
  const Vec3 vector{q.x, q.y, q.z};
  const Vec3 cross_1{
    vector.y * point.z - vector.z * point.y,
    vector.z * point.x - vector.x * point.z,
    vector.x * point.y - vector.y * point.x};
  const Vec3 cross_2{
    vector.y * cross_1.z - vector.z * cross_1.y,
    vector.z * cross_1.x - vector.x * cross_1.z,
    vector.x * cross_1.y - vector.y * cross_1.x};
  return point + cross_1 * (2.0 * q.w) + cross_2 * 2.0;
}

double Quaternion::yaw() const
{
  const Quaternion q = normalized();
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

Quaternion Quaternion::fromYaw(const double yaw_value)
{
  const double half = 0.5 * yaw_value;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
}

Quaternion Quaternion::slerp(
  const Quaternion & lhs_value, const Quaternion & rhs_value, const double ratio_value)
{
  Quaternion lhs = lhs_value.normalized();
  Quaternion rhs = rhs_value.normalized();
  const double ratio = clamp(ratio_value, 0.0, 1.0);
  double cosine = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;

  if (cosine < 0.0) {
    rhs = {-rhs.x, -rhs.y, -rhs.z, -rhs.w};
    cosine = -cosine;
  }

  if (cosine > 0.9995) {
    return Quaternion{
      lhs.x + ratio * (rhs.x - lhs.x),
      lhs.y + ratio * (rhs.y - lhs.y),
      lhs.z + ratio * (rhs.z - lhs.z),
      lhs.w + ratio * (rhs.w - lhs.w)}.normalized();
  }

  const double theta = std::acos(clamp(cosine, -1.0, 1.0));
  const double sine = std::sin(theta);
  const double lhs_weight = std::sin((1.0 - ratio) * theta) / sine;
  const double rhs_weight = std::sin(ratio * theta) / sine;
  return {
    lhs_weight * lhs.x + rhs_weight * rhs.x,
    lhs_weight * lhs.y + rhs_weight * rhs.y,
    lhs_weight * lhs.z + rhs_weight * rhs.z,
    lhs_weight * lhs.w + rhs_weight * rhs.w};
}

Vec3 Transform::apply(const Vec3 & point) const
{
  return rotation.rotate(point) + translation;
}

Transform Transform::inverse() const
{
  const Quaternion inverse_rotation = rotation.inverse().normalized();
  return {inverse_rotation.rotate(translation * -1.0), inverse_rotation};
}

Transform Transform::operator*(const Transform & rhs) const
{
  return {apply(rhs.translation), (rotation * rhs.rotation).normalized()};
}

Transform Transform::interpolate(
  const Transform & lhs, const Transform & rhs, const double ratio_value)
{
  const double ratio = clamp(ratio_value, 0.0, 1.0);
  return {
    lhs.translation + (rhs.translation - lhs.translation) * ratio,
    Quaternion::slerp(lhs.rotation, rhs.rotation, ratio)};
}

const char * toString(const RouteNodeType type)
{
  switch (type) {
    case RouteNodeType::kEndpoint:
      return "endpoint";
    case RouteNodeType::kJunction:
      return "junction";
    case RouteNodeType::kNormal:
    default:
      return "normal";
  }
}

double polylineLength(const std::vector<Vec3> & points)
{
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance3d(points[index - 1U], points[index]);
  }
  return result;
}

}  // namespace lidar_mobility_map_generator
