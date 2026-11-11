import 'dart:math' as math;

/// Minimal pure-Dart 2D vector for cross-section geometry in `voxextrude`.
final class ExtVec2 {
  final double x;
  final double y;

  const ExtVec2(this.x, this.y);

  static const ExtVec2 zero = ExtVec2(0.0, 0.0);

  ExtVec2 operator +(ExtVec2 o) => ExtVec2(x + o.x, y + o.y);
  ExtVec2 operator -(ExtVec2 o) => ExtVec2(x - o.x, y - o.y);
  ExtVec2 operator -() => ExtVec2(-x, -y);
  ExtVec2 operator *(double s) => ExtVec2(x * s, y * s);
  ExtVec2 operator /(double s) => ExtVec2(x / s, y / s);

  double dot(ExtVec2 o) => x * o.x + y * o.y;
  double get lengthSquared => x * x + y * y;
  double get length => math.sqrt(lengthSquared);

  ExtVec2 get normalized {
    final len = length;
    if (len <= 1e-12) return const ExtVec2(0.0, 1.0);
    return ExtVec2(x / len, y / len);
  }

  static ExtVec2 lerp(ExtVec2 a, ExtVec2 b, double t) =>
      ExtVec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);

  @override
  String toString() => 'ExtVec2(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)})';
}

/// Minimal pure-Dart 3D vector for spline sweeping and mesh generation in `voxextrude`.
final class ExtVec3 {
  final double x;
  final double y;
  final double z;

  const ExtVec3(this.x, this.y, this.z);

  static const ExtVec3 zero = ExtVec3(0.0, 0.0, 0.0);
  static const ExtVec3 unitX = ExtVec3(1.0, 0.0, 0.0);
  static const ExtVec3 unitY = ExtVec3(0.0, 1.0, 0.0);
  static const ExtVec3 unitZ = ExtVec3(0.0, 0.0, 1.0);

  ExtVec3 operator +(ExtVec3 o) => ExtVec3(x + o.x, y + o.y, z + o.z);
  ExtVec3 operator -(ExtVec3 o) => ExtVec3(x - o.x, y - o.y, z - o.z);
  ExtVec3 operator -() => ExtVec3(-x, -y, -z);
  ExtVec3 operator *(double s) => ExtVec3(x * s, y * s, z * s);
  ExtVec3 operator /(double s) => ExtVec3(x / s, y / s, z / s);

  double dot(ExtVec3 o) => x * o.x + y * o.y + z * o.z;

  ExtVec3 cross(ExtVec3 o) => ExtVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  ExtVec3 get normalized {
    final len = length;
    if (len <= 1e-12) return ExtVec3.unitZ;
    return ExtVec3(x / len, y / len, z / len);
  }

  double distanceTo(ExtVec3 o) => (this - o).length;

  static ExtVec3 lerp(ExtVec3 a, ExtVec3 b, double t) => ExtVec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      );

  @override
  String toString() =>
      'ExtVec3(${x.toStringAsFixed(3)}, ${y.toStringAsFixed(3)}, ${z.toStringAsFixed(3)})';
}
