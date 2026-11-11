import 'dart:math' as math;

import 'math.dart';

/// A sampled station along a 3D spline with tangent, cumulative arc distance,
/// cross-section scale, and banking/roll twist.
final class SplineSample {
  final ExtVec3 position;
  final ExtVec3 tangent;
  final double distance;
  final double scaleX;
  final double scaleY;
  final double rollRadians;

  SplineSample({
    required this.position,
    required ExtVec3 tangent,
    required this.distance,
    this.scaleX = 1.0,
    this.scaleY = 1.0,
    this.rollRadians = 0.0,
  }) : tangent = tangent.lengthSquared > 1e-12 ? tangent.normalized : ExtVec3.unitZ;
}

/// Orthonormal cross-section coordinate frame at a [SplineSample].
///
/// Follows the right-handed sweep basis `(right, up, tangent)` where
/// `right = tangent x up`, `up = right x tangent`, and `right x (-up)...`:
/// specifically `right x tangent = up` and `tangent x right = -up` wait:
/// `(tangent x up) x tangent = up`! Wait: check:
/// `(T x U) x T = U * (T.T) - T * (U.T) = U` when `T` is unit and `T . U = 0`.
/// Therefore `right = T x U`, `R x T = U`, and `U x R = -T` wait:
/// `(T x U) . (U x R)` -> if `R = T x U`, `R x T = U`, so `T x R = -U`, and `U x R = -T`.
final class ExtrudedFrame {
  final ExtVec3 position;
  final ExtVec3 tangent;
  final ExtVec3 right;
  final ExtVec3 up;
  final double distance;
  final double scaleX;
  final double scaleY;

  const ExtrudedFrame({
    required this.position,
    required this.tangent,
    required this.right,
    required this.up,
    required this.distance,
    this.scaleX = 1.0,
    this.scaleY = 1.0,
  });

  /// Maps a 2D cross-section coordinate `(px, py)` into 3D world space.
  ExtVec3 transformPoint(ExtVec2 point) =>
      position + right * (point.x * scaleX) + up * (point.y * scaleY);

  /// Maps a 2D cross-section outward normal into unit 3D world space,
  /// accounting for inverse non-uniform scale.
  ExtVec3 transformNormal(ExtVec2 normal) {
    final sx = scaleX.abs() > 1e-6 ? scaleX : 1.0;
    final sy = scaleY.abs() > 1e-6 ? scaleY : 1.0;
    final world = right * (normal.x / sx) + up * (normal.y / sy);
    if (world.lengthSquared <= 1e-12) return up;
    return world.normalized;
  }
}

/// Computes twist-free rotation-minimizing Bishop frames (double-reflection) or
/// horizon-locked road/bridge frames along a sequence of [SplineSample]s.
final class RotationMinimizingFrame {
  const RotationMinimizingFrame._();

  /// Computes extruded frames for [samples].
  ///
  /// When [upLocked] is false (default), uses Wang et al. (2008) double-reflection
  /// parallel transport so vertical loops, corkscrews, and 3D splines never experience
  /// Frenet-Serret gimbal flips.
  ///
  /// When [upLocked] is true, projects [referenceUp] perpendicular to each sample's
  /// tangent so decks remain horizon-leveled for roads, rivers, and bridges.
  static List<ExtrudedFrame> computeFrames(
    List<SplineSample> samples, {
    ExtVec3 referenceUp = ExtVec3.unitY,
    bool upLocked = false,
  }) {
    if (samples.isEmpty) return const [];
    final refUp =
        referenceUp.lengthSquared > 1e-12 ? referenceUp.normalized : ExtVec3.unitY;

    final frames = <ExtrudedFrame>[];
    final t0 = samples.first.tangent;
    var r = _initialRightForTangent(t0, refUp);
    var u = r.cross(t0).normalized;

    for (var i = 0; i < samples.length; i++) {
      final sample = samples[i];
      final ti = sample.tangent;

      if (i > 0) {
        if (upLocked) {
          final projectedRight = ti.cross(refUp);
          if (projectedRight.lengthSquared > 1e-8) {
            r = projectedRight.normalized;
          } else {
            // Near-vertical tangent in up-locked mode: parallel-transport previous right.
            r = _transportRightDoubleReflection(
              samples[i - 1].position,
              samples[i - 1].tangent,
              sample.position,
              ti,
              r,
            );
          }
          u = r.cross(ti).normalized;
        } else {
          r = _transportRightDoubleReflection(
            samples[i - 1].position,
            samples[i - 1].tangent,
            sample.position,
            ti,
            r,
          );
          u = r.cross(ti).normalized;
        }
      }

      // Apply local roll twist in the (right, up) plane around tangent.
      var finalR = r;
      var finalU = u;
      if (sample.rollRadians.abs() > 1e-9) {
        final c = math.cos(sample.rollRadians);
        final s = math.sin(sample.rollRadians);
        finalR = (r * c + u * s).normalized;
        finalU = finalR.cross(ti).normalized;
      }

      frames.add(
        ExtrudedFrame(
          position: sample.position,
          tangent: ti,
          right: finalR,
          up: finalU,
          distance: sample.distance,
          scaleX: sample.scaleX,
          scaleY: sample.scaleY,
        ),
      );
    }

    return frames;
  }

  static ExtVec3 _initialRightForTangent(ExtVec3 tangent, ExtVec3 refUp) {
    final cross = tangent.cross(refUp);
    if (cross.lengthSquared > 1e-8) {
      return cross.normalized;
    }
    // Tangent is parallel to referenceUp; choose deterministic orthogonal axis.
    final alt = tangent.cross(ExtVec3.unitX);
    if (alt.lengthSquared > 1e-8) return alt.normalized;
    return tangent.cross(ExtVec3.unitZ).normalized;
  }

  static ExtVec3 _transportRightDoubleReflection(
    ExtVec3 p0,
    ExtVec3 t0,
    ExtVec3 p1,
    ExtVec3 t1,
    ExtVec3 r0,
  ) {
    final v1 = p1 - p0;
    final c1 = v1.dot(v1);

    ExtVec3 rL;
    ExtVec3 tL;
    if (c1 > 1e-12) {
      rL = r0 - v1 * (2.0 / c1 * v1.dot(r0));
      tL = t0 - v1 * (2.0 / c1 * v1.dot(t0));
    } else {
      rL = r0;
      tL = t0;
    }

    final v2 = t1 - tL;
    final c2 = v2.dot(v2);
    var r1 = c2 > 1e-12 ? (rL - v2 * (2.0 / c2 * v2.dot(rL))) : rL;

    // Gram-Schmidt orthogonalize against t1.
    r1 = r1 - t1 * r1.dot(t1);
    if (r1.lengthSquared <= 1e-12) {
      return _initialRightForTangent(t1, ExtVec3.unitY);
    }
    return r1.normalized;
  }
}

/// Resampling helpers converting control polylines or Catmull-Rom curves into [SplineSample]s.
final class ExtrusionResampler {
  const ExtrusionResampler._();

  /// Converts a 3D polyline into [SplineSample]s with central-difference tangents
  /// and exact cumulative arc distances.
  static List<SplineSample> fromPolyline(
    List<ExtVec3> points, {
    bool closed = false,
    double startRollRadians = 0.0,
    double endRollRadians = 0.0,
    double startScaleX = 1.0,
    double endScaleX = 1.0,
    double startScaleY = 1.0,
    double endScaleY = 1.0,
  }) {
    if (points.length < 2) {
      throw ArgumentError.value(points.length, 'points.length', 'must be >= 2');
    }

    final n = points.length;
    final distances = List<double>.filled(n, 0.0);
    var totalLen = 0.0;
    for (var i = 1; i < n; i++) {
      totalLen += points[i - 1].distanceTo(points[i]);
      distances[i] = totalLen;
    }

    final denom = totalLen > 1e-9 ? totalLen : 1.0;
    final samples = <SplineSample>[];

    for (var i = 0; i < n; i++) {
      ExtVec3 tangent;
      if (closed) {
        final prev = points[(i - 1 + n) % n];
        final next = points[(i + 1) % n];
        tangent = next - prev;
        if (tangent.lengthSquared <= 1e-12) {
          tangent = next - points[i];
        }
      } else if (i == 0) {
        tangent = points[1] - points[0];
      } else if (i == n - 1) {
        tangent = points[n - 1] - points[n - 2];
      } else {
        tangent = points[i + 1] - points[i - 1];
      }

      final t = distances[i] / denom;
      samples.add(
        SplineSample(
          position: points[i],
          tangent: tangent,
          distance: distances[i],
          scaleX: startScaleX + (endScaleX - startScaleX) * t,
          scaleY: startScaleY + (endScaleY - startScaleY) * t,
          rollRadians: startRollRadians + (endRollRadians - startRollRadians) * t,
        ),
      );
    }

    return samples;
  }

  /// Evaluates a centripetal/uniform Catmull-Rom spline through [controlPoints]
  /// into evenly subdivided [SplineSample] stations.
  static List<SplineSample> fromCatmullRom(
    List<ExtVec3> controlPoints, {
    bool closed = false,
    int subdivisionsPerSegment = 12,
  }) {
    if (controlPoints.length < 2) {
      throw ArgumentError.value(
        controlPoints.length,
        'controlPoints.length',
        'must be >= 2',
      );
    }
    if (subdivisionsPerSegment < 1) {
      throw ArgumentError.value(
        subdivisionsPerSegment,
        'subdivisionsPerSegment',
        'must be >= 1',
      );
    }

    final count = controlPoints.length;
    final segmentCount = closed ? count : count - 1;
    final evaluated = <ExtVec3>[];

    ExtVec3 pointAt(int index) {
      if (closed) {
        return controlPoints[(index % count + count) % count];
      }
      if (index < 0) {
        return controlPoints[0] * 2.0 - controlPoints[1];
      }
      if (index >= count) {
        return controlPoints[count - 1] * 2.0 - controlPoints[count - 2];
      }
      return controlPoints[index];
    }

    for (var seg = 0; seg < segmentCount; seg++) {
      final p0 = pointAt(seg - 1);
      final p1 = pointAt(seg);
      final p2 = pointAt(seg + 1);
      final p3 = pointAt(seg + 2);

      final steps = subdivisionsPerSegment;
      for (var step = 0; step < steps; step++) {
        final t = step / steps;
        evaluated.add(_evalCatmullRom(p0, p1, p2, p3, t));
      }
    }

    if (!closed) {
      evaluated.add(controlPoints.last);
    } else {
      evaluated.add(controlPoints.first);
    }

    return fromPolyline(evaluated, closed: closed);
  }

  static ExtVec3 _evalCatmullRom(
    ExtVec3 p0,
    ExtVec3 p1,
    ExtVec3 p2,
    ExtVec3 p3,
    double t,
  ) {
    final t2 = t * t;
    final t3 = t2 * t;
    return (p1 * 2.0 +
            (p2 - p0) * t +
            (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 +
            (-p0 + p1 * 3.0 - p2 * 3.0 + p3) * t3) *
        0.5;
  }
}
