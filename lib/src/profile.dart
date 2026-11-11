import 'dart:math' as math;

import 'math.dart';

/// A vertex along a 2D extrusion cross-section in `(right, up)` local coordinates.
final class CrossSectionPoint {
  final ExtVec2 position;
  final double u;
  final ExtVec2? explicitNormal;

  CrossSectionPoint({
    required double x,
    required double y,
    this.u = 0.0,
    ExtVec2? normal,
  })  : position = ExtVec2(x, y),
        explicitNormal = normal;

  const CrossSectionPoint.vec(
    this.position, {
    this.u = 0.0,
    ExtVec2? normal,
  }) : explicitNormal = normal;
}

/// Describes a 2D cross-section polygon or open strip swept along a 3D spline.
///
/// For open strips (roads, river ribbons), points ordered from left (`-X`) to
/// right (`+X`) produce upward (`+Y`) outward normals under the sweep frame
/// `right = tangent x up`.
/// For closed shapes (pipes, tunnels, beams), points ordered clockwise in
/// `(right, up)` (e.g. top -> right -> bottom -> left) produce outward normals.
final class CrossSectionProfile {
  final List<CrossSectionPoint> points;
  final bool closed;

  CrossSectionProfile({
    required List<CrossSectionPoint> points,
    this.closed = false,
  }) : points = List.unmodifiable(points) {
    if (points.length < 2) {
      throw ArgumentError.value(
        points.length,
        'points.length',
        'CrossSectionProfile requires at least 2 points',
      );
    }
  }

  /// Resolves per-point 2D outward normals (using explicit normals where supplied,
  /// otherwise averaging adjacent edge normals `(-dy, dx)`).
  List<ExtVec2> resolveNormals() {
    final n = points.length;
    final hasCoincidentSeam = closed &&
        (points.last.position - points.first.position).lengthSquared <= 1e-10;
    final uniqueSegCount = (closed && !hasCoincidentSeam) ? n : n - 1;

    final segNormals = List<ExtVec2>.generate(uniqueSegCount, (i) {
      final a = points[i].position;
      final b = points[(i + 1) % n].position;
      final d = b - a;
      if (d.lengthSquared <= 1e-12) return const ExtVec2(0.0, 1.0);
      return ExtVec2(-d.y, d.x).normalized;
    });

    final normals = List<ExtVec2>.filled(n, const ExtVec2(0.0, 1.0));
    for (var i = 0; i < n; i++) {
      final explicit = points[i].explicitNormal;
      if (explicit != null) {
        normals[i] = explicit.normalized;
        continue;
      }

      if (hasCoincidentSeam && (i == 0 || i == n - 1)) {
        final sum = segNormals.first + segNormals.last;
        normals[i] = sum.lengthSquared > 1e-12 ? sum.normalized : segNormals.first;
      } else if (closed && !hasCoincidentSeam) {
        final prev = segNormals[(i - 1 + uniqueSegCount) % uniqueSegCount];
        final next = segNormals[i % uniqueSegCount];
        final sum = prev + next;
        normals[i] = sum.lengthSquared > 1e-12 ? sum.normalized : next;
      } else if (i == 0) {
        normals[i] = segNormals.first;
      } else if (i == n - 1) {
        normals[i] = segNormals.last;
      } else {
        final sum = segNormals[i - 1] + segNormals[i];
        normals[i] = sum.lengthSquared > 1e-12 ? sum.normalized : segNormals[i];
      }
    }
    return normals;
  }

  /// Cambered 2-lane roadbed with chamfered raised curbs and side aprons (`closed: false`).
  factory CrossSectionProfile.road({
    double width = 6.0,
    double crownHeight = 0.08,
    double curbWidth = 0.35,
    double curbHeight = 0.14,
  }) {
    if (width <= 0) throw ArgumentError.value(width, 'width', 'must be > 0');
    final halfRoad = width * 0.5;
    final outerX = halfRoad + curbWidth;
    final bevel = math.min(0.08, curbWidth * 0.25);
    final pts = <CrossSectionPoint>[
      CrossSectionPoint(x: -outerX - 0.15, y: 0.0, u: 0.0),
      CrossSectionPoint(x: -outerX, y: curbHeight, u: 0.08),
      CrossSectionPoint(x: -halfRoad - bevel, y: curbHeight, u: 0.14),
      CrossSectionPoint(x: -halfRoad, y: 0.0, u: 0.18),
      CrossSectionPoint(x: 0.0, y: crownHeight, u: 0.5),
      CrossSectionPoint(x: halfRoad, y: 0.0, u: 0.82),
      CrossSectionPoint(x: halfRoad + bevel, y: curbHeight, u: 0.86),
      CrossSectionPoint(x: outerX, y: curbHeight, u: 0.92),
      CrossSectionPoint(x: outerX + 0.15, y: 0.0, u: 1.0),
    ];
    return CrossSectionProfile(points: pts, closed: false);
  }

  /// Flat or subtly concave water/river ribbon strip (`closed: false`).
  factory CrossSectionProfile.riverRibbon({
    double width = 5.0,
    int subdivisions = 4,
    double centerDip = 0.0,
  }) {
    if (width <= 0) throw ArgumentError.value(width, 'width', 'must be > 0');
    if (subdivisions < 1) {
      throw ArgumentError.value(subdivisions, 'subdivisions', 'must be >= 1');
    }
    final halfW = width * 0.5;
    final pts = <CrossSectionPoint>[];
    for (var i = 0; i <= subdivisions; i++) {
      final t = i / subdivisions;
      final x = -halfW + width * t;
      final par = 1.0 - (t * 2.0 - 1.0) * (t * 2.0 - 1.0);
      final y = -centerDip * par;
      pts.add(CrossSectionPoint(x: x, y: y, u: t));
    }
    return CrossSectionProfile(points: pts, closed: false);
  }

  /// Smooth circular pipe / rail cross-section (`closed: true`) with UV seam `0..1`.
  factory CrossSectionProfile.circle({
    double radius = 0.5,
    int segments = 12,
  }) {
    if (radius <= 0) throw ArgumentError.value(radius, 'radius', 'must be > 0');
    if (segments < 3) throw ArgumentError.value(segments, 'segments', 'must be >= 3');

    final pts = <CrossSectionPoint>[];
    for (var i = 0; i <= segments; i++) {
      final t = i / segments;
      final angle = t * math.pi * 2.0;
      final nx = math.sin(angle);
      final ny = math.cos(angle);
      pts.add(
        CrossSectionPoint(
          x: nx * radius,
          y: ny * radius,
          u: t,
          normal: ExtVec2(nx, ny),
        ),
      );
    }
    return CrossSectionProfile(points: pts, closed: true);
  }

  /// Hard-edged architectural box girder cross-section (`closed: true`) with split normals.
  factory CrossSectionProfile.boxBeam({
    double width = 1.0,
    double height = 0.5,
  }) {
    if (width <= 0) throw ArgumentError.value(width, 'width', 'must be > 0');
    if (height <= 0) throw ArgumentError.value(height, 'height', 'must be > 0');

    final hw = width * 0.5;
    final hh = height * 0.5;
    const upN = ExtVec2(0.0, 1.0);
    const rightN = ExtVec2(1.0, 0.0);
    const downN = ExtVec2(0.0, -1.0);
    const leftN = ExtVec2(-1.0, 0.0);

    return CrossSectionProfile(
      closed: true,
      points: [
        CrossSectionPoint(x: -hw, y: hh, u: 0.0, normal: upN),
        CrossSectionPoint(x: hw, y: hh, u: 0.25, normal: upN),
        CrossSectionPoint(x: hw, y: hh, u: 0.25, normal: rightN),
        CrossSectionPoint(x: hw, y: -hh, u: 0.50, normal: rightN),
        CrossSectionPoint(x: hw, y: -hh, u: 0.50, normal: downN),
        CrossSectionPoint(x: -hw, y: -hh, u: 0.75, normal: downN),
        CrossSectionPoint(x: -hw, y: -hh, u: 0.75, normal: leftN),
        CrossSectionPoint(x: -hw, y: hh, u: 1.0, normal: leftN),
      ],
    );
  }
}
