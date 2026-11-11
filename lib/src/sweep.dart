import 'dart:math' as math;
import 'dart:typed_data';

import 'framing.dart';
import 'math.dart';
import 'profile.dart';

/// Pure-Dart triangle mesh output produced by `SplineSweepBuilder` and `RoadIntersectionMeshBuilder`.
final class ExtrudedMeshData {
  /// Packed `Float32List` XYZ positions (`length == vertexCount * 3`).
  final Float32List positions;

  /// Packed `Float32List` XYZ unit normals (`length == vertexCount * 3`).
  final Float32List normals;

  /// Packed `Float32List` UV coordinates (`length == vertexCount * 2`).
  final Float32List uvs;

  /// Triangle index buffer (counter-clockwise winding for outward normals).
  final Uint32List indices;

  /// Axis-aligned bounding box minimum.
  final ExtVec3 boundsMin;

  /// Axis-aligned bounding box maximum.
  final ExtVec3 boundsMax;

  /// Total swept arc length along the path (`0.0` for junctions).
  final double totalArcLength;

  const ExtrudedMeshData({
    required this.positions,
    required this.normals,
    required this.uvs,
    required this.indices,
    required this.boundsMin,
    required this.boundsMax,
    this.totalArcLength = 0.0,
  });

  int get vertexCount => positions.length ~/ 3;
  int get triangleCount => indices.length ~/ 3;

  ExtVec3 positionAt(int vertexIndex) {
    final base = vertexIndex * 3;
    return ExtVec3(positions[base], positions[base + 1], positions[base + 2]);
  }

  ExtVec3 normalAt(int vertexIndex) {
    final base = vertexIndex * 3;
    return ExtVec3(normals[base], normals[base + 1], normals[base + 2]);
  }

  ExtVec2 uvAt(int vertexIndex) {
    final base = vertexIndex * 2;
    return ExtVec2(uvs[base], uvs[base + 1]);
  }
}

/// Sweeps a [CrossSectionProfile] along [SplineSample]s into a welded UV-tiled mesh.
final class SplineSweepBuilder {
  const SplineSweepBuilder._();

  static ExtrudedMeshData build({
    required List<SplineSample> samples,
    required CrossSectionProfile profile,
    bool upLocked = false,
    ExtVec3 referenceUp = ExtVec3.unitY,
    bool closedLoop = false,
    bool capEnds = true,
    double uvScaleU = 1.0,
    double uvScaleV = 0.25,
  }) {
    if (samples.length < 2) {
      throw ArgumentError.value(samples.length, 'samples.length', 'must be >= 2');
    }

    final frames = RotationMinimizingFrame.computeFrames(
      samples,
      referenceUp: referenceUp,
      upLocked: upLocked,
    );

    final profPts = profile.points;
    final profNorms = profile.resolveNormals();
    final hasCoincidentSeam = profile.closed &&
        (profPts.last.position - profPts.first.position).lengthSquared <= 1e-10;
    final needExtraClosingVertex = profile.closed && !hasCoincidentSeam;
    final ringVertexCount = needExtraClosingVertex ? profPts.length + 1 : profPts.length;
    final segmentQuadsPerRing = ringVertexCount - 1;
    final activeSegmentMask = List<bool>.generate(segmentQuadsPerRing, (i) {
      final a = profPts[i].position;
      final b = profPts[(i + 1) < profPts.length ? i + 1 : 0].position;
      return (b - a).lengthSquared > 1e-10;
    });

    final posList = <double>[];
    final normList = <double>[];
    final uvList = <double>[];
    final indices = <int>[];

    var minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;

    int addVertex(ExtVec3 p, ExtVec3 n, double u, double v) {
      final idx = posList.length ~/ 3;
      posList.add(p.x);
      posList.add(p.y);
      posList.add(p.z);
      final nn = n.normalized;
      normList.add(nn.x);
      normList.add(nn.y);
      normList.add(nn.z);
      uvList.add(u);
      uvList.add(v);

      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.z < minZ) minZ = p.z;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
      if (p.z > maxZ) maxZ = p.z;
      return idx;
    }

    // Generate swept station rings.
    for (var s = 0; s < frames.length; s++) {
      final frame = frames[s];
      final v = frame.distance * uvScaleV;
      for (var i = 0; i < ringVertexCount; i++) {
        final srcIdx = i < profPts.length ? i : 0;
        final pt = profPts[srcIdx];
        final worldPos = frame.transformPoint(pt.position);
        final worldNorm = frame.transformNormal(profNorms[srcIdx]);
        final u = (i >= profPts.length ? 1.0 : pt.u) * uvScaleU;
        addVertex(worldPos, worldNorm, u, v);
      }
    }

    // Connect consecutive rings with outward-wound triangles (skipping normal-split seams).
    final ringPairs = frames.length - 1;
    for (var s = 0; s < ringPairs; s++) {
      final rowA = s * ringVertexCount;
      final rowB = (s + 1) * ringVertexCount;
      for (var i = 0; i < segmentQuadsPerRing; i++) {
        if (!activeSegmentMask[i]) continue;
        final i00 = rowA + i;
        final i01 = rowA + i + 1;
        final i10 = rowB + i;
        final i11 = rowB + i + 1;

        indices.add(i00);
        indices.add(i01);
        indices.add(i11);

        indices.add(i00);
        indices.add(i11);
        indices.add(i10);
      }
    }

    // Close loop back to first station if requested and endpoints aren't already duplicated.
    var totalArcLength = samples.last.distance;
    if (closedLoop) {
      final endPos = frames.last.position;
      final startPos = frames.first.position;
      final wrapDist = endPos.distanceTo(startPos);
      if (wrapDist > 1e-5) {
        totalArcLength += wrapDist;
        final closingRingStart = posList.length ~/ 3;
        final frame = frames.first;
        final v = totalArcLength * uvScaleV;
        for (var i = 0; i < ringVertexCount; i++) {
          final srcIdx = i < profPts.length ? i : 0;
          final pt = profPts[srcIdx];
          final worldPos = frame.transformPoint(pt.position);
          final worldNorm = frame.transformNormal(profNorms[srcIdx]);
          final u = (i >= profPts.length ? 1.0 : pt.u) * uvScaleU;
          addVertex(worldPos, worldNorm, u, v);
        }

        final rowA = (frames.length - 1) * ringVertexCount;
        final rowB = closingRingStart;
        for (var i = 0; i < segmentQuadsPerRing; i++) {
          if (!activeSegmentMask[i]) continue;
          final i00 = rowA + i;
          final i01 = rowA + i + 1;
          final i10 = rowB + i;
          final i11 = rowB + i + 1;

          indices.add(i00);
          indices.add(i01);
          indices.add(i11);

          indices.add(i00);
          indices.add(i11);
          indices.add(i10);
        }
      }
    }

    // Emit centroid fan end caps when profile is closed and sweep is open.
    if (capEnds && profile.closed && !closedLoop) {
      _emitCap(
        frame: frames.first,
        profile: profile,
        isEndCap: false,
        addVertex: addVertex,
        indices: indices,
      );
      _emitCap(
        frame: frames.last,
        profile: profile,
        isEndCap: true,
        addVertex: addVertex,
        indices: indices,
      );
    }

    if (posList.isEmpty) {
      minX = minY = minZ = maxX = maxY = maxZ = 0.0;
    }

    return ExtrudedMeshData(
      positions: Float32List.fromList(posList),
      normals: Float32List.fromList(normList),
      uvs: Float32List.fromList(uvList),
      indices: Uint32List.fromList(indices),
      boundsMin: ExtVec3(minX, minY, minZ),
      boundsMax: ExtVec3(maxX, maxY, maxZ),
      totalArcLength: totalArcLength,
    );
  }

  static void _emitCap({
    required ExtrudedFrame frame,
    required CrossSectionProfile profile,
    required bool isEndCap,
    required int Function(ExtVec3 p, ExtVec3 n, double u, double v) addVertex,
    required List<int> indices,
  }) {
    final pts = profile.points;
    final hasCoincidentSeam =
        (pts.last.position - pts.first.position).lengthSquared <= 1e-10;
    final uniqueCount = hasCoincidentSeam ? pts.length - 1 : pts.length;
    if (uniqueCount < 3) return;

    var sum2 = ExtVec2.zero;
    var minX = double.infinity, minY = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity;
    for (var i = 0; i < uniqueCount; i++) {
      final p = pts[i].position;
      sum2 += p;
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }

    final centroid2 = sum2 / uniqueCount.toDouble();
    final spanX = math.max(maxX - minX, 1e-5);
    final spanY = math.max(maxY - minY, 1e-5);

    final capNormal = isEndCap ? frame.tangent : -frame.tangent;
    final centerIdx = addVertex(
      frame.transformPoint(centroid2),
      capNormal,
      0.5,
      0.5,
    );

    final ringIndices = List<int>.generate(uniqueCount, (i) {
      final p = pts[i].position;
      return addVertex(
        frame.transformPoint(p),
        capNormal,
        (p.x - minX) / spanX,
        (p.y - minY) / spanY,
      );
    });

    for (var i = 0; i < uniqueCount; i++) {
      final a = ringIndices[i];
      final b = ringIndices[(i + 1) % uniqueCount];
      if (isEndCap) {
        // Points on profile move clockwise around +tangent:
        // (+up) -> (+right) has (+up) x (+right) = +tangent.
        indices.add(centerIdx);
        indices.add(a);
        indices.add(b);
      } else {
        indices.add(centerIdx);
        indices.add(b);
        indices.add(a);
      }
    }
  }
}

/// Incoming road arm specification for [RoadIntersectionMeshBuilder].
final class RoadArm {
  final String id;
  final ExtVec3 direction;
  final double width;
  final double elevationOffset;

  RoadArm({
    required this.id,
    required ExtVec3 direction,
    this.width = 6.0,
    this.elevationOffset = 0.0,
  }) : direction = direction.lengthSquared > 1e-12 ? direction.normalized : ExtVec3.unitZ {
    if (width <= 0) throw ArgumentError.value(width, 'width', 'must be > 0');
  }
}

/// Connection portal produced for each arm of a generated road intersection.
final class RoadPortal {
  final String armId;
  final ExtVec3 center;
  final ExtVec3 leftCorner;
  final ExtVec3 rightCorner;
  final ExtVec3 outwardDirection;
  final double width;

  const RoadPortal({
    required this.armId,
    required this.center,
    required this.leftCorner,
    required this.rightCorner,
    required this.outwardDirection,
    required this.width,
  });
}

/// Combined mesh + per-arm portal metadata from [RoadIntersectionMeshBuilder].
final class RoadIntersectionResult {
  final ExtrudedMeshData mesh;
  final List<RoadPortal> portals;

  const RoadIntersectionResult({
    required this.mesh,
    required this.portals,
  });
}

/// Builds a convex/star N-way junction deck polygon mesh and snaps portals per [RoadArm].
final class RoadIntersectionMeshBuilder {
  const RoadIntersectionMeshBuilder._();

  static RoadIntersectionResult build({
    ExtVec3 center = ExtVec3.zero,
    ExtVec3 up = ExtVec3.unitY,
    required List<RoadArm> arms,
    double hubRadius = 4.0,
    double uvScale = 0.25,
  }) {
    if (arms.length < 2) {
      throw ArgumentError.value(arms.length, 'arms.length', 'must be >= 2');
    }
    if (hubRadius <= 0) {
      throw ArgumentError.value(hubRadius, 'hubRadius', 'must be > 0');
    }

    final u = up.lengthSquared > 1e-12 ? up.normalized : ExtVec3.unitY;
    final refAxis = u.x.abs() < 0.9 ? ExtVec3.unitX : ExtVec3.unitZ;
    final basisX = refAxis.cross(u).normalized;
    final basisZ = u.cross(basisX).normalized;
    // Note: basisZ x basisX == u because (u x X) x X = ... wait:
    // Let's verify: if u = (0,1,0), ref = (1,0,0):
    // basisX = (1,0,0) x (0,1,0) = (0,0,1).
    // basisZ = (0,1,0) x (0,0,1) = (1,0,0).
    // basisX x basisZ = (0,0,1) x (1,0,0) = (0,+1,0) = +u!
    // And descending angle from basisX toward basisZ gives (+Z) x (+X) or (+X) x (+Z) ->
    // Ascending angle around (basisX -> basisZ) has (+X) -> (+Z), and basisX x basisZ = +u!
    // Wait: let's double check!
    // If angle = 0 -> dir = basisX, angle = +pi/2 -> dir = basisZ:
    // (dir_0 - center) x (dir_1 - center) = basisX x basisZ = +u!
    // So ASCENDING angle order with triangles (center, v_curr, v_next) produces +u normal!

    final projected = <_SortedArm>[];
    for (final arm in arms) {
      final flat = arm.direction - u * arm.direction.dot(u);
      final dir = flat.lengthSquared > 1e-12 ? flat.normalized : basisX;
      final angle = math.atan2(dir.dot(basisZ), dir.dot(basisX));
      projected.add(_SortedArm(arm: arm, dir: dir, angle: angle));
    }
    projected.sort((a, b) => a.angle.compareTo(b.angle));

    final perimeterPoints = <ExtVec3>[];
    final portals = <RoadPortal>[];

    for (var i = 0; i < projected.length; i++) {
      final current = projected[i];
      final next = projected[(i + 1) % projected.length];
      final dir = current.dir;
      // Right when facing outward along `dir` with `u` up: dir x u.
      final right = dir.cross(u).normalized;
      final halfW = current.arm.width * 0.5;
      final armDist = math.max(hubRadius, halfW * 1.15);
      final armCenter = center + dir * armDist + u * current.arm.elevationOffset;
      final leftCorner = armCenter - right * halfW;
      final rightCorner = armCenter + right * halfW;

      portals.add(
        RoadPortal(
          armId: current.arm.id,
          center: armCenter,
          leftCorner: leftCorner,
          rightCorner: rightCorner,
          outwardDirection: dir,
          width: current.arm.width,
        ),
      );

      // Facing outward (+dir), left -> right rotates around `u` from +dir toward +right?
      // Wait: check (dir - center) vs angle!
      // At dir = basisX = (0,0,1) with u = (0,1,0):
      // right = dir x u = (0,0,1) x (0,1,0) = (-1, 0, 0) = -basisZ.
      // So rightCorner is at angle NEGATIVE, leftCorner is at angle POSITIVE!
      // That means in ASCENDING angle order around the perimeter, rightCorner comes BEFORE leftCorner!
      perimeterPoints.add(rightCorner);
      perimeterPoints.add(leftCorner);

      // Optional mid-arc fillet vertex between widely separated arms for smooth outer curb.
      var deltaAngle = next.angle - current.angle;
      if (deltaAngle <= 0) deltaAngle += math.pi * 2.0;
      if (deltaAngle > math.pi * 0.65) {
        final midAngle = current.angle + deltaAngle * 0.5;
        final midDir = (basisX * math.cos(midAngle) + basisZ * math.sin(midAngle)).normalized;
        final midElev =
            (current.arm.elevationOffset + next.arm.elevationOffset) * 0.5;
        perimeterPoints.add(center + midDir * hubRadius + u * midElev);
      }
    }

    final posList = <double>[];
    final normList = <double>[];
    final uvList = <double>[];
    final indices = <int>[];

    var minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    var maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;

    int addHubVertex(ExtVec3 p) {
      final idx = posList.length ~/ 3;
      posList.add(p.x);
      posList.add(p.y);
      posList.add(p.z);
      normList.add(u.x);
      normList.add(u.y);
      normList.add(u.z);
      final rel = p - center;
      uvList.add(rel.dot(basisX) * uvScale + 0.5);
      uvList.add(rel.dot(basisZ) * uvScale + 0.5);

      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.z < minZ) minZ = p.z;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
      if (p.z > maxZ) maxZ = p.z;
      return idx;
    }

    final centerIdx = addHubVertex(center);
    final ringIndices = perimeterPoints.map(addHubVertex).toList();
    for (var i = 0; i < ringIndices.length; i++) {
      final curr = ringIndices[i];
      final next = ringIndices[(i + 1) % ringIndices.length];
      indices.add(centerIdx);
      indices.add(curr);
      indices.add(next);
    }

    final mesh = ExtrudedMeshData(
      positions: Float32List.fromList(posList),
      normals: Float32List.fromList(normList),
      uvs: Float32List.fromList(uvList),
      indices: Uint32List.fromList(indices),
      boundsMin: ExtVec3(minX, minY, minZ),
      boundsMax: ExtVec3(maxX, maxY, maxZ),
    );

    return RoadIntersectionResult(mesh: mesh, portals: portals);
  }
}

final class _SortedArm {
  final RoadArm arm;
  final ExtVec3 dir;
  final double angle;

  _SortedArm({required this.arm, required this.dir, required this.angle});
}
