import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:voxextrude/voxextrude.dart';

void main() {
  group('RotationMinimizingFrame & ExtrusionResampler', () {
    test('double-reflection Bishop frame avoids Frenet flip across vertical S-bend', () {
      // Sweeps forward +Z, curves vertically +Y, bends back over -Z without gimbal flip.
      final points = <ExtVec3>[];
      const count = 32;
      for (var i = 0; i < count; i++) {
        final t = i / (count - 1);
        final angle = t * math.pi;
        points.add(ExtVec3(
          t * 1.5,
          1.0 - math.cos(angle) * 2.0,
          math.sin(angle) * 3.0,
        ));
      }
      final samples = ExtrusionResampler.fromPolyline(points);
      final frames = RotationMinimizingFrame.computeFrames(samples);
      expect(frames.length, count);

      // Adjacent Bishop right vectors must vary smoothly (dot > 0.8) across the entire arc.
      for (var i = 1; i < frames.length; i++) {
        final prev = frames[i - 1];
        final curr = frames[i];
        expect(curr.right.dot(curr.tangent).abs(), lessThan(1e-6));
        expect(curr.up.dot(curr.tangent).abs(), lessThan(1e-6));
        expect(curr.right.dot(prev.right), greaterThan(0.8));
      }
    });

    test('upLocked mode keeps road cross-sections horizon-leveled over hills', () {
      final samples = ExtrusionResampler.fromCatmullRom(
        const [
          ExtVec3(0.0, 0.0, 0.0),
          ExtVec3(2.0, 3.0, 5.0),
          ExtVec3(5.0, 1.0, 10.0),
          ExtVec3(8.0, 4.0, 15.0),
        ],
        subdivisionsPerSegment: 8,
      );
      final frames = RotationMinimizingFrame.computeFrames(samples, upLocked: true);
      for (final f in frames) {
        expect(f.right.y.abs(), lessThan(1e-6));
        expect(f.up.y, greaterThan(0.0));
      }
    });
  });

  group('SplineSweepBuilder', () {
    test('builds seamless capped pipe mesh with outward winding and arc-length UVs', () {
      final samples = ExtrusionResampler.fromPolyline(const [
        ExtVec3(0.0, 0.0, 0.0),
        ExtVec3(0.0, 0.0, 4.0),
        ExtVec3(3.0, 0.0, 8.0),
      ]);
      final profile = CrossSectionProfile.circle(radius: 0.5, segments: 8);
      final mesh = SplineSweepBuilder.build(
        samples: samples,
        profile: profile,
        capEnds: true,
        uvScaleV: 0.5,
      );

      expect(mesh.totalArcLength, greaterThan(8.5));
      expect(mesh.vertexCount, greaterThan(27));
      expect(mesh.triangleCount, greaterThan(32));

      // Verify start and end cap triangles match outward tangent normals.
      for (var t = 0; t < mesh.triangleCount; t++) {
        final i0 = mesh.indices[t * 3 + 0];
        final i1 = mesh.indices[t * 3 + 1];
        final i2 = mesh.indices[t * 3 + 2];
        final p0 = mesh.positionAt(i0);
        final p1 = mesh.positionAt(i1);
        final p2 = mesh.positionAt(i2);
        final triNormal = (p1 - p0).cross(p2 - p0).normalized;
        final avgStored =
            (mesh.normalAt(i0) + mesh.normalAt(i1) + mesh.normalAt(i2)).normalized;
        expect(triNormal.dot(avgStored), greaterThan(0.75));
      }

      // First ring V == 0, last swept ring V == totalArcLength * uvScaleV.
      expect(mesh.uvAt(0).y, closeTo(0.0, 1e-6));
      final lastRingVertex = 2 * profile.points.length;
      expect(mesh.uvAt(lastRingVertex).y, closeTo(mesh.totalArcLength * 0.5, 1e-5));
    });

    test('builds open cambered road ribbon with upward winding', () {
      final samples = ExtrusionResampler.fromPolyline(const [
        ExtVec3(0.0, 0.0, 0.0),
        ExtVec3(0.0, 0.0, -5.0),
        ExtVec3(4.0, 0.5, -10.0),
      ]);
      final road = CrossSectionProfile.road(width: 6.0);
      final mesh = SplineSweepBuilder.build(
        samples: samples,
        profile: road,
        upLocked: true,
      );

      expect(mesh.vertexCount, 3 * road.points.length);
      var upwardFaceCount = 0;
      for (var t = 0; t < mesh.triangleCount; t++) {
        final i0 = mesh.indices[t * 3 + 0];
        final i1 = mesh.indices[t * 3 + 1];
        final i2 = mesh.indices[t * 3 + 2];
        final p0 = mesh.positionAt(i0);
        final p1 = mesh.positionAt(i1);
        final p2 = mesh.positionAt(i2);
        final triNormal = (p1 - p0).cross(p2 - p0).normalized;
        final avgStored =
            (mesh.normalAt(i0) + mesh.normalAt(i1) + mesh.normalAt(i2)).normalized;
        expect(triNormal.y, greaterThanOrEqualTo(-1e-4));
        expect(triNormal.dot(avgStored), greaterThan(0.75));
        if (triNormal.y > 0.7) upwardFaceCount++;
      }
      expect(upwardFaceCount, greaterThan(mesh.triangleCount ~/ 2));
    });

    test('closes spline loop seamlessly when closedLoop is enabled', () {
      const n = 12;
      final pts = List<ExtVec3>.generate(n, (i) {
        final a = (i / n) * math.pi * 2.0;
        return ExtVec3(math.cos(a) * 4.0, 0.0, math.sin(a) * 4.0);
      });
      final samples = ExtrusionResampler.fromPolyline(pts, closed: true);
      final mesh = SplineSweepBuilder.build(
        samples: samples,
        profile: CrossSectionProfile.boxBeam(width: 0.6, height: 0.4),
        closedLoop: true,
        capEnds: false,
      );
      expect(mesh.totalArcLength, greaterThan(20.0));
      expect(mesh.triangleCount, 12 * 4 * 2);
      for (var t = 0; t < mesh.triangleCount; t++) {
        final p0 = mesh.positionAt(mesh.indices[t * 3 + 0]);
        final p1 = mesh.positionAt(mesh.indices[t * 3 + 1]);
        final p2 = mesh.positionAt(mesh.indices[t * 3 + 2]);
        expect((p1 - p0).cross(p2 - p0).length, greaterThan(1e-4));
      }
    });
  });

  group('RoadIntersectionMeshBuilder', () {
    test('constructs 4-way junction hub with upward triangles and aligned portals', () {
      final result = RoadIntersectionMeshBuilder.build(
        center: const ExtVec3(10.0, 0.0, -5.0),
        hubRadius: 5.0,
        arms: [
          RoadArm(id: 'north', direction: const ExtVec3(0.0, 0.0, -1.0), width: 6.0),
          RoadArm(id: 'east', direction: const ExtVec3(1.0, 0.0, 0.0), width: 6.0),
          RoadArm(id: 'south', direction: const ExtVec3(0.0, 0.0, 1.0), width: 6.0),
          RoadArm(id: 'west', direction: const ExtVec3(-1.0, 0.0, 0.0), width: 4.0),
        ],
      );

      expect(result.portals.length, 4);
      final west = result.portals.firstWhere((p) => p.armId == 'west');
      expect(west.width, 4.0);
      expect(west.leftCorner.distanceTo(west.rightCorner), closeTo(4.0, 1e-4));

      for (var t = 0; t < result.mesh.triangleCount; t++) {
        final p0 = result.mesh.positionAt(result.mesh.indices[t * 3 + 0]);
        final p1 = result.mesh.positionAt(result.mesh.indices[t * 3 + 1]);
        final p2 = result.mesh.positionAt(result.mesh.indices[t * 3 + 2]);
        final triNormal = (p1 - p0).cross(p2 - p0).normalized;
        expect(triNormal.y, greaterThan(0.99));
      }
    });
  });
}
