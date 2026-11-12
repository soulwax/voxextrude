<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxextrude logo"/>
</p>

# voxextrude

Pure Dart procedural 3D spline sweep, road/river ribbon generator, and intersection mesh builder.

## Features

- **Twist-Free Bishop / Rotation-Minimizing Frames (`RotationMinimizingFrame`)**: Smoothly sweeps frames along 3D spline tangents using double-reflection parallel transport without Frenet gimbal flips, or horizon-locked mode for roads/bridges.
- **Customizable Cross-Sections (`CrossSectionProfile`)**: Presets for cambered curbed roads, river ribbons, circular pipes/rails, and box girders.
- **Arc-Length UVs & End Caps (`SplineSweepBuilder`)**: Clean triangle meshes with normals, seams, end caps for closed tubes, and scale/roll profiles.
- **N-Way Road Junctions (`RoadIntersectionMeshBuilder`)**: Radially sorted junction hub polygons + connection portal edges.
