#!/usr/bin/env python3
# Copyright (C) 2026 Honu Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""
Generate a dense procedural water-surface mesh as a COLLADA file.

The original `water.dae` shipped with VRX is an LOD-style mesh whose finest
vertex spacing is ~18.75m. With the FFT wavefield producing detail down to
a fraction of a metre (cell_count=256, tile_size=100), the renderer's
triangle interpolation collapses most of the wave shape into faceted
polygons. This script emits a uniformly-tessellated plane whose vertex
spacing is fine enough to resolve the FFT field.

Usage:
    python3 generate_water_mesh.py [--size 200] [--segments 400] [--out water.dae]

The defaults produce a 200m x 200m mesh with 0.5m vertex spacing (~160k
vertices, ~320k triangles).
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path
import sys


def generate_collada(size: float, segments: int) -> str:
    """
    Build a COLLADA 1.4.1 document for a uniformly-tessellated XY plane.

    The mesh lives in the +X/+Y/+Z = up convention so it drops straight
    into the existing `<up_axis>Z_UP</up_axis>` water surface model.

    Vertex layout: a (segments+1) x (segments+1) grid of positions, with
    matching UV0 coordinates spanning [0, 1] across the mesh. Triangles
    are emitted as two per quad, indexing the position array directly.
    """
    n = segments + 1
    half = size * 0.5
    step = size / float(segments)

    # Position + UV arrays.
    positions = []
    uvs = []
    for j in range(n):
        y = -half + j * step
        v = j / float(segments)
        for i in range(n):
            x = -half + i * step
            u = i / float(segments)
            positions.extend([x, y, 0.0])
            uvs.extend([u, v])

    # Triangle indices, two per quad. Each index triplet here is
    # (position_index, normal_index, uv_index) but we share one normal
    # for the whole mesh (Z up) and use position==uv index mapping.
    triangles = []
    for j in range(segments):
        for i in range(segments):
            a = j * n + i
            b = a + 1
            c = a + n
            d = c + 1
            # Two CCW triangles per quad: (a, b, d) and (a, d, c)
            triangles.extend([a, 0, a, b, 0, b, d, 0, d])
            triangles.extend([a, 0, a, d, 0, d, c, 0, c])

    pos_count = len(positions)
    uv_count = len(uvs)
    tri_count = len(triangles) // 9

    now = datetime.datetime.now(datetime.UTC).strftime('%Y-%m-%dT%H:%M:%SZ')

    pos_text = ' '.join(f'{v:.6f}' for v in positions)
    uv_text = ' '.join(f'{v:.6f}' for v in uvs)
    tri_text = ' '.join(str(i) for i in triangles)

    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>generate_water_mesh.py</authoring_tool>
      <comments>Uniform plane, size={size}m, segments={segments}</comments>
    </contributor>
    <created>{now}</created>
    <modified>{now}</modified>
    <unit meter="1.000000" name="metre"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="WaterPlane-lib" name="WaterPlaneMesh">
      <mesh>
        <source id="WaterPlane-POSITION">
          <float_array id="WaterPlane-POSITION-array" count="{pos_count}">{pos_text}</float_array>
          <technique_common>
            <accessor source="#WaterPlane-POSITION-array" count="{pos_count // 3}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="WaterPlane-Normal0">
          <float_array id="WaterPlane-Normal0-array" count="3">
            0.000000 0.000000 1.000000</float_array>
          <technique_common>
            <accessor source="#WaterPlane-Normal0-array" count="1" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="WaterPlane-UV0">
          <float_array id="WaterPlane-UV0-array" count="{uv_count}">{uv_text}</float_array>
          <technique_common>
            <accessor source="#WaterPlane-UV0-array" count="{uv_count // 2}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="WaterPlane-VERTEX">
          <input semantic="POSITION" source="#WaterPlane-POSITION"/>
        </vertices>
        <triangles count="{tri_count}">
          <input semantic="VERTEX" source="#WaterPlane-VERTEX" offset="0"/>
          <input semantic="NORMAL" source="#WaterPlane-Normal0" offset="1"/>
          <input semantic="TEXCOORD" source="#WaterPlane-UV0" offset="2" set="0"/>
          <p>{tri_text}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="WaterPlane" name="WaterPlane" type="NODE">
        <instance_geometry url="#WaterPlane-lib"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--size', type=float, default=200.0,
                        help='Side length of the plane in metres (default: 200)')
    parser.add_argument('--segments', type=int, default=400,
                        help='Subdivisions per axis (default: 400 → 0.5m spacing)')
    parser.add_argument('--out', type=Path, default=Path('water.dae'),
                        help='Output path (default: ./water.dae)')
    args = parser.parse_args()

    if args.segments < 1:
        print('--segments must be >= 1', file=sys.stderr)
        return 2

    text = generate_collada(args.size, args.segments)
    args.out.write_text(text, encoding='utf-8')
    n_verts = (args.segments + 1) ** 2
    n_tris = 2 * args.segments * args.segments
    print(f'Wrote {args.out} ({n_verts} verts, {n_tris} tris, '
          f'{args.size}m × {args.size}m at {args.size / args.segments:.3f}m spacing)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
