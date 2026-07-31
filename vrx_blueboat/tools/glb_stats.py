#!/usr/bin/env python3
# Copyright 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Report geometry complexity of glTF-binary (.glb) mesh files.

For each .glb it prints the vertex and triangle counts (the independent
variable when assessing the sim's sensitivity to model complexity), plus
mesh/primitive counts, texture presence, Draco compression, and the world-
space bounding-box extents in metres.

Counts are summed over every primitive of every mesh in the file (the unique
geometry). If any mesh is instanced by more than one node, the number of
extra instances is reported so you can reason about the rendered/collided
load, which is what actually costs CPU/GPU at run time.

Usage:
    glb_stats.py MODEL.glb ...
    glb_stats.py path/to/model_dir            # recurses for *.glb
    glb_stats.py --csv meshes/*.glb > stats.csv

Pure standard library - no trimesh/pygltflib needed.
"""

import argparse
import glob
import json
import os
import struct
import sys

# glTF primitive.mode -> how to turn an index/vertex count into a triangle count.
_MODE_TRIANGLES = 4
_MODE_TRIANGLE_STRIP = 5
_MODE_TRIANGLE_FAN = 6

_GLB_MAGIC = 0x46546C67       # 'glTF' little-endian
_CHUNK_JSON = 0x4E4F534A      # 'JSON'


class GlbError(Exception):
    """Raised when a file is not a parseable GLB."""


def _read_json_chunk(path):
    """Return the parsed JSON header of a GLB file."""
    with open(path, 'rb') as f:
        header = f.read(12)
        if len(header) < 12:
            raise GlbError('file shorter than a GLB header')
        magic, version, _length = struct.unpack('<III', header)
        if magic != _GLB_MAGIC:
            raise GlbError('not a binary glTF (bad magic - is this a .gltf '
                           'text file or non-glTF?)')
        if version != 2:
            raise GlbError(f'unsupported glTF version {version} (expected 2)')
        chunk_header = f.read(8)
        if len(chunk_header) < 8:
            raise GlbError('missing JSON chunk')
        clen, ctype = struct.unpack('<II', chunk_header)
        if ctype != _CHUNK_JSON:
            raise GlbError('first chunk is not JSON')
        data = f.read(clen)
    return json.loads(data.decode('utf-8', errors='replace'))


def _triangles(mode, count):
    """Triangles produced by `count` indices/vertices under primitive `mode`."""
    if mode is None:
        mode = _MODE_TRIANGLES
    if mode == _MODE_TRIANGLES:
        return count // 3
    if mode in (_MODE_TRIANGLE_STRIP, _MODE_TRIANGLE_FAN):
        return max(0, count - 2)
    return 0  # points / lines contribute no triangles


def analyze(path):
    """Return a dict of geometry stats for one GLB file."""
    j = _read_json_chunk(path)
    accessors = j.get('accessors', [])
    meshes = j.get('meshes', [])

    # How many nodes reference each mesh (instancing).
    instances = [0] * len(meshes)
    for node in j.get('nodes', []):
        m = node.get('mesh')
        if isinstance(m, int) and 0 <= m < len(meshes):
            instances[m] += 1

    verts = tris = prims = 0
    draco = False
    gmin = [float('inf')] * 3
    gmax = [float('-inf')] * 3
    inst_verts = inst_tris = 0

    for mi, mesh in enumerate(meshes):
        m_verts = m_tris = 0
        for prim in mesh.get('primitives', []):
            prims += 1
            if 'KHR_draco_mesh_compression' in prim.get('extensions', {}):
                draco = True
            mode = prim.get('mode')
            pos_idx = prim.get('attributes', {}).get('POSITION')
            n_pos = 0
            if isinstance(pos_idx, int) and pos_idx < len(accessors):
                acc = accessors[pos_idx]
                n_pos = acc.get('count', 0)
                if 'min' in acc and 'max' in acc and len(acc['min']) == 3:
                    for k in range(3):
                        gmin[k] = min(gmin[k], acc['min'][k])
                        gmax[k] = max(gmax[k], acc['max'][k])
            m_verts += n_pos
            idx = prim.get('indices')
            if isinstance(idx, int) and idx < len(accessors):
                m_tris += _triangles(mode, accessors[idx].get('count', 0))
            else:
                m_tris += _triangles(mode, n_pos)
        verts += m_verts
        tris += m_tris
        # Instanced totals: count each mesh once even if it appears in no node.
        mult = instances[mi] if instances[mi] > 0 else 1
        inst_verts += m_verts * mult
        inst_tris += m_tris * mult

    extents = [round(gmax[k] - gmin[k], 4) for k in range(3)] \
        if gmax[0] > float('-inf') else [0.0, 0.0, 0.0]

    images = j.get('images', [])
    embedded = sum(1 for im in images if 'bufferView' in im)
    external = sum(1 for im in images if 'uri' in im)

    return {
        'file': path,
        'bytes': os.path.getsize(path),
        'meshes': len(meshes),
        'primitives': prims,
        'vertices': verts,
        'triangles': tris,
        'inst_vertices': inst_verts,
        'inst_triangles': inst_tris,
        'instanced': inst_tris != tris,
        'materials': len(j.get('materials', [])),
        'images': len(images),
        'tex_embedded': embedded,
        'tex_external': external,
        'draco': draco,
        'extents': extents,
    }


def _expand(paths):
    """Turn file/dir/glob arguments into a sorted list of .glb files."""
    out = []
    for p in paths:
        if os.path.isdir(p):
            out.extend(sorted(glob.glob(os.path.join(p, '**', '*.glb'),
                                        recursive=True)))
        elif any(c in p for c in '*?['):
            out.extend(sorted(glob.glob(p, recursive=True)))
        else:
            out.append(p)
    return out


def _fmt(n):
    return f'{n:,}'


def _print_table(rows):
    hdr = ('FILE', 'VERTS', 'TRIS', 'MSH', 'PRIM', 'TEX', 'DRACO',
           'EXTENTS (x y z, m)')
    name_w = max([len(os.path.basename(r['file'])) for r in rows]
                 + [len(hdr[0])])
    fmt = (f'{{:<{name_w}}}  {{:>10}}  {{:>10}}  {{:>3}}  {{:>4}}  '
           f'{{:>7}}  {{:>5}}  {{}}')
    print(fmt.format(*hdr))
    print('-' * (name_w + 60))
    tot_v = tot_t = tot_iv = tot_it = 0
    any_inst = False
    for r in rows:
        tex = f"{r['tex_embedded']}e/{r['tex_external']}x" if r['images'] \
            else '-'
        ext = ' '.join(f'{v:g}' for v in r['extents'])
        print(fmt.format(
            os.path.basename(r['file']),
            _fmt(r['vertices']), _fmt(r['triangles']),
            r['meshes'], r['primitives'], tex,
            'yes' if r['draco'] else '-', ext))
        tot_v += r['vertices']
        tot_t += r['triangles']
        tot_iv += r['inst_vertices']
        tot_it += r['inst_triangles']
        any_inst = any_inst or r['instanced']
    if len(rows) > 1:
        print('-' * (name_w + 60))
        print(fmt.format('TOTAL', _fmt(tot_v), _fmt(tot_t),
                         '', '', '', '', ''))
    if any_inst:
        print()
        print(f'note: instancing present - rendered/collided totals are '
              f'{_fmt(tot_iv)} verts / {_fmt(tot_it)} tris '
              f'(unique geometry shown above).')


def _print_csv(rows):
    import csv
    cols = ['file', 'bytes', 'meshes', 'primitives', 'vertices', 'triangles',
            'inst_vertices', 'inst_triangles', 'materials', 'images',
            'tex_embedded', 'tex_external', 'draco',
            'extent_x', 'extent_y', 'extent_z']
    w = csv.writer(sys.stdout)
    w.writerow(cols)
    for r in rows:
        w.writerow([
            r['file'], r['bytes'], r['meshes'], r['primitives'],
            r['vertices'], r['triangles'], r['inst_vertices'],
            r['inst_triangles'], r['materials'], r['images'],
            r['tex_embedded'], r['tex_external'], int(r['draco']),
            r['extents'][0], r['extents'][1], r['extents'][2]])


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Report vertex/triangle complexity of .glb mesh files.')
    ap.add_argument('paths', nargs='+',
                    help='.glb files, directories (recursed), or globs')
    ap.add_argument('--csv', action='store_true',
                    help='emit CSV instead of a table')
    args = ap.parse_args(argv)

    files = _expand(args.paths)
    if not files:
        print('no .glb files found', file=sys.stderr)
        return 1

    rows = []
    errors = 0
    for path in files:
        try:
            rows.append(analyze(path))
        except (OSError, GlbError, ValueError) as e:
            print(f'skip {path}: {e}', file=sys.stderr)
            errors += 1

    if not rows:
        return 1
    if args.csv:
        _print_csv(rows)
    else:
        _print_table(rows)
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
