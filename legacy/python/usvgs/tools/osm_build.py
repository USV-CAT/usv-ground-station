#!/usr/bin/env python3
"""
Build a tiled binary vector map of the WHOLE country from a big .osm.pbf, once.
The GUI then lazily loads only the tiles in view, so all of Latvia is navigable.

  python -m usvgs.tools.osm_build --pbf latvia-latest.osm.pbf --out maps/latvia_map
  python -m usvgs.tools.osm_build --pbf ... --buildings        # also include buildings (more RAM)

Output (default prefix 'latvia_map'):
  latvia_map.idx.json   tile index  {tile,bbox,layers,tiles:{"ix_iy":{layer:[off,len]}}}
  latvia_map.bin        concatenated tile blocks
Polyline block = repeated [ uint16 npts | npts x (int32 lat_e7, int32 lon_e7) ].
"""
import argparse, json, os, sys, time, struct, array
from math import floor as math_floor
from . import osm_pbf

LAYERS = ["water", "landuse", "rail", "road_minor", "road_major", "building"]
LAYER_IDX = {n: i for i, n in enumerate(LAYERS)}

def classify(tags):
    hw = tags.get("highway")
    if hw in ("motorway", "trunk", "primary", "secondary", "tertiary",
              "motorway_link", "trunk_link", "primary_link"):
        return "road_major"
    if hw in ("residential", "unclassified", "service", "living_street", "road",
              "track", "pedestrian", "footway", "path", "cycleway", "steps"):
        return "road_minor"
    if tags.get("railway") in ("rail", "light_rail", "subway", "tram", "narrow_gauge"):
        return "rail"
    if (tags.get("natural") in ("water", "coastline") or
            tags.get("waterway") in ("river", "riverbank", "stream", "canal", "drain") or
            tags.get("water") or tags.get("landuse") == "reservoir"):
        return "water"
    if "building" in tags:
        return "building"
    if (tags.get("landuse") in ("forest", "grass", "meadow", "farmland", "residential",
                                "industrial", "cemetery", "orchard", "vineyard") or
            tags.get("natural") in ("wood", "scrub", "grassland", "heath", "wetland") or
            tags.get("leisure") in ("park", "garden", "pitch", "golf_course")):
        return "landuse"
    return None


def build(pbf, out_prefix, tile=0.05, with_buildings=False):
    t0 = time.time()
    # ---- pass 1: ways (skip node coords -> fast) ----
    print("[*] pass 1/2: ways…", file=sys.stderr)
    ways = []                      # (layer_idx, array('q', refs))
    needed = set()
    for item in osm_pbf.iter_primitives(pbf, want_ways=True, want_nodes=False):
        if item[0] != "way":
            continue
        _, wid, refs, tags = item
        layer = classify(tags)
        if not layer or (layer == "building" and not with_buildings):
            continue
        if len(refs) < 2:
            continue
        ways.append((LAYER_IDX[layer], array.array("q", refs)))
        needed.update(refs)
    print(f"    {len(ways):,} ways, {len(needed):,} needed nodes, {time.time()-t0:.0f}s", file=sys.stderr)

    # ---- pass 2: coords for needed nodes ----
    print("[*] pass 2/2: node coords…", file=sys.stderr)
    coords = {}
    for item in osm_pbf.iter_primitives(pbf, want_ways=False, want_nodes=True):
        if item[0] != "node":
            continue
        _, nid, la, lo, _ = item
        if nid in needed:
            coords[nid] = (int(round(la * 1e7)), int(round(lo * 1e7)))
    needed = None
    print(f"    resolved {len(coords):,} coords, {time.time()-t0:.0f}s", file=sys.stderr)

    # ---- assemble into tiles ----
    print("[*] assembling tiles…", file=sys.stderr)
    tiles = {}                     # (ix,iy) -> {layer_idx: bytearray}
    minla = minlo = 1e18; maxla = maxlo = -1e18
    inv = 1.0 / tile
    def emit(layer_idx, pts):
        nonlocal minla, minlo, maxla, maxlo
        if len(pts) < 2:
            return
        blob = struct.pack("<H", len(pts)) + b"".join(struct.pack("<ii", a, b) for a, b in pts)
        seen = set()
        for a, b in pts:
            seen.add((int(math_floor(a * 1e-7 * inv)), int(math_floor(b * 1e-7 * inv))))
            if a < minla: minla = a
            if a > maxla: maxla = a
            if b < minlo: minlo = b
            if b > maxlo: maxlo = b
        for tk in seen:
            d = tiles.setdefault(tk, {})
            d.setdefault(layer_idx, bytearray()).extend(blob)

    for layer_idx, refs in ways:
        line = []
        for r in refs:
            c = coords.get(r)
            if c:
                line.append(c)
            elif len(line) >= 2:
                emit(layer_idx, line); line = []
            else:
                line = []
        if len(line) >= 2:
            emit(layer_idx, line)
    ways = None; coords = None

    # ---- write bin + index ----
    d = os.path.dirname(out_prefix)
    if d: os.makedirs(d, exist_ok=True)
    binp = out_prefix + ".bin"; idxp = out_prefix + ".idx.json"
    index = {"version": 1, "tile": tile,
             "bbox": [minla * 1e-7, minlo * 1e-7, maxla * 1e-7, maxlo * 1e-7],
             "layers": LAYERS, "bin": os.path.basename(binp), "tiles": {}}
    off = 0
    with open(binp, "wb") as bf:
        for (ix, iy), layers in tiles.items():
            rec = {}
            for layer_idx, buf in layers.items():
                bf.write(buf)
                rec[LAYERS[layer_idx]] = [off, len(buf)]
                off += len(buf)
            index["tiles"][f"{ix}_{iy}"] = rec
    json.dump(index, open(idxp, "w"))
    print(f"[*] wrote {idxp} ({len(index['tiles'])} tiles) and {binp} "
          f"({off//1024//1024} MB) in {time.time()-t0:.0f}s", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="Build tiled country map from .osm.pbf")
    ap.add_argument("--pbf", required=True)
    ap.add_argument("--out", default="latvia_map")
    ap.add_argument("--tile", type=float, default=0.05)
    ap.add_argument("--buildings", action="store_true")
    args = ap.parse_args()
    if not os.path.exists(args.pbf):
        sys.exit(f"!! pbf not found: {args.pbf}")
    build(args.pbf, args.out, args.tile, args.buildings)


if __name__ == "__main__":
    main()
