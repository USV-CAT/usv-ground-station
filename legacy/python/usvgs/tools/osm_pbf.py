#!/usr/bin/env python3
"""
Minimal, dependency-free OSM PBF reader (pure Python: struct + zlib only).

Enough of the format to stream nodes and ways out of a .osm.pbf so we can build
an offline vector map for the ground station. Not a general OSM library.

PBF layout:  repeated [ uint32 BE header-len | BlobHeader | Blob ]
  BlobHeader {1:type(str) 3:datasize(int32)}
  Blob       {1:raw 2:raw_size 3:zlib_data}   -> PrimitiveBlock (zlib)
  PrimitiveBlock {1:StringTable 2:PrimitiveGroup* 17:granularity 19:lat_off 20:lon_off}
  PrimitiveGroup {2:DenseNodes 3:Way*}
  DenseNodes {1:id(sint64,delta,packed) 8:lat 9:lon (delta,packed) 10:keys_vals}
  Way {1:id 2:keys(packed) 3:vals(packed) 8:refs(sint64,delta,packed)}
coord(deg) = 1e-9 * (offset + granularity * value)
"""
import struct, zlib


# ---------------- protobuf wire primitives ----------------
def _varint(b, p):
    shift = 0; result = 0
    while True:
        c = b[p]; p += 1
        result |= (c & 0x7F) << shift
        if not (c & 0x80):
            return result, p
        shift += 7

def _zigzag(n):
    return (n >> 1) ^ -(n & 1)

def _packed_varints(b):
    p = 0; n = len(b); out = []
    ap = out.append
    while p < n:
        c = b[p]; p += 1
        if c < 0x80:
            ap(c); continue
        result = c & 0x7F; shift = 7
        while True:
            c = b[p]; p += 1
            result |= (c & 0x7F) << shift
            if not (c & 0x80):
                ap(result); break
            shift += 7
    return out

def _fields(b):
    """Iterate (field_number, wire_type, value) over a protobuf message.
       value is int (varint/fixed) or memoryview (length-delimited)."""
    p = 0; n = len(b)
    while p < n:
        tag, p = _varint(b, p)
        fn = tag >> 3; wt = tag & 7
        if wt == 0:
            v, p = _varint(b, p); yield fn, wt, v
        elif wt == 2:
            ln, p = _varint(b, p); yield fn, wt, b[p:p+ln]; p += ln
        elif wt == 1:
            yield fn, wt, b[p:p+8]; p += 8
        elif wt == 5:
            yield fn, wt, b[p:p+4]; p += 4
        else:
            raise ValueError(f"bad wire type {wt}")


# ---------------- blocks ----------------
def blocks(path):
    """Yield decompressed PrimitiveBlock bytes (skips the header block)."""
    with open(path, "rb") as f:
        while True:
            lenb = f.read(4)
            if len(lenb) < 4:
                return
            (hlen,) = struct.unpack(">I", lenb)
            header = f.read(hlen)
            btype = None; dsize = 0
            for fn, wt, v in _fields(header):
                if fn == 1: btype = bytes(v).decode()
                elif fn == 3: dsize = v
            blob = f.read(dsize)
            raw = None
            for fn, wt, v in _fields(blob):
                if fn == 1: raw = bytes(v)                 # uncompressed
                elif fn == 3: raw = zlib.decompress(bytes(v))
            if btype == "OSMData" and raw is not None:
                yield raw


def iter_primitives(path, want_ways=True, want_nodes=True):
    """Yield ('node', id, lat, lon, tags) and ('way', id, [refs], tags).
       tags is a dict[str,str]. Coordinates in degrees."""
    for blk in blocks(path):
        strings = []; groups = []
        granularity = 100; lat_off = 0; lon_off = 0
        for fn, wt, v in _fields(blk):
            if fn == 1:            # StringTable
                strings = [bytes(s).decode("utf-8", "replace")
                           for sfn, swt, s in _fields(v) if sfn == 1]
            elif fn == 2:          # PrimitiveGroup
                groups.append(v)
            elif fn == 17: granularity = v
            elif fn == 19: lat_off = v
            elif fn == 20: lon_off = v
        gran = granularity
        for g in groups:
            for fn, wt, v in _fields(g):
                if fn == 2 and want_nodes:      # DenseNodes
                    ids = []; lats = []; lons = []; kv = []
                    for dfn, dwt, dv in _fields(v):
                        if dfn == 1: ids = _packed_varints(dv)
                        elif dfn == 8: lats = _packed_varints(dv)
                        elif dfn == 9: lons = _packed_varints(dv)
                        elif dfn == 10: kv = _packed_varints(dv)
                    _id = _lat = _lon = 0; ki = 0
                    for i in range(len(ids)):
                        _id += _zigzag(ids[i])
                        _lat += _zigzag(lats[i])
                        _lon += _zigzag(lons[i])
                        tags = {}
                        if kv:
                            while ki < len(kv) and kv[ki] != 0:
                                k = strings[kv[ki]]; val = strings[kv[ki+1]]
                                tags[k] = val; ki += 2
                            ki += 1     # skip the 0 separator
                        yield ("node", _id,
                               1e-9 * (lat_off + gran * _lat),
                               1e-9 * (lon_off + gran * _lon), tags)
                elif fn == 3 and want_ways:     # Way
                    wid = 0; keys = []; vals = []; refs = []
                    for wfn, wwt, wv in _fields(v):
                        if wfn == 1: wid = wv
                        elif wfn == 2: keys = _packed_varints(wv)
                        elif wfn == 3: vals = _packed_varints(wv)
                        elif wfn == 8:
                            r = 0
                            for d in _packed_varints(wv):
                                r += _zigzag(d); refs.append(r)
                    tags = {strings[k]: strings[val] for k, val in zip(keys, vals)}
                    yield ("way", wid, refs, tags)


if __name__ == "__main__":
    import sys, collections, time
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "/home/ivanmars/Downloads/latvia-260725.osm.pbf"
    t0 = time.time(); nn = nw = 0
    minlat = minlon = 1e9; maxlat = maxlon = -1e9
    hw = collections.Counter()
    for item in iter_primitives(path):
        if item[0] == "node":
            nn += 1
            _, _, la, lo, _ = item
            minlat = min(minlat, la); maxlat = max(maxlat, la)
            minlon = min(minlon, lo); maxlon = max(maxlon, lo)
        else:
            nw += 1
            hw[item[3].get("highway", "")] += 1
        if nn + nw >= 2_000_000:
            break
    print(f"scanned {nn} nodes, {nw} ways in {time.time()-t0:.1f}s")
    print(f"bbox lat[{minlat:.4f},{maxlat:.4f}] lon[{minlon:.4f},{maxlon:.4f}]")
    print("top highway tags:", hw.most_common(6))
