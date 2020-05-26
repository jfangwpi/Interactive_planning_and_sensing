"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import distribution_data.sample

class path_dist(object):
    __slots__ = ["num_sample", "cost_dist"]

    __typenames__ = ["int32_t", "distribution_data.sample"]

    __dimensions__ = [None, ["num_sample"]]

    def __init__(self):
        self.num_sample = 0
        self.cost_dist = []

    def encode(self):
        buf = BytesIO()
        buf.write(path_dist._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_sample))
        for i0 in range(self.num_sample):
            assert self.cost_dist[i0]._get_packed_fingerprint() == distribution_data.sample._get_packed_fingerprint()
            self.cost_dist[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != path_dist._get_packed_fingerprint():
            raise ValueError("Decode error")
        return path_dist._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = path_dist()
        self.num_sample = struct.unpack(">i", buf.read(4))[0]
        self.cost_dist = []
        for i0 in range(self.num_sample):
            self.cost_dist.append(distribution_data.sample._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if path_dist in parents: return 0
        newparents = parents + [path_dist]
        tmphash = (0x29238fd62ba89e85+ distribution_data.sample._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if path_dist._packed_fingerprint is None:
            path_dist._packed_fingerprint = struct.pack(">Q", path_dist._get_hash_recursive([]))
        return path_dist._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
