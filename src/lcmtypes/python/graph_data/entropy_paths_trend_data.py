"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import graph_data.entropy_path_trend_data

class entropy_paths_trend_data(object):
    __slots__ = ["num_path_", "entropy_paths_"]

    __typenames__ = ["int32_t", "graph_data.entropy_path_trend_data"]

    __dimensions__ = [None, ["num_path_"]]

    def __init__(self):
        self.num_path_ = 0
        self.entropy_paths_ = []

    def encode(self):
        buf = BytesIO()
        buf.write(entropy_paths_trend_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_path_))
        for i0 in range(self.num_path_):
            assert self.entropy_paths_[i0]._get_packed_fingerprint() == graph_data.entropy_path_trend_data._get_packed_fingerprint()
            self.entropy_paths_[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != entropy_paths_trend_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return entropy_paths_trend_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = entropy_paths_trend_data()
        self.num_path_ = struct.unpack(">i", buf.read(4))[0]
        self.entropy_paths_ = []
        for i0 in range(self.num_path_):
            self.entropy_paths_.append(graph_data.entropy_path_trend_data._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if entropy_paths_trend_data in parents: return 0
        newparents = parents + [entropy_paths_trend_data]
        tmphash = (0xf1223a9b5bea0642+ graph_data.entropy_path_trend_data._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if entropy_paths_trend_data._packed_fingerprint is None:
            entropy_paths_trend_data._packed_fingerprint = struct.pack(">Q", entropy_paths_trend_data._get_hash_recursive([]))
        return entropy_paths_trend_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
