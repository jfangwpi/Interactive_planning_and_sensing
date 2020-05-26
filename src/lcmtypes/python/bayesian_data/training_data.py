"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import bayesian_data.training_pair

class training_data(object):
    __slots__ = ["num_training_", "training_vertex_"]

    __typenames__ = ["int32_t", "bayesian_data.training_pair"]

    __dimensions__ = [None, ["num_training_"]]

    def __init__(self):
        self.num_training_ = 0
        self.training_vertex_ = []

    def encode(self):
        buf = BytesIO()
        buf.write(training_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_training_))
        for i0 in range(self.num_training_):
            assert self.training_vertex_[i0]._get_packed_fingerprint() == bayesian_data.training_pair._get_packed_fingerprint()
            self.training_vertex_[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != training_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return training_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = training_data()
        self.num_training_ = struct.unpack(">i", buf.read(4))[0]
        self.training_vertex_ = []
        for i0 in range(self.num_training_):
            self.training_vertex_.append(bayesian_data.training_pair._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if training_data in parents: return 0
        newparents = parents + [training_data]
        tmphash = (0xd2783642ff2f952a+ bayesian_data.training_pair._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if training_data._packed_fingerprint is None:
            training_data._packed_fingerprint = struct.pack(">Q", training_data._get_hash_recursive([]))
        return training_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
