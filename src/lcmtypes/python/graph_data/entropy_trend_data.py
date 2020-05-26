"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class entropy_trend_data(object):
    __slots__ = ["num_iteration_", "entropy_"]

    __typenames__ = ["int32_t", "double"]

    __dimensions__ = [None, ["num_iteration_"]]

    def __init__(self):
        self.num_iteration_ = 0
        self.entropy_ = []

    def encode(self):
        buf = BytesIO()
        buf.write(entropy_trend_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_iteration_))
        buf.write(struct.pack('>%dd' % self.num_iteration_, *self.entropy_[:self.num_iteration_]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != entropy_trend_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return entropy_trend_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = entropy_trend_data()
        self.num_iteration_ = struct.unpack(">i", buf.read(4))[0]
        self.entropy_ = struct.unpack('>%dd' % self.num_iteration_, buf.read(self.num_iteration_ * 8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if entropy_trend_data in parents: return 0
        tmphash = (0x67a634745e24e10f) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if entropy_trend_data._packed_fingerprint is None:
            entropy_trend_data._packed_fingerprint = struct.pack(">Q", entropy_trend_data._get_hash_recursive([]))
        return entropy_trend_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
