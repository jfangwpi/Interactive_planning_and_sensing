"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class probability_data(object):
    __slots__ = ["num_probs_", "probs_"]

    __typenames__ = ["int32_t", "double"]

    __dimensions__ = [None, ["num_probs_"]]

    def __init__(self):
        self.num_probs_ = 0
        self.probs_ = []

    def encode(self):
        buf = BytesIO()
        buf.write(probability_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_probs_))
        buf.write(struct.pack('>%dd' % self.num_probs_, *self.probs_[:self.num_probs_]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != probability_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return probability_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = probability_data()
        self.num_probs_ = struct.unpack(">i", buf.read(4))[0]
        self.probs_ = struct.unpack('>%dd' % self.num_probs_, buf.read(self.num_probs_ * 8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if probability_data in parents: return 0
        tmphash = (0x5efcf92981c8c738) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if probability_data._packed_fingerprint is None:
            probability_data._packed_fingerprint = struct.pack(">Q", probability_data._get_hash_recursive([]))
        return probability_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
