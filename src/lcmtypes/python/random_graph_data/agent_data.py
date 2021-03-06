"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class agent_data(object):
    __slots__ = ["init_pos_", "idx_"]

    __typenames__ = ["int32_t", "int32_t"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.init_pos_ = 0
        self.idx_ = 0

    def encode(self):
        buf = BytesIO()
        buf.write(agent_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ii", self.init_pos_, self.idx_))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != agent_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return agent_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = agent_data()
        self.init_pos_, self.idx_ = struct.unpack(">ii", buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if agent_data in parents: return 0
        tmphash = (0x48dd73df93cd94b7) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if agent_data._packed_fingerprint is None:
            agent_data._packed_fingerprint = struct.pack(">Q", agent_data._get_hash_recursive([]))
        return agent_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

