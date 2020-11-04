from typing import Any

class Decoder:
    GUID_LENGTH: int = ...
    @classmethod
    def guid(cls, data: Any): ...
    @classmethod
    def decode(cls, data: Any, typ: Any): ...
    @classmethod
    def decode_message_size(cls, data: Any): ...
    @classmethod
    def decode_message(cls, data: Any, typ: Any): ...

class _ValueDecoder:
    @classmethod
    def decode_sint32(cls, data: Any): ...
    @classmethod
    def decode_sint64(cls, data: Any): ...
    @classmethod
    def decode_uint32(cls, data: Any): ...
    @classmethod
    def decode_uint64(cls, data: Any): ...
    @classmethod
    def decode_double(cls, data: Any): ...
    @classmethod
    def decode_float(cls, data: Any): ...
    @classmethod
    def decode_bool(cls, data: Any): ...
    @classmethod
    def decode_string(cls, data: Any): ...
    @classmethod
    def decode_bytes(cls, data: Any): ...