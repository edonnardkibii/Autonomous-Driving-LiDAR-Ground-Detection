from ctypes import *
import struct


def hex2dec(s) -> int:
    ret = int(s, 16)
    return ret


def hex2float(s):
    i: int = int(s, 16)                  # Convert to int
    cp = pointer(c_int(i))               # Make into C-type integer & reference it's pointer
    fp = cast(cp, POINTER(c_float))      # cast int pointer to a float pointer
    ret = fp.contents.value              # dereference floating pointer & obtain the floating point value
    return ret


def hex2int32(s):
    ret = struct.unpack('>i', bytes.fromhex(s))[0]
    return ret


def list2int(s):
    res = sum(d * 10**i for i, d in enumerate(s[::-1]))
    return res
