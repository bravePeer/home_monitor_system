def bytes_to_str(data: bytes | bytearray)->str:
    ret = ""
    for i in data:
        ret += f" {i:02X}"
    return ret


def int_to_bytes(value: int) -> bytes:
    data = []
    data.append((value >> 24) & 0xff)
    data.append((value >> 16) & 0xff)
    data.append((value >> 8) & 0xff)
    data.append(value & 0xff)
    return bytes(data)