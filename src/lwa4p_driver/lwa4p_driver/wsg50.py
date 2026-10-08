import struct
import sys
import time

import can

TX_ID, RX_ID = 0x01, 0x02

STATUS = {0: "E_SUCCESS", 1: "E_NOT_AVAILABLE", 2: "E_NO_SENSOR", 3: "E_NOT_INITIALIZED",
          4: "E_ALREADY_RUNNING", 5: "E_FEATURE_NOT_SUPPORTED", 6: "E_INCONSISTENT_DATA",
          7: "E_TIMEOUT", 8: "E_READ_ERROR", 9: "E_WRITE_ERROR", 10: "E_INSUFFICIENT_RESOURCES",
          11: "E_CHECKSUM_ERROR", 12: "E_NO_PARAM_EXPECTED", 13: "E_NOT_ENOUGH_PARAMS",
          14: "E_CMD_UNKNOWN", 15: "E_CMD_FORMAT_ERROR", 16: "E_ACCESS_DENIED",
          17: "E_ALREADY_OPEN", 18: "E_CMD_FAILED", 19: "E_CMD_ABORTED", 20: "E_INVALID_HANDLE",
          21: "E_NOT_FOUND", 22: "E_NOT_OPEN", 23: "E_IO_ERROR", 24: "E_INVALID_PARAMETER",
          25: "E_INDEX_OUT_OF_BOUNDS", 26: "E_CMD_PENDING", 27: "E_OVERRUN", 28: "E_RANGE_ERROR",
          29: "E_AXIS_BLOCKED", 30: "E_FILE_EXISTS"}
E_SUCCESS, E_CMD_PENDING = 0, 26

GRASP_STATE = {0: "IDLE", 1: "GRASPING", 2: "NO PART FOUND", 3: "PART LOST", 4: "HOLDING",
               5: "RELEASING", 6: "POSITIONING", 7: "ERROR"}


SYS_FLAGS = {0: "REFERENCED", 1: "MOVING", 2: "BLOCKED_MINUS", 3: "BLOCKED_PLUS",
             4: "SOFT_LIMIT_MINUS", 5: "SOFT_LIMIT_PLUS", 6: "AXIS_STOPPED",
             7: "TARGET_POS_REACHED", 8: "OVERDRIVE_MODE", 12: "FAST_STOP",
             13: "TEMP_WARNING", 14: "TEMP_FAULT", 15: "POWER_FAULT", 16: "CURR_FAULT",
             17: "FINGER_FAULT", 18: "CMD_FAILURE", 19: "SCRIPT_RUNNING",
             20: "SCRIPT_FAILURE", 31: "ERROR"}

_T = []
for _i in range(256):
    _c = _i << 8
    for _ in range(8):
        _c = ((_c << 1) ^ 0x1021) if _c & 0x8000 else (_c << 1)
    _T.append(_c & 0xFFFF)


def crc16(data, c=0xFFFF):
    for b in data:
        c = _T[(c ^ b) & 0xFF] ^ (c >> 8)
    return c


class WSG:
    def __init__(self, channel="can1"):
        self.bus = can.Bus(interface="socketcan", channel=channel,
                           can_filters=[{"can_id": RX_ID, "can_mask": 0x7FF}])
        while self.bus.recv(0.02):
            pass

    def close(self):
        self.bus.shutdown()

    def send(self, cmd, payload=b""):
        p = bytes([0xAA, 0xAA, 0xAA, cmd]) + struct.pack("<H", len(payload)) + payload
        p += struct.pack("<H", crc16(p))
        for k in range(0, len(p), 8):
            self.bus.send(can.Message(arbitration_id=TX_ID, data=p[k:k + 8], is_extended_id=False))
            time.sleep(0.001)

    def receive(self, cmd, timeout=1.0):
        buf, t0 = b"", time.time()
        while time.time() - t0 < timeout:
            m = self.bus.recv(0.05)
            if m is None:
                continue
            buf += bytes(m.data)
            while len(buf) >= 3 and buf[:3] != b"\xAA\xAA\xAA":   # resync
                buf = buf[1:]
            if len(buf) >= 6:
                n = struct.unpack("<H", buf[4:6])[0]
                if len(buf) >= 8 + n:
                    pkt, buf = buf[:8 + n], buf[8 + n:]
                    if crc16(pkt[:-2]) != struct.unpack("<H", pkt[-2:])[0]:
                        raise IOError("reply checksum error")
                    if pkt[3] != cmd:
                        continue
                    body = pkt[6:6 + n]
                    return struct.unpack("<H", body[:2])[0], body[2:]
        raise TimeoutError(f"no reply to command 0x{cmd:02X}")

    def call(self, cmd, payload=b"", wait_done=False, timeout=30.0):
        self.send(cmd, payload)
        st, data = self.receive(cmd)
        if st == E_CMD_PENDING and wait_done:
            st, data = self.receive(cmd, timeout)
        return st, data

    # ---- queries ----
    def width(self):
        return self._float(0x43)

    def speed(self):
        return self._float(0x44)

    def force(self):
        return self._float(0x45)

    def _float(self, cmd):
        st, d = self.call(cmd, bytes([0, 0, 0]))   # no auto-update
        check(st, cmd)
        return struct.unpack("<f", d[:4])[0]

    def system_state(self):
        st, d = self.call(0x40, bytes([0, 0, 0]))
        check(st, 0x40)
        flags = struct.unpack("<I", d[:4])[0]
        return flags, [n for b, n in SYS_FLAGS.items() if flags & (1 << b)]

    def grasping_state(self):
        st, d = self.call(0x41, bytes([0, 0, 0]))
        check(st, 0x41)
        return GRASP_STATE.get(d[0], str(d[0]))

    def get_param(self, cmd):
        st, d = self.call(cmd)
        check(st, cmd)
        return struct.unpack("<f", d[:4])[0]

    # ---- actions ----
    def home(self, direction=0):
        return self.call(0x20, bytes([direction]), wait_done=True)[0]

    def move(self, width, speed=50.0):
        return self.call(0x21, bytes([0]) + struct.pack("<ff", width, speed), wait_done=True)[0]

    def grasp(self, width, speed=50.0):
        return self.call(0x25, struct.pack("<ff", width, speed), wait_done=True)[0]

    def release(self, width, speed=50.0):
        return self.call(0x26, struct.pack("<ff", width, speed), wait_done=True)[0]

    def stop(self):
        return self.call(0x22)[0]

    def ack(self):
        return self.call(0x24, b"ack")[0]

    def set_force(self, newton):
        return self.call(0x32, struct.pack("<f", newton))[0]

    def set_accel(self, mm_s2):
        return self.call(0x30, struct.pack("<f", mm_s2))[0]


def check(st, cmd):
    if st not in (E_SUCCESS, E_CMD_PENDING):
        raise RuntimeError(f"command 0x{cmd:02X} failed: {STATUS.get(st, st)}")
