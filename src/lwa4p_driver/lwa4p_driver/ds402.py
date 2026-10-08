import struct
import time

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE = 0x000F
CW_FAULT_RESET = 0x0080
CW_NEW_SETPOINT = 0x003F
CW_SETPOINT_DONE = 0x002F

MODE_PP = 1


def rd(node, idx, sub, fmt):
    data = node.sdo.upload(idx, sub)
    return struct.unpack(fmt, data[:struct.calcsize(fmt)])[0]


def wr(node, idx, sub, fmt, value):
    node.sdo.download(idx, sub, struct.pack(fmt, value))


def statusword(node):
    return rd(node, 0x6041, 0, "<H")


def controlword(node, value):
    wr(node, 0x6040, 0, "<H", value)


def position(node):
    return rd(node, 0x6064, 0, "<i")


def sw_limits(node):
    return rd(node, 0x607D, 1, "<i"), rd(node, 0x607D, 2, "<i")


def is_fault(sw):
    return bool(sw & 0x0008)


def is_enabled(sw):
    return sw & 0x006F == 0x0027


def ready_state(sw):
    if sw & 0x004F == 0x0040:
        return "switch on disabled"
    if sw & 0x006F == 0x0021:
        return "ready to switch on"
    if sw & 0x006F == 0x0023:
        return "switched on"
    if sw & 0x006F == 0x0027:
        return "operation enabled"
    return None


def last_error(node):
    try:
        if rd(node, 0x1003, 0, "<B"):
            return rd(node, 0x1003, 1, "<I") & 0xFFFF
    except Exception:
        pass
    return None


def wait_status(node, mask, value, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        sw = statusword(node)
        if sw & mask == value:
            return sw
        time.sleep(0.02)
    raise TimeoutError(f"node {node.id}: statusword 0x{statusword(node):04X}, "
                       f"wanted mask 0x{mask:X}=0x{value:X}")


def bring_up(node):
    node.nmt.send_command(0x01)
    time.sleep(0.05)
    sw = statusword(node)
    if not ready_state(sw):
        controlword(node, CW_FAULT_RESET)
        time.sleep(0.1)
        controlword(node, 0x0000)
        time.sleep(0.1)
        sw = statusword(node)
    return sw


def setup_pp(node, accel_mdeg):
    wr(node, 0x6060, 0, "<b", MODE_PP)
    time.sleep(0.02)
    wr(node, 0x6083, 0, "<I", int(accel_mdeg))


def enable(node):
    wr(node, 0x607A, 0, "<i", position(node))
    if ready_state(statusword(node)) == "operation enabled":
        return
    controlword(node, CW_SHUTDOWN)
    wait_status(node, 0x006F, 0x0021)
    controlword(node, CW_SWITCH_ON)
    wait_status(node, 0x006F, 0x0023)
    controlword(node, CW_ENABLE)        # brake opens
    # first enable after power-up may do a commutation search (small wiggle)
    wait_status(node, 0x006F, 0x0027, timeout=10.0)


def disable(node):
    try:
        controlword(node, CW_SWITCH_ON)
        time.sleep(0.05)
        controlword(node, CW_SHUTDOWN)
    except Exception:
        pass


def send_target(node, target_mdeg, vel_mdeg):
    wr(node, 0x6081, 0, "<I", max(1, int(vel_mdeg)))
    wr(node, 0x607A, 0, "<i", int(target_mdeg))
    controlword(node, CW_NEW_SETPOINT)
    controlword(node, CW_SETPOINT_DONE)


def single_move(node, target_mdeg, vel_mdeg, tol_mdeg=100, timeout=6.0):
    wr(node, 0x6081, 0, "<I", max(1, int(vel_mdeg)))
    wr(node, 0x607A, 0, "<i", int(target_mdeg))
    controlword(node, 0x001F)
    time.sleep(0.05)
    controlword(node, CW_ENABLE)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if abs(position(node) - target_mdeg) < tol_mdeg:
            return True
        if is_fault(statusword(node)):
            return False
        time.sleep(0.05)
    return False


def commutate(node, step_mdeg=500, vel_mdeg=2000):
    start = position(node)
    ok = single_move(node, start + step_mdeg, vel_mdeg)
    back = single_move(node, start, vel_mdeg)
    return ok and back
