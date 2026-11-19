#!/usr/bin/env python3
"""Hardware-in-the-loop run: plant model on the PC, PID on the Tang Nano 9K.

Runs the same 4-second scenario as the RTL simulation, but each control
update goes over USB-UART to the FPGA. The result is checked against the
bit-exact golden model, so it shows the hardware itself matches.

  pip install pyserial
  make flash
  python3 host/hil.py               # auto-detects the Tang Nano port
  python3 host/hil.py /dev/ttyUSB1  # or name it
"""
import struct
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "model"))
import fc_model as m  # noqa: E402


def xor(bs):
    x = 0
    for b in bs:
        x ^= b
    return x


class FpgaPID:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(0.1)
        self.s.reset_input_buffer()
        self.frames = 0
        self.latency = []

    def set_gains(self, kp, ki, kd):
        p = struct.pack("<hHh", kp, ki, kd)
        self.s.write(b"\xC3" + p + bytes([xor(p)]))
        ack = self.s.read(1)
        if ack != b"\x3C":
            sys.exit(f"no gains ack from FPGA (got {ack!r}) - is the bitstream loaded?")

    def step(self, sp_q, ang_q, rate_q):
        p = struct.pack("<hhh", sp_q, ang_q, rate_q)
        t0 = time.perf_counter()
        self.s.write(b"\xA5" + p + bytes([xor(p)]))
        r = self.s.read(4)
        self.latency.append(time.perf_counter() - t0)
        if len(r) != 4 or r[0] != 0x5A or r[3] != r[1] ^ r[2]:
            sys.exit(f"bad reply at frame {self.frames}: {r!r}")
        self.frames += 1
        return struct.unpack("<h", r[1:3])[0]


def find_port():
    # Tang Nano 9K's BL702 shows up as two FTDI-like ports; the UART is the 2nd
    ports = sorted(p.device for p in serial.tools.list_ports.comports()
                   if "ttyUSB" in p.device or "ttyACM" in p.device)
    if not ports:
        sys.exit("no serial port found - plug in the Tang Nano 9K")
    return ports[-1]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"FPGA on {port}")
    fpga = FpgaPID(port)
    fpga.set_gains(m.KP_Q, m.KI_Q, m.KD_Q)

    t0 = time.time()
    hw = m.simulate("fpga", fpga)
    dt = time.time() - t0
    gold = m.simulate("golden")

    mism = [k for k in range(m.N_STEPS) if hw[k][3] != gold[k][3]]
    lat = sorted(fpga.latency)
    print(f"{fpga.frames} control updates in {dt:.1f} s, "
          f"round trip median {lat[len(lat)//2]*1e3:.2f} ms")
    if mism:
        print(f"FAIL: {len(mism)} samples differ from golden model, first k={mism[0]}")
    else:
        print(f"PASS: FPGA output matches golden model bit-exactly on all {m.N_STEPS} updates")

    out = m.RES / "hil_fpga.csv"
    with open(out, "w") as f:
        for row in hw:
            f.write(",".join(str(x) for x in row) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r[0] for r in hw]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(t, [r[1] for r in hw], "k--", lw=1, label="setpoint")
    ax.plot(t, [r[2] for r in gold], color="#999", lw=3, label="golden model")
    ax.plot(t, [r[2] for r in hw], color="#d29922", lw=1.2, label="Tang Nano 9K (hardware)")
    ax.axvspan(2.0, 2.05, color="#f85149", alpha=0.2, label="push")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("angle (deg)")
    ax.set_title("Hardware-in-the-loop: PID running on the FPGA, plant on the PC")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(m.RES / "hil_fpga.png", dpi=150)
    print(f"plot: {m.RES / 'hil_fpga.png'}")
    sys.exit(1 if mism else 0)


if __name__ == "__main__":
    main()
