#!/usr/bin/env python3
"""Step-by-step Tello link check (standard library only). Run it while connected to the TELLO-xxxxxx Wi-Fi.

    python scripts/tello_udp_check.py

Tests, in order: local address, which ports are free, 'command' from a random port, 'command' from
port 8889 (what djitellopy does), state packets on 8890, video packets on 11111. Writes a report to
logs/udp_check_<time>.txt so it can be read after reconnecting to the internet. Motors never start.
"""
import json
import platform
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

TELLO = ("192.168.10.1", 8889)
report = {"time": datetime.now().isoformat(timespec="seconds"), "python": sys.executable,
          "platform": platform.platform()}


def log(key, value):
    report[key] = value
    print(f"{key:28s} {value}")


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(TELLO)
        return s.getsockname()[0]
    finally:
        s.close()


def port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("", port))
        return True
    except OSError as e:
        return f"NO ({e})"
    finally:
        s.close()


def send_command(bind_port, cmd=b"command", timeout=3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.bind(("", bind_port))
        t0 = time.time()
        s.sendto(cmd, TELLO)
        data, addr = s.recvfrom(1024)
        return f"reply {data!r} from {addr} in {1000 * (time.time() - t0):.0f} ms"
    except socket.timeout:
        return "NO REPLY (timeout)"
    except OSError as e:
        return f"ERROR {e}"
    finally:
        s.close()


def listen(port, seconds=3.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    n, first = 0, None
    try:
        s.bind(("", port))
        t_end = time.time() + seconds
        while time.time() < t_end:
            try:
                data, _ = s.recvfrom(4096)
                n += 1
                first = first or data[:120]
            except socket.timeout:
                pass
        return f"{n} packets" + (f", first: {first!r}" if first else "")
    except OSError as e:
        return f"ERROR {e}"
    finally:
        s.close()


def shell(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10, shell=True).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f"ERROR {e}"


def main():
    ip = local_ip()
    log("local address", ip)
    if not ip.startswith("192.168.10."):
        log("verdict", "NOT on the Tello Wi-Fi: connect to TELLO-xxxxxx first")
        return finish()
    if sys.platform == "win32":
        log("wifi ssid", shell('netsh wlan show interfaces | findstr /R /C:"^ *SSID"'))
        log("network category", shell('powershell -NoProfile -Command "(Get-NetConnectionProfile -InterfaceAlias Wi-Fi).NetworkCategory"'))
    log("ping", "OK" if "TTL=" in shell("ping -n 2 192.168.10.1") or "ttl=" in shell("ping -c 2 192.168.10.1") else "FAILED")
    for p in (8889, 8890, 11111):
        log(f"port {p} free", port_free(p))
    log("command from random port", send_command(0))
    log("command from port 8889", send_command(8889))
    log("state on 8890 (3 s)", listen(8890))
    log("sdk? (from random port)", send_command(0, b"sdk?"))
    log("battery? (from 8889)", send_command(8889, b"battery?"))
    log("streamon", send_command(8889, b"streamon"))
    log("video on 11111 (3 s)", listen(11111))
    send_command(8889, b"streamoff", timeout=1.0)

    r1, r2 = report["command from random port"], report["command from port 8889"]
    if r1.startswith("reply") and r2.startswith("reply"):
        v = "UDP link OK: the problem is elsewhere (djitellopy setup), send this report"
    elif r1.startswith("reply"):
        v = "Replies reach a random port but not 8889: another program owns 8889, or a rule blocks that port"
    else:
        v = "The drone never answers UDP: firewall dropping replies, drone busy with another controller (phone app), or drone needs a restart"
    log("verdict", v)
    finish()


def finish():
    out = Path(__file__).resolve().parent.parent / "logs"
    out.mkdir(exist_ok=True)
    path = out / f"udp_check_{datetime.now():%Y%m%d_%H%M%S}.txt"
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"\nReport saved to {path}")


if __name__ == "__main__":
    main()
