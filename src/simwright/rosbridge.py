"""ROS 2 bridge between Isaac Sim (Windows host, bundled ROS 2 libraries) and ROS 2 in WSL or Linux.

What has to be true for topics to cross, all handled here:
1. Isaac Sim starts with ROS_DISTRO, RMW_IMPLEMENTATION=rmw_fastrtps_cpp, the bundled <distro>/lib on PATH
   (LD_LIBRARY_PATH on Linux) and the isaacsim.ros2.bridge extension (`simwright isaac launch --ros`).
2. Fast DDS uses UDP only (shared memory cannot cross Windows <-> WSL) and discovers peers by unicast through
   127.0.0.1 (multicast from Windows does not reach WSL in mirrored networking). Same profile on both sides.
3. Same ROS_DOMAIN_ID on both sides (read from WSL's ~/.bashrc if set there).
4. WSL's Hyper-V firewall lets the DDS ports of that domain in from this PC only.
"""
from __future__ import annotations
import json, os, pathlib, re
from . import config

PEERS = ["127.0.0.1"]


def dds_ports(domain: int) -> tuple[int, int]:
    """RTPS well-known ports for a domain: PB 7400 + DG 250 * domain, then 250 ports for participants."""
    base = 7400 + 250 * int(domain)
    return base, base + 249


def profile_xml(peers: list[str] | None = None) -> str:
    peers = PEERS if peers is None else peers
    loc = "".join(f"\n                    <locator><udpv4><address>{p}</address></udpv4></locator>" for p in peers)
    initial = (f"\n            <builtin>\n                <initialPeersList>{loc}\n                </initialPeersList>\n            </builtin>" if peers else "")
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<!-- Simwright: Fast DDS over UDP only with unicast discovery. Same file for Isaac Sim and ROS 2 (WSL / Linux). -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>{initial}
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
"""


def settings() -> dict:
    s = dict(config.load().get("ros_bridge") or {})
    s.setdefault("distro", "humble")
    s.setdefault("domain_id", 0)
    s.setdefault("profile", str(config.state_file("fastdds.xml")))
    return s


def launch_env(isaac_dir: str, s: dict | None = None, base_env: dict | None = None) -> dict:
    """Environment for starting Isaac Sim with its bundled ROS 2 libraries."""
    s = s or settings()
    env = dict(base_env if base_env is not None else os.environ)
    lib = os.path.join(isaac_dir, "exts", "isaacsim.ros2.core", s["distro"], "lib")
    env["ROS_DISTRO"] = s["distro"]
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_DOMAIN_ID"] = str(int(s["domain_id"]))
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = s["profile"]
    key, sep = ("PATH", ";") if os.name == "nt" else ("LD_LIBRARY_PATH", ":")
    cur = env.get(key, "")
    if lib.lower() not in cur.lower():
        env[key] = cur + (sep if cur else "") + lib
    return env


LAUNCH_FLAG = "--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge"


def wsl_setup_script(profile_src_mnt: str, domain: int | None) -> str:
    dom = "" if domain is None else f"export ROS_DOMAIN_ID={int(domain)}\n"
    return f"""#!/bin/bash
set -e
mkdir -p ~/.ros
cp '{profile_src_mnt}' ~/.ros/fastdds.xml
if ! grep -q '# >>> simwright ros bridge' ~/.bashrc; then
cat >> ~/.bashrc <<'EOF'
# >>> simwright ros bridge (Isaac Sim on Windows <-> ROS 2 in WSL; remove this block to undo)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds.xml
{dom}# <<< simwright ros bridge
EOF
fi
echo ok
"""


def firewall_ps(domain: int) -> str:
    lo, hi = dds_ports(domain)
    name = f"Simwright-DDS-WSL-In-d{int(domain)}"
    return ("$ips = @(Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '169.254*' } | "
            "Select-Object -ExpandProperty IPAddress) + '127.0.0.1' | Sort-Object -Unique; "
            f"Remove-NetFirewallHyperVRule -Name '{name}' -ErrorAction SilentlyContinue; "
            f"New-NetFirewallHyperVRule -Name '{name}' -DisplayName 'Simwright: ROS 2 DDS from this PC into WSL (domain {int(domain)})' "
            "-Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP "
            f"-LocalPorts {lo}-{hi} -RemoteAddresses $ips -Action Allow | Out-Null; 'firewall rule {name}: UDP {lo}-{hi}'")


def check_script(domain: int, topics: list[str], seconds: int = 6) -> str:
    """Bash for WSL / Linux: list topics, one /clock sample, rates. Prints one JSON line at the end."""
    ts = " ".join(topics)
    return f"""#!/bin/bash
source /opt/ros/${{ROS_DISTRO:-humble}}/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash
export ROS_DOMAIN_ID={int(domain)} RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds.xml
ros2 daemon stop >/dev/null 2>&1
TOPICS=$(timeout 15 ros2 topic list 2>/dev/null | tr '\\n' ' ')
CLOCK=$(timeout 10 ros2 topic echo /clock --once 2>/dev/null | grep -m1 'sec:' | tr -dc '0-9')
RATES=""
for t in {ts}; do r=$(timeout {seconds} ros2 topic hz $t 2>/dev/null | grep -m1 'average rate' | grep -oE '[0-9.]+'); RATES="$RATES\\"$t\\": \\"${{r:-0}}\\","; done
echo "SWJSON {{\\"topics\\": \\"$TOPICS\\", \\"clock_sec\\": \\"$CLOCK\\", \\"rates\\": {{${{RATES%,}}}}}}"
"""


def parse_check(out: str) -> dict:
    m = [l for l in out.splitlines() if l.startswith("SWJSON ")]
    if not m:
        return {"ok": False, "error": "no result from ROS 2", "raw": out[-800:]}
    j = json.loads(m[-1][7:])
    topics = sorted(t for t in j.get("topics", "").split() if t)
    rates = {k: float(v or 0) for k, v in (j.get("rates") or {}).items()}
    sim_topics = [t for t in topics if t not in ("/parameter_events", "/rosout")]
    return {"ok": bool(j.get("clock_sec")) and all(v > 0 for v in rates.values()), "topics": topics, "from_isaac_sim": sim_topics,
            "clock_sec": int(j["clock_sec"]) if j.get("clock_sec") else None, "rates_hz": rates}


def wsl_domain_from_bashrc(text: str) -> int | None:
    m = re.findall(r"^\s*export\s+ROS_DOMAIN_ID=(\d+)", text or "", re.M)
    return int(m[-1]) if m else None
