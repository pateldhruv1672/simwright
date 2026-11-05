"""Live ROS 2 introspection through the ROS target (WSL, Docker or native).

One bash script per question keeps it to a single round trip into WSL.
"""
from __future__ import annotations
import re
from collections import defaultdict

GRAPH_SCRIPT = r"""
echo "@@DISTRO ${ROS_DISTRO:-none}"
command -v ros2 >/dev/null || { echo "@@ERROR ros2 not found: source your ROS setup in this target"; exit 3; }
echo "@@NODES"; timeout 15 ros2 node list 2>&1
echo "@@TOPICS"; timeout 15 ros2 topic list -t 2>&1
n=0
for t in $(timeout 15 ros2 topic list 2>/dev/null); do
  n=$((n+1)); [ $n -gt {limit} ] && { echo "@@TRUNCATED"; break; }
  echo "@@TOPIC $t"; timeout 10 ros2 topic info -v "$t" 2>&1
done
echo "@@CONTROLLERS"; timeout 8 ros2 control list_controllers 2>&1 || true
echo "@@END"
"""


def graph_script(limit: int = 80) -> str:
    return GRAPH_SCRIPT.replace("{limit}", str(limit))


def parse_endpoints(info: str) -> list[dict]:
    """Parse `ros2 topic info -v` output into endpoints with QoS."""
    eps, cur = [], None
    for line in info.splitlines():
        s = line.strip()
        if s.startswith("Node name:"):
            cur = {"node": s.split(":", 1)[1].strip(), "ns": "/", "type": "", "kind": "", "qos": {}}
            eps.append(cur)
        elif cur is None:
            continue
        elif s.startswith("Node namespace:"):
            cur["ns"] = s.split(":", 1)[1].strip()
        elif s.startswith("Topic type:"):
            cur["type"] = s.split(":", 1)[1].strip()
        elif s.startswith("Endpoint type:"):
            cur["kind"] = s.split(":", 1)[1].strip()
        elif s.startswith("Reliability:"):
            cur["qos"]["reliability"] = s.split(":", 1)[1].strip()
        elif s.startswith("Durability:"):
            cur["qos"]["durability"] = s.split(":", 1)[1].strip()
        elif s.startswith("History (Depth):"):
            cur["qos"]["history"] = s.split(":", 1)[1].strip()
    for e in eps:
        e["fqn"] = (e["ns"].rstrip("/") + "/" + e["node"]).replace("//", "/") if e["node"] != "_NODE_NAME_UNKNOWN_" else "(unknown)"
    return eps


def qos_issues(topic: str, eps: list[dict]) -> list[dict]:
    pubs = [e for e in eps if e["kind"].upper().startswith("PUB")]
    subs = [e for e in eps if e["kind"].upper().startswith("SUB")]
    out = []
    types = {e["type"] for e in eps if e["type"]}
    if len(types) > 1:
        out.append({"topic": topic, "severity": "error", "issue": "type mismatch", "detail": ", ".join(sorted(types))})
    for p in pubs:
        for s in subs:
            pr, sr = p["qos"].get("reliability", ""), s["qos"].get("reliability", "")
            pd, sd = p["qos"].get("durability", ""), s["qos"].get("durability", "")
            if pr.startswith("BEST_EFFORT") and sr.startswith("RELIABLE"):
                out.append({"topic": topic, "severity": "error", "issue": "QoS incompatible: reliability",
                            "detail": f"{p['fqn']} publishes BEST_EFFORT but {s['fqn']} requests RELIABLE; no messages will arrive",
                            "fix": f"make {s['fqn']} subscribe with best-effort (rclcpp::SensorDataQoS() / qos_profile_sensor_data)"})
            if pd.startswith("VOLATILE") and sd.startswith("TRANSIENT_LOCAL"):
                out.append({"topic": topic, "severity": "error", "issue": "QoS incompatible: durability",
                            "detail": f"{p['fqn']} publishes VOLATILE but {s['fqn']} requests TRANSIENT_LOCAL",
                            "fix": f"publish with transient_local (latched) or subscribe volatile"})
    if pubs and not subs and not topic.startswith(("/rosout", "/parameter_events")):
        out.append({"topic": topic, "severity": "info", "issue": "no subscribers", "detail": ", ".join(p["fqn"] for p in pubs)})
    if subs and not pubs:
        out.append({"topic": topic, "severity": "warn", "issue": "no publishers",
                    "detail": "waiting: " + ", ".join(s["fqn"] for s in subs)})
    return out


def parse_graph(text: str) -> dict:
    out = {"distro": "", "nodes": [], "topics": {}, "controllers": [], "issues": [], "error": "", "truncated": False}
    sections = re.split(r"^@@", text, flags=re.M)
    for sec in sections:
        head, _, body = sec.partition("\n")
        head = head.strip()
        if head.startswith("DISTRO"):
            out["distro"] = head.split(" ", 1)[1] if " " in head else ""
        elif head.startswith("ERROR"):
            out["error"] = head[6:]
        elif head == "NODES":
            out["nodes"] = [l.strip() for l in body.splitlines() if l.strip().startswith("/")]
        elif head == "TOPICS":
            for l in body.splitlines():
                m = re.match(r"^(/\S+)\s+\[(.+)\]$", l.strip())
                if m:
                    out["topics"].setdefault(m.group(1), {"type": m.group(2), "endpoints": []})
        elif head.startswith("TOPIC "):
            t = head.split(" ", 1)[1]
            eps = parse_endpoints(body)
            out["topics"].setdefault(t, {"type": eps[0]["type"] if eps else "", "endpoints": []})["endpoints"] = eps
            out["issues"] += qos_issues(t, eps)
        elif head == "CONTROLLERS":
            out["controllers"] = [l.strip() for l in body.splitlines() if l.strip() and "error" not in l.lower()]
        elif head == "TRUNCATED":
            out["truncated"] = True
    return out


def trace(graph: dict, topic: str) -> dict:
    """Follow a command topic: who publishes, who consumes, which controllers are active."""
    t = graph["topics"].get(topic)
    if not t:
        near = [x for x in graph["topics"] if topic.strip("/").split("/")[-1] in x]
        return {"topic": topic, "found": False, "similar": near[:10],
                "hint": "topic does not exist; check remappings and namespaces"}
    pubs = [e["fqn"] for e in t["endpoints"] if e["kind"].upper().startswith("PUB")]
    subs = [e["fqn"] for e in t["endpoints"] if e["kind"].upper().startswith("SUB")]
    downstream = defaultdict(list)
    for name, info in graph["topics"].items():
        for e in info["endpoints"]:
            if e["kind"].upper().startswith("PUB") and e["fqn"] in subs:
                downstream[e["fqn"]].append(name)
    return {"topic": topic, "found": True, "type": t["type"], "publishers": pubs, "subscribers": subs,
            "subscriber_outputs": dict(downstream), "controllers": graph.get("controllers", []),
            "issues": [i for i in graph["issues"] if i["topic"] == topic]}
