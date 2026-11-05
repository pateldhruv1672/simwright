"""Connect Simwright on the user's PC to Mana's models through the sandbox relay (no API key on the PC).

Writes ~/.simwright/mana.json on every paired Windows/Linux machine: {base_url, token, expires}. The ingress link lasts
30 days; renew() re-mints it when fewer than 7 days remain and rewrites the file. Called by autosync every 15 min.
"""
import json, pathlib, sys, time

R = pathlib.Path("/workspace/simwright-relay")


def link(min_days: float = 7.0) -> dict:
    info = json.loads((R / "ingress.json").read_text()) if (R / "ingress.json").exists() else {}
    if not info or info.get("expires", 0) - time.time() < min_days * 86400:
        from ingress.utils import mint_ingress_token
        m = mint_ingress_token("simwright-relay", ttl_seconds=30 * 86400)
        info = {"url": m["ingress_url"], "expires": int(time.time()) + 30 * 86400, "minted": int(time.time())}
        (R / "ingress.json").write_text(json.dumps(info))
    return info


def push(min_days: float = 7.0, force: bool = False) -> dict:
    """Write mana.json to every paired machine whose copy is missing, stale or older than the current link."""
    info = link(min_days)
    payload = {"base_url": info["url"] + "/v1", "token": (R / "token").read_text().strip(), "expires": info["expires"],
               "note": "Written by Mana: Simwright uses Mana's models through your sandbox relay. Billed to your Mana credits."}
    sys.path.insert(0, "/workspace")
    from mana_terminal import terms, term
    done, seen = [], set()
    for w in terms() or []:
        dev = w.get("device")
        if dev in seen:
            continue
        seen.add(dev)
        t = term(w["sid"])
        home = w.get("home") or ""
        sep = "\\" if w.get("os", "").startswith("Windows") else "/"
        path = home + sep + ".simwright" + sep + "mana.json"
        try:
            cur = json.loads(t.read(path)) if not force else {}
        except Exception:
            cur = {}
        if not force and cur.get("base_url") == payload["base_url"] and cur.get("token") == payload["token"]:
            done.append({"device": w.get("label"), "status": "current"})
            continue
        t.write(path, json.dumps(payload, indent=2))
        done.append({"device": w.get("label"), "status": "written", "path": path})
    return {"ok": True, "expires": info["expires"], "machines": done}
