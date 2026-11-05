"""Read colcon output, find the root cause, propose the coordinated fix."""
from __future__ import annotations
import re

RULES = [
    (r'Could not find a package configuration file provided by\s+"(\w+)"', "missing_cmake_dep",
     lambda m: f'add `<depend>{to_key(m.group(1))}</depend>` to package.xml, `find_package({m.group(1)} REQUIRED)` and link it with ament_target_dependencies(...); then `rosdep install --from-paths src -y --ignore-src`'),
    (r"fatal error: ([\w./]+)\.(?:h|hpp|hh): No such file or directory", "missing_include",
     lambda m: f"header `{m.group(1)}` not found: add `<depend>{m.group(1).split('/')[0]}</depend>` and include it in ament_target_dependencies, or fix the include path"),
    (r"undefined reference to [`'](\S+?)'", "link_error",
     lambda m: f"symbol `{m.group(1)[:80]}` not linked: add the library to ament_target_dependencies/target_link_libraries for this target"),
    (r"No module named '([\w.]+)'", "missing_python_dep",
     lambda m: f"Python module `{m.group(1)}` missing: add `<exec_depend>{to_pykey(m.group(1))}</exec_depend>` and install it in the ROS target"),
    (r"Cannot find source file:\s*\n?\s*(\S+)", "missing_source",
     lambda m: f"CMakeLists.txt lists `{m.group(1)}` which does not exist: fix the path or create the file"),
    (r"error: package directory '([^']+)' does not exist", "setup_py_package",
     lambda m: f"setup.py packages=['{m.group(1)}'] but there is no `{m.group(1)}/` folder with __init__.py"),
    (r"Duplicate package names not supported:\s*\n?\s*- (\S+)", "duplicate_package",
     lambda m: f"package `{m.group(1)}` exists twice in the workspace: remove or COLCON_IGNORE one copy"),
    (r"Could not find a package configuration file provided by \"ament_cmake\"|ament_cmake.*not found", "ros_not_sourced",
     lambda m: "ROS environment not sourced in this shell: `source /opt/ros/<distro>/setup.bash` (set the target's setup in ~/.simwright/config.json)"),
    (r"Error\(s\) in package '([^']+)'", "bad_package_xml",
     lambda m: f"package.xml at `{m.group(1)}` is invalid: check format=3 schema, tags order and names"),
    (r"(\S+\.(?:msg|srv|action)):\s*\d+.*error|Unable to find (?:message|service) type ['\"]?(\S+)", "interface_error",
     lambda m: "interface generation failed: check the .msg/.srv field types and add the interface package as <depend> + rosidl_generate_interfaces DEPENDENCIES"),
    (r"([\w/.-]+\.(?:cpp|cc|hpp|h)):(\d+):\d+: error: (.+)", "compile_error",
     lambda m: f"compile error at {m.group(1)}:{m.group(2)}: {m.group(3)[:160]}"),
    (r"No rule to make target '([^']+)'", "stale_build",
     lambda m: f"stale build state for `{m.group(1)}`: delete build/<pkg> and install/<pkg> for that package, rebuild"),
    (r"SetuptoolsDeprecationWarning|easy_install command is deprecated", "setuptools_warning",
     lambda m: "harmless setuptools deprecation warning; ignore unless the build failed for another reason"),
]

PY_KEYS = {"yaml": "python3-yaml", "numpy": "python3-numpy", "scipy": "python3-scipy", "cv2": "python3-opencv",
           "transforms3d": "python3-transforms3d", "serial": "python3-serial", "requests": "python3-requests"}


def to_key(cmake_name: str) -> str:
    known = {"Eigen3": "eigen", "OpenCV": "libopencv-dev", "PCL": "libpcl-dev", "yaml-cpp": "yaml-cpp", "Boost": "boost"}
    if cmake_name in known:
        return known[cmake_name]
    return re.sub(r"(?<!^)(?=[A-Z])", "_", cmake_name).lower() if not "_" in cmake_name else cmake_name


def to_pykey(mod: str) -> str:
    return PY_KEYS.get(mod.split(".")[0], mod.split(".")[0])


def failed_packages(text: str) -> list[str]:
    return re.findall(r"Failed\s+<<<\s+(\S+)", text)


def diagnose(text: str) -> dict:
    failed = failed_packages(text)
    diags, seen = [], set()
    # attribute lines to the package whose stderr block they are in
    current = None
    for line_no, line in enumerate(text.splitlines()):
        m = re.search(r"--- stderr: (\S+)|Starting >>> (\S+)", line)
        if m:
            current = m.group(1) or m.group(2)
    for pat, kind, fix in RULES:
        for m in re.finditer(pat, text):
            key = (kind, m.group(0)[:120])
            if key in seen:
                continue
            seen.add(key)
            pkg = _pkg_for(text, m.start()) or current
            diags.append({"kind": kind, "package": pkg, "evidence": m.group(0).strip()[:300], "fix": fix(m)})
    order = ["ros_not_sourced", "duplicate_package", "bad_package_xml", "missing_cmake_dep", "missing_include",
             "interface_error", "missing_source", "setup_py_package", "missing_python_dep", "link_error",
             "compile_error", "stale_build", "setuptools_warning"]
    diags.sort(key=lambda d: order.index(d["kind"]) if d["kind"] in order else 99)
    summary = re.search(r"Summary: (.+)", text)
    return {"failed_packages": failed, "summary": summary.group(1) if summary else "",
            "root_cause": diags[0] if diags and diags[0]["kind"] != "setuptools_warning" else None, "diagnoses": diags[:20]}


def _pkg_for(text: str, pos: int) -> str | None:
    before = text[:pos]
    m = list(re.finditer(r"--- stderr: (\S+)|Starting >>> (\S+)", before))
    if not m:
        return None
    last = m[-1]
    return last.group(1) or last.group(2)
