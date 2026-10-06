#!/usr/bin/env python3
"""Build the pinned Gazebo Harmonic MID-360 backend in the project cache.

Run with Python 3.12 on Ubuntu 24.04 after installing ROS 2 Jazzy. No sudo,
global installation, Python packages, or modifications of upstream C++ are
needed. Downloads have fixed SHA-256 hashes; all build inputs stay below
Agricultural_Bot/.cache/rgl. Use the named ``Livox Mid360`` preset and the
reported patterns directory, rather than treating its 40 scans as one scan.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import subprocess
import sys
import tarfile
from urllib.request import Request, urlopen
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLUGIN_COMMIT = "d4bf3cf36fe4a363a56df1bec2ce3809720db563"
CORE_VERSION = "0.21.0"
CORE_COMMIT = "a65f07f9565adbfe5cda33a58732638786cb34e8"
PATTERN_GIT_BLOB = "3d8faa6a44b8eaac5d56ffb320e3b10546705576"
DOWNLOADS = {
    f"RGLGazeboPlugin-{PLUGIN_COMMIT}.tar.gz": (
        f"https://codeload.github.com/RobotecAI/RGLGazeboPlugin/tar.gz/{PLUGIN_COMMIT}",
        "3f2d09a7e49c2c336a36f390d59909df555ace97ba54217143b1e2d7a9e77f2d",
    ),
    "RGL-core-linux-x64-v0.21.0.zip": (
        "https://github.com/RobotecAI/RobotecGPULidar/releases/download/v0.21.0/RGL-core-linux-x64.zip",
        "59518816f1818fb1dfb39cc6af48022708b71e7980b45a412942d4f928b661bc",
    ),
    "core-v0.21.0.h": (
        f"https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/{CORE_COMMIT}/include/rgl/api/core.h",
        "02857a1909944ae7f5a5fe47394fe84828e31ce5ef9d80b222b9f439dc397414",
    ),
    "RobotecGPULidar-v0.21.0.LICENSE": (
        f"https://raw.githubusercontent.com/RobotecAI/RobotecGPULidar/{CORE_COMMIT}/LICENSE",
        "3f1b4d99668b72cc1fad2e0e370734cac1d5b4982a421cb153d4fee92643497e",
    ),
}

# This runs in a child process so the dynamic loader sees LD_LIBRARY_PATH
# before Python starts. The one-ray probe executes CUDA/OptiX, not only dlopen.
PROBE = r'''
import ctypes as c
import json
from pathlib import Path
import sys

directory = Path(sys.argv[1]) / "RGLServerPlugin"
core = c.CDLL(str(directory / "libRobotecGPULidar.so"), mode=c.RTLD_GLOBAL)
for name in ("libRGLServerPluginManager.so", "libRGLServerPluginInstance.so"):
    c.CDLL(str(directory / name), mode=c.RTLD_GLOBAL)

core.rgl_get_last_error_string.argtypes = [c.POINTER(c.c_char_p)]
core.rgl_get_last_error_string.restype = None
def check(status):
    if status:
        message = c.c_char_p()
        core.rgl_get_last_error_string(c.byref(message))
        raise RuntimeError(message.value.decode() if message.value else str(status))

version = [c.c_int32() for _ in range(3)]
core.rgl_get_version_info.argtypes = [c.POINTER(c.c_int32)] * 3
check(core.rgl_get_version_info(*(c.byref(value) for value in version)))
assert tuple(value.value for value in version) == (0, 21, 0)

node_ptr = c.POINTER(c.c_void_p)
core.rgl_node_rays_from_mat3x4f.argtypes = [node_ptr, c.POINTER(c.c_float), c.c_int32]
core.rgl_node_rays_set_range.argtypes = [node_ptr, c.POINTER(c.c_float), c.c_int32]
core.rgl_node_raytrace.argtypes = [node_ptr, c.c_void_p]
core.rgl_node_points_yield.argtypes = [node_ptr, c.POINTER(c.c_int32), c.c_int32]
core.rgl_graph_node_add_child.argtypes = [c.c_void_p, c.c_void_p]
core.rgl_graph_run.argtypes = [c.c_void_p]
core.rgl_graph_get_result_size.argtypes = [c.c_void_p, c.c_int32, c.POINTER(c.c_int32), c.POINTER(c.c_int32)]
core.rgl_graph_get_result_data.argtypes = [c.c_void_p, c.c_int32, c.c_void_p]
core.rgl_graph_destroy.argtypes = [c.c_void_p]

rays, ranges, trace, result = [c.c_void_p() for _ in range(4)]
matrix = (c.c_float * 12)(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
limits = (c.c_float * 2)(0.1, 40.0)
fields = (c.c_int32 * 1)(4)  # RGL_FIELD_IS_HIT_I32 in the pinned core.h
check(core.rgl_node_rays_from_mat3x4f(c.byref(rays), matrix, 1))
check(core.rgl_node_rays_set_range(c.byref(ranges), limits, 1))
check(core.rgl_node_raytrace(c.byref(trace), None))
check(core.rgl_node_points_yield(c.byref(result), fields, 1))
for parent, child in ((rays, ranges), (ranges, trace), (trace, result)):
    check(core.rgl_graph_node_add_child(parent, child))
check(core.rgl_graph_run(result))
count, size = c.c_int32(), c.c_int32()
check(core.rgl_graph_get_result_size(result, 4, c.byref(count), c.byref(size)))
assert (count.value, size.value) == (1, 4)
hit = c.c_int32(-1)
check(core.rgl_graph_get_result_data(result, 4, c.byref(hit)))
assert hit.value == 0  # An empty scene has no hit.
check(core.rgl_graph_destroy(result))
check(core.rgl_cleanup())
print(json.dumps({"core_version": "0.21.0", "shared_library_load": True, "gpu_raytrace": True}))
'''


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def download(path: Path, url: str, expected_hash: str) -> None:
    if not path.is_file():
        print(f"Downloading {url}", flush=True)
        temporary = path.with_suffix(path.suffix + ".partial")
        request = Request(url, headers={"User-Agent": "agri-mid360-rgl-setup/1"})
        with urlopen(request, timeout=60) as response, temporary.open("wb") as output:
            shutil.copyfileobj(response, output)
        if sha256(temporary) != expected_hash:
            raise RuntimeError(f"SHA-256 mismatch for {url}; kept {temporary} for inspection")
        temporary.replace(path)
    if sha256(path) != expected_hash:
        raise RuntimeError(f"SHA-256 mismatch for cached download {path}")


def prepare_source(downloads: Path, source: Path, core: Path, install: Path) -> Path:
    """Extract fixed build inputs and retain upstream license files."""
    source.mkdir(parents=True, exist_ok=True)
    archive_root = f"RGLGazeboPlugin-{PLUGIN_COMMIT}"
    with tarfile.open(downloads / f"{archive_root}.tar.gz", "r:gz") as archive:
        for member in archive:
            relative = PurePosixPath(member.name).relative_to(archive_root)
            if not member.isfile() or not relative.parts or ".." in relative.parts:
                continue
            if relative.parts[0] in {"docs", "test_world", ".git"}:
                continue
            if relative.parts[0] == "lidar_patterns" and relative.name != "LivoxMid360.mat3x4f":
                continue
            target = source.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.extractfile(member) as input_stream, target.open("wb") as output:
                shutil.copyfileobj(input_stream, output)

    library_dir = core / "lib"
    headers_dir = core / "include" / "rgl" / "api"
    library_dir.mkdir(parents=True, exist_ok=True)
    headers_dir.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(downloads / "RGL-core-linux-x64-v0.21.0.zip") as archive:
        with archive.open("libRobotecGPULidar.so") as input_stream:
            with (library_dir / "libRobotecGPULidar.so").open("wb") as output:
                shutil.copyfileobj(input_stream, output)
    shutil.copyfile(downloads / "core-v0.21.0.h", headers_dir / "core.h")

    licenses = install / "share" / "agri_mid360_rgl" / "licenses"
    licenses.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / "LICENSE", licenses / "RGLGazeboPlugin.LICENSE")
    shutil.copyfile(downloads / "RobotecGPULidar-v0.21.0.LICENSE", licenses / "RobotecGPULidar.LICENSE")
    return source / "lidar_patterns"


def validate_pattern(patterns: Path) -> dict:
    pattern = patterns / "LivoxMid360.mat3x4f"
    payload = pattern.read_bytes()
    # Git blob hash proves this is the exact upstream preset, including all
    # 40 alternating scans. The fixed archive hash also protects this file.
    git_hash = hashlib.sha1(f"blob {len(payload)}\0".encode() + payload).hexdigest()
    if git_hash != PATTERN_GIT_BLOB or len(payload) != 38400000:
        raise RuntimeError(f"Unexpected Livox Mid360 preset: {pattern}")
    return {
        "name": "Livox Mid360",
        "git_blob": git_hash,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "scan_count": 40,
        "rays_per_scan": len(payload) // 48 // 40,
        "update_rate_hz": 10,
        "sdf": "<pattern_preset>Livox Mid360</pattern_preset>",
    }


def environment(ros_prefix: Path, install: Path) -> dict[str, str]:
    env = os.environ.copy()
    prefixes = [ros_prefix] + sorted(path for path in (ros_prefix / "opt").glob("*") if path.is_dir())
    env["CMAKE_PREFIX_PATH"] = ":".join(filter(None, [*(str(path) for path in prefixes), env.get("CMAKE_PREFIX_PATH", "")]))
    libraries = [install / "RGLServerPlugin"] + [path / "lib" for path in prefixes]
    env["LD_LIBRARY_PATH"] = ":".join(filter(None, [*(str(path) for path in libraries), env.get("LD_LIBRARY_PATH", "")]))
    return env


def run_logged(command: list[str], env: dict[str, str], log_path: Path) -> None:
    print("Running:", " ".join(command), flush=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as output:
        process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in process.stdout:
            output.write(line)
            print(line, end="", flush=True)
        if process.wait():
            raise RuntimeError(f"Command failed; see {log_path}")


def verify_install(install: Path, env: dict[str, str], skip_gpu_check: bool) -> dict:
    for name in ("libRobotecGPULidar.so", "libRGLServerPluginManager.so", "libRGLServerPluginInstance.so"):
        library = install / "RGLServerPlugin" / name
        if not library.is_file():
            raise RuntimeError(f"Missing installed library: {library}")
        result = subprocess.run(["ldd", str(library)], env=env, capture_output=True, text=True, check=True)
        if "not found" in result.stdout:
            raise RuntimeError(f"Unresolved dependencies for {library}:\n{result.stdout}")
    if skip_gpu_check:
        return {"gpu_raytrace": False, "gpu_check_skipped": True}
    result = subprocess.run([sys.executable, "-c", PROBE, str(install)], env=env, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"RGL shared-library / CUDA / OptiX probe failed:\n{result.stdout}\n{result.stderr}")
    return json.loads(result.stdout.splitlines()[-1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path, default=PROJECT_ROOT / ".cache" / "rgl")
    parser.add_argument("--ros-prefix", type=Path, default=Path("/opt/ros/jazzy"))
    parser.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1))
    parser.add_argument("--verify-only", action="store_true", help="Check the existing install without downloads or compilation")
    parser.add_argument("--skip-gpu-check", action="store_true", help="Build on a machine without a GPU; does not report the backend runnable")
    args = parser.parse_args()
    cache = args.cache_dir.resolve()
    if not cache.is_relative_to(PROJECT_ROOT) or cache == PROJECT_ROOT:
        parser.error("--cache-dir must stay inside Agricultural_Bot")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    source = cache / "source" / "RGLGazeboPlugin"
    core = cache / "core"
    install = cache / "install"
    patterns = source / "lidar_patterns"
    env = environment(args.ros_prefix.resolve(), install)

    if not args.verify_only:
        for name in ("cmake", "make", "g++", "ldd"):
            if not shutil.which(name):
                raise RuntimeError(f"Required tool not found: {name}")
        downloads = cache / "downloads"
        downloads.mkdir(parents=True, exist_ok=True)
        for filename, (url, digest) in DOWNLOADS.items():
            download(downloads / filename, url, digest)
        patterns = prepare_source(downloads, source, core, install)
        build = cache / "build"
        run_logged([
            "cmake", "-S", str(source), "-B", str(build), "-G", "Unix Makefiles",
            "-DCMAKE_BUILD_TYPE=Release", f"-DCMAKE_INSTALL_PREFIX={install}",
            f"-DRGL_CUSTOM_LIBRARY_PATH={core / 'lib' / 'libRobotecGPULidar.so'}",
            f"-DRGL_CUSTOM_API_HEADER_PATH={core / 'include'}",
        ], env, cache / "logs" / "configure.log")
        run_logged(["cmake", "--build", str(build), "--parallel", str(args.jobs)], env, cache / "logs" / "build.log")
        run_logged(["cmake", "--install", str(build)], env, cache / "logs" / "install.log")

    pattern_info = validate_pattern(patterns)
    verification = verify_install(install, env, args.skip_gpu_check)
    manifest = {
        "plugin_repository": "https://github.com/RobotecAI/RGLGazeboPlugin",
        "plugin_commit": PLUGIN_COMMIT,
        "plugin_package_version": "0.2.0",
        "core_version": CORE_VERSION,
        "core_commit": CORE_COMMIT,
        "ros_prefix": str(args.ros_prefix.resolve()),
        "install_prefix": str(install),
        "system_plugin_path": str(install / "RGLServerPlugin"),
        "gui_plugin_path": str(install / "RGLVisualize"),
        "patterns_dir": str(patterns),
        "licenses_dir": str(install / "share" / "agri_mid360_rgl" / "licenses"),
        "downloads": {name: {"url": url, "sha256": digest} for name, (url, digest) in DOWNLOADS.items()},
        "pattern": pattern_info,
        "verification": verification,
        "libraries": {path.name: sha256(path) for path in (install / "RGLServerPlugin").glob("*.so")},
    }
    (cache / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    print("Launch with " + shlex.quote("rgl_install_prefix:=" + str(install)))
    print("Launch with " + shlex.quote("rgl_patterns_dir:=" + str(patterns)))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"RGL setup failed: {error}", file=sys.stderr)
        raise SystemExit(1)
