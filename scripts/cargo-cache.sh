# Source from a trusted Blackbox Actions job before running Cargo:
#   source "$GITHUB_WORKSPACE/scripts/cargo-cache.sh" stable host
#   source "$GITHUB_WORKSPACE/scripts/cargo-cache.sh" nightly-2025-10-18 firmware
# Firmware output stays in the checkout: ESP-IDF derives app version metadata
# by finding Git above OUT_DIR. Only host artifacts and downloaded SDK/tools
# persist across checkout cleaning. One dedicated runner serializes jobs.

blackbox_cargo_cache() {
  local toolchain="${1:?Rust toolchain required}" mode="${2:?host or firmware required}"
  local keys build_key tools_key cache_root config_root
  case "$mode" in host|firmware) ;; *) echo 'Expected host or firmware' >&2; return 1 ;; esac
  keys=$(python3 - "$toolchain" "$mode" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

toolchain, mode = sys.argv[1:]
root = Path(os.environ["GITHUB_WORKSPACE"]).resolve()
def output(*args):
    """Read a tool identity or tracked-file list from the workspace."""
    return subprocess.check_output(args, cwd=root).decode().strip()

# Hash configuration, not source revisions: Cargo tracks host source changes.
base = {
    "schema": 1,
    "files": {},
}
for name in output("git", "ls-files", "-z").split("\0"):
    path = Path(name)
    if name and (path.name in {"Cargo.toml", "Cargo.lock", "partitions.csv"}
                 or path.name.startswith(("sdkconfig", "rust-toolchain"))
                 or ".cargo" in path.parts):
        base["files"][name] = hashlib.sha256((root / path).read_bytes()).hexdigest()

def digest(value):
    """Hash cache inputs without exposing environment values in logs."""
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()[:24]

ignored = {"CARGO_TARGET_DIR", "CARGO_WORKSPACE_DIR", "ESP_IDF_TOOLS_INSTALL_DIR",
           "CARGO_MAKEFLAGS", "CARGO_HOME", "RUSTUP_HOME", "RUSTUP_TOOLCHAIN"}
environment = {key: value for key, value in os.environ.items()
               if key not in ignored and (key.startswith(("CARGO_", "RUST"))
                   or key in {"LIBCLANG_PATH",
                              "CC", "CXX", "CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS",
                              "LD", "AR", "GCC_EXEC_PREFIX", "COMPILER_PATH",
                              "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH"})}
# Downloaded SDK/tools are independent of Rust and GPS variants. Explicit IDF
# overrides still select separate installations.
tools_key = digest({"schema": 1, "files": base["files"],
                    "env": {key: value for key, value in os.environ.items()
                            if key != "ESP_IDF_TOOLS_INSTALL_DIR"
                            and (key.startswith(("ESP_IDF_", "IDF_")) or key == "MCU")}})
host_key = "checkout-local"
if mode == "host":
    # Include exact compiler identity even for a moving "stable" channel.
    host_key = digest({"base": base, "workspace": str(root), "env": environment,
                       "rustc": output("rustc", "+" + toolchain, "-Vv"),
                       "cargo": output("cargo", "+" + toolchain, "-V")})
print(host_key, tools_key)
PY
  ) || return
  read -r build_key tools_key <<< "$keys"
  cache_root="$HOME/.cache/blackbox-build-v1"
  config_root="$cache_root/config/$tools_key"
  if [ "$mode" = host ]; then
    export CARGO_TARGET_DIR="$config_root/target/host/$build_key"
  else
    # Reset the host cache inherited from an earlier CI step. Keeping OUT_DIR
    # beneath this checkout preserves ESP-IDF's existing git-describe version.
    export CARGO_TARGET_DIR="$GITHUB_WORKSPACE/target"
  fi
  # Make workspace defaults explicit, including when the host target is external.
  export CARGO_WORKSPACE_DIR="$GITHUB_WORKSPACE"
  export ESP_IDF_TOOLS_INSTALL_DIR="custom:$config_root/esp-tools"
  mkdir -p "$CARGO_TARGET_DIR" "$config_root/esp-tools" || return
  # Only this namespace is pruned, with no concurrent jobs on this runner.
  # Keep current and previous SDK/configuration groups and two host targets.
  python3 - "$cache_root" "$mode" "$build_key" "$tools_key" <<'PY'
from pathlib import Path
import re
import shutil
import sys

root, mode, build_key, tools_key = sys.argv[1:]
config = Path(root) / "config" / tools_key
groups = [(Path(root) / "config", tools_key, 2)]
if mode == "host":
    groups.append((config / "target" / "host", build_key, 2))
for parent, current, keep in groups:
    (parent / current).touch()
    entries = sorted((path for path in parent.iterdir()
                      if re.fullmatch(r"[0-9a-f]{24}", path.name)
                      and path.is_dir() and not path.is_symlink()),
                     key=lambda path: path.stat().st_mtime, reverse=True)
    for path in entries[keep:]:
        if path.name != current:
            shutil.rmtree(path)
PY
  if [ "$?" -ne 0 ]; then return 1; fi
  # Later steps (image packaging / artifact upload) use this exact target path.
  if [ -n "${GITHUB_ENV:-}" ]; then
    printf '%s\n' "CARGO_TARGET_DIR=$CARGO_TARGET_DIR" \
      "CARGO_WORKSPACE_DIR=$CARGO_WORKSPACE_DIR" \
      "ESP_IDF_TOOLS_INSTALL_DIR=$ESP_IDF_TOOLS_INSTALL_DIR" >> "$GITHUB_ENV" || return
  fi
  printf 'Blackbox Cargo cache: %s (%s)\n' "$mode" "$build_key"
}

blackbox_cargo_cache "$@" || return "$?"
unset -f blackbox_cargo_cache
