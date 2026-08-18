#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

TORCH_WHEEL_NAME="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
TORCH_WHEEL_URL="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/$TORCH_WHEEL_NAME"
TORCH_WHEEL_SIZE=806950107
TORCH_WHEEL_SHA256="6f75fd2d2ef840ede1a90dbcf40a5458214bee26cc803fa510cda2e8978d972a"
CUSPARSELT_ARCHIVE_NAME="libcusparse_lt-linux-sbsa-0.6.2.3-archive.tar.xz"
CUSPARSELT_URL="https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-sbsa/$CUSPARSELT_ARCHIVE_NAME"
CUSPARSELT_ARCHIVE_SIZE=99872276
CUSPARSELT_ARCHIVE_SHA256="512faabdf1a095796ba113a8f81921303dfa203b3ae02345fd114407e9792ad7"
CACHE_DIR="$NAVDP_ROOT/.cache/jetson"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This setup targets Jetson aarch64; detected $(uname -m)." >&2
  exit 1
fi

mkdir -p "$CACHE_DIR"
if [[ ! -x "$NAVDP_VENV/bin/python" ]]; then
  echo "Creating $NAVDP_VENV with ROS system packages visible"
  python3 -m venv --system-site-packages "$NAVDP_VENV"
fi

navdp_source_ros
navdp_activate_venv
python -m pip install --upgrade pip 'setuptools<80' wheel

download_exact_size() {
  local url="$1"
  local output="$2"
  local expected_size="$3"
  local expected_sha256="$4"
  local current_size=0
  if [[ -f "$output" ]]; then
    current_size="$(stat -c %s "$output")"
  fi
  if [[ "$current_size" != "$expected_size" ]]; then
    echo "Downloading $(basename "$output")"
    curl -L --fail --retry 5 --retry-delay 2 --continue-at - "$url" -o "$output"
  fi
  current_size="$(stat -c %s "$output")"
  if [[ "$current_size" != "$expected_size" ]]; then
    echo "Unexpected file size for $output: $current_size (expected $expected_size)" >&2
    exit 1
  fi
  echo "$expected_sha256  $output" | sha256sum --check --status || {
    echo "SHA256 verification failed for $output" >&2
    exit 1
  }
}

CUSPARSELT_ARCHIVE="$CACHE_DIR/$CUSPARSELT_ARCHIVE_NAME"
if ! find "$NAVDP_CUSPARSELT_DIR/lib" -maxdepth 1 -name 'libcusparseLt.so*' -print -quit 2>/dev/null | grep -q .; then
  download_exact_size \
    "$CUSPARSELT_URL" "$CUSPARSELT_ARCHIVE" \
    "$CUSPARSELT_ARCHIVE_SIZE" "$CUSPARSELT_ARCHIVE_SHA256"
  mkdir -p "$NAVDP_CUSPARSELT_DIR"
  tar -xJf "$CUSPARSELT_ARCHIVE" -C "$NAVDP_CUSPARSELT_DIR" --strip-components=1
fi
export LD_LIBRARY_PATH="$NAVDP_CUSPARSELT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

TORCH_WHEEL="$CACHE_DIR/$TORCH_WHEEL_NAME"
if ! python -c 'import torch; assert torch.__version__.startswith("2.5.0a0")' >/dev/null 2>&1; then
  download_exact_size \
    "$TORCH_WHEEL_URL" "$TORCH_WHEEL" \
    "$TORCH_WHEEL_SIZE" "$TORCH_WHEEL_SHA256"
  python -m pip install --no-cache-dir "$TORCH_WHEEL"
fi

python -m pip install --no-cache-dir -r "$NAVDP_GO2_DIR/requirements-jetson.txt"

python - <<'PY'
import cv2
import diffusers
import flask
import message_filters
import numpy
import rclpy
import torch

print(f"torch={torch.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"cuda={torch.version.cuda}")
print(f"numpy={numpy.__version__} cv2={cv2.__version__} diffusers={diffusers.__version__}")
print(f"device={torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none'}")
if not torch.cuda.is_available():
    raise SystemExit("PyTorch installed, but CUDA is unavailable")
PY

echo "Jetson NavDP environment is ready: $NAVDP_VENV"
