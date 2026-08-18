#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

HF_REVISION="7cee38a8d8308874d2b8488783c612f42060ac41"
HF_ROOT="https://huggingface.co/InternRobotics/X-NavDP/resolve/$HF_REVISION"
X_CHECKPOINT="$NAVDP_ROOT/baselines/x-navdp/checkpoints/x-navdp_posttrain.ckpt"
BASE_CHECKPOINT="$NAVDP_ROOT/baselines/navdp/checkpoints/navdp_pretrain.ckpt"
X_SHA256="267089a81bbbe7a913debda6603f3f1b66a79520370ce953b2d888d793b89f24"
BASE_SHA256="3bb3ad4ab241e857bb57a4021cc6aab76d5263e81fbf80298d579053ef011947"

download_one() {
  local name="$1"
  local url="$2"
  local output="$3"
  local expected="$4"
  mkdir -p "$(dirname "$output")"

  if [[ -f "$output" ]] && echo "$expected  $output" | sha256sum --check --status; then
    echo "$name already verified: $output"
    return
  fi
  if [[ -f "$output" ]]; then
    local backup="${output}.invalid.$(date +%Y%m%d-%H%M%S)"
    echo "Existing $name checkpoint is invalid; moving it to $backup"
    mv "$output" "$backup"
  fi

  local temporary="${output}.download"
  echo "Downloading $name from pinned Hugging Face revision $HF_REVISION"
  curl -L --fail --retry 5 --retry-delay 2 "$url?download=true" -o "$temporary"
  echo "$expected  $temporary" | sha256sum --check
  mv "$temporary" "$output"
  echo "$name installed: $output"
}

selection="${1:-all}"
case "$selection" in
  x|x-navdp)
    download_one "X-NavDP" "$HF_ROOT/x-navdp_posttrain.ckpt" "$X_CHECKPOINT" "$X_SHA256"
    ;;
  base|navdp)
    download_one "NavDP" "$HF_ROOT/navdp_pretrain.ckpt" "$BASE_CHECKPOINT" "$BASE_SHA256"
    ;;
  all)
    download_one "X-NavDP" "$HF_ROOT/x-navdp_posttrain.ckpt" "$X_CHECKPOINT" "$X_SHA256"
    download_one "NavDP" "$HF_ROOT/navdp_pretrain.ckpt" "$BASE_CHECKPOINT" "$BASE_SHA256"
    ;;
  *)
    echo "Usage: $0 [all|x|base]" >&2
    exit 2
    ;;
esac
