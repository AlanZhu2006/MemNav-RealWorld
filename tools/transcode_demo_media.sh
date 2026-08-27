#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  transcode_demo_media.sh INPUT OUTPUT WIDTH HEIGHT [BITRATE_KBPS] [FPS]

Creates a browser-ready, video-only H.264 MP4 with fast-start metadata.
INPUT must be an MP4 containing H.264 or H.265 video. OUTPUT must not exist.
EOF
}

[[ $# -ge 4 && $# -le 6 ]] || { usage >&2; exit 2; }
input="$1"
output="$2"
width="$3"
height="$4"
bitrate="${5:-1800}"
fps="${6:-12}"

[[ -s "$input" ]] || { echo "input is missing or empty: $input" >&2; exit 1; }
[[ ! -e "$output" ]] || { echo "refusing to overwrite: $output" >&2; exit 1; }
for value in "$width" "$height" "$bitrate" "$fps"; do
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || { echo "numeric arguments must be positive integers" >&2; exit 2; }
done
(( width % 2 == 0 && height % 2 == 0 )) \
  || { echo "H.264 output dimensions must be even" >&2; exit 2; }
for command in gst-launch-1.0 gst-discoverer-1.0 gst-inspect-1.0; do
  command -v "$command" >/dev/null 2>&1 || { echo "missing command: $command" >&2; exit 1; }
done

description="$(gst-discoverer-1.0 "$input" 2>&1)"
parser=""
decoder=""
if grep -q 'video #[0-9][0-9]*: H.265' <<<"$description"; then
  parser=h265parse
  decoder=avdec_h265
elif grep -q 'video #[0-9][0-9]*: H.264' <<<"$description"; then
  parser=h264parse
  decoder=avdec_h264
else
  echo "input must contain H.264 or H.265 video" >&2
  exit 1
fi
gst-inspect-1.0 "$parser" >/dev/null
gst-inspect-1.0 "$decoder" >/dev/null
mkdir -p "$(dirname "$output")"
temporary="$(dirname "$output")/.${output##*/}.$$"
trap 'rm -f "$temporary"' EXIT

gst-launch-1.0 -q -e \
  filesrc location="$input" ! qtdemux name=demux \
  demux.video_0 ! queue ! "$parser" ! "$decoder" ! \
  videoconvert ! videoscale ! videorate ! \
  "video/x-raw,width=$width,height=$height,framerate=$fps/1,pixel-aspect-ratio=1/1,format=I420" ! \
  x264enc bitrate="$bitrate" speed-preset=veryfast key-int-max="$((fps * 2))" ! \
  h264parse config-interval=-1 ! mp4mux faststart=true ! filesink location="$temporary"

[[ -s "$temporary" ]] || { echo "transcoder produced no output" >&2; exit 1; }
mv "$temporary" "$output"
trap - EXIT
echo "$output"
