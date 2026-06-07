#!/usr/bin/env bash
set -euo pipefail

# Build using host networking to avoid iptables RAW table issues on some hosts
# (avoids: "Unable to enable DIRECT ACCESS FILTERING - DROP rule" during RUN)
cd "$(dirname "$0")"

DOCKER_BUILDKIT=1 docker build --no-cache --network=host -f Dockerfile.ros -t felix-ai-ros:latest .
