#!/bin/bash
set -e

if [ ! -f .env_vars ]; then
    echo "Error: .env_vars not found"
    echo "Run setup.sh first to configure"
    exit 1
fi

source .env_vars

echo "Rebuilding image: $CONTAINER_NAME:latest"
docker compose build --build-arg WS_NAME="$WORKSPACE_NAME" --build-arg CONTAINER_NAME="$CONTAINER_NAME"

echo "Done! Image rebuilt."