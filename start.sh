#!/bin/bash
set -e

if [ ! -f .env_vars ]; then
    echo "Error: .env_vars not found"
    echo "Run setup.sh first to configure"
    exit 1
fi

source .env_vars

echo "Starting container: $CONTAINER_NAME-dev"
docker compose up -d

echo "Done! Container running."
echo "Run: docker exec -it $CONTAINER_NAME-dev bash"