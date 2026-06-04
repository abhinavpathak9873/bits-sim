#!/bin/bash
set -e

if [ ! -f .env_vars ]; then
    echo "Error: .env_vars not found"
    echo "Run setup.sh first to configure"
    exit 1
fi

source .env_vars

echo "Stopping container: $CONTAINER_NAME-dev"
docker compose down

echo "Done!"