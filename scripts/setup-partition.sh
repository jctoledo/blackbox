#!/bin/bash
#
# Setup script for custom partition table
# Run this after cloning the repo to configure the partition table path
#

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

PARTITIONS_FILE="$REPO_ROOT/partitions.csv"
SDKCONFIG_FILE="$REPO_ROOT/sdkconfig.defaults"

if [ ! -f "$PARTITIONS_FILE" ]; then
    echo "Error: partitions.csv not found at $PARTITIONS_FILE"
    exit 1
fi

if [ ! -f "$SDKCONFIG_FILE" ]; then
    echo "Error: sdkconfig.defaults not found at $SDKCONFIG_FILE"
    exit 1
fi

# Update the partition table path in sdkconfig.defaults
echo "Updating partition table path in sdkconfig.defaults..."
sed -i "s|CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=.*|CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"$PARTITIONS_FILE\"|" "$SDKCONFIG_FILE"

echo "Done! Partition table configured with path:"
echo "  $PARTITIONS_FILE"
echo ""
echo "You may need to clean the build cache before rebuilding:"
echo "  rm -rf target/riscv32imc-esp-espidf/release/build/esp-idf-sys-*"
echo "  cargo build --release"
