#!/bin/bash
set -e

VERSION=$1

if [ -z "$VERSION" ]; then
    echo "Usage: ./scripts/release.sh v0.0.1"
    exit 1
fi

# Validate version format
if [[ ! $VERSION =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Error: Version must be in format vX.Y.Z (e.g., v0.0.1)"
    exit 1
fi

echo "======================================"
echo "Building Blackbox Release $VERSION"
echo "======================================"
echo ""

# Navigate to firmware directory
cd sensors/blackbox

# Build optimized firmware
echo "📦 Building optimized firmware..."
cargo build --release

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"
echo ""

# Generate flashable binary with bootloader
echo "🔨 Generating flashable binary (with bootloader)..."
cargo espflash save-image \
    --chip esp32c3 \
    --release \
    --merge \
    "blackbox-${VERSION}.bin"

if [ $? -ne 0 ]; then
    echo "❌ Binary generation failed!"
    exit 1
fi

echo "✅ Binary created: sensors/blackbox/blackbox-${VERSION}.bin"
echo ""

# Get binary size
BIN_SIZE=$(ls -lh "blackbox-${VERSION}.bin" | awk '{print $5}')
echo "📊 Binary size: $BIN_SIZE"
echo ""

echo "======================================"
echo "Release build complete!"
echo "======================================"
echo ""
echo "Binary location: sensors/blackbox/blackbox-${VERSION}.bin"
echo ""
echo "Next steps:"
echo "1. Test the binary:"
echo "   espflash write-bin 0x0 sensors/blackbox/blackbox-${VERSION}.bin"
echo ""
echo "2. Commit any remaining changes:"
echo "   git add ."
echo "   git commit -m 'Prepare release ${VERSION}'"
echo "   git push origin ap_enabled"
echo ""
echo "3. Merge your PR on GitHub"
echo ""
echo "4. Create and push the tag:"
echo "   git checkout main"
echo "   git pull origin main"
echo "   git tag -a ${VERSION} -m 'Release ${VERSION} - Access Point Mode + Mobile Dashboard'"
echo "   git push origin ${VERSION}"
echo ""
echo "5. Create GitHub release with binary:"
echo "   gh release create ${VERSION} \\"
echo "     --title '${VERSION} - Access Point Mode + Mobile Dashboard' \\"
echo "     --notes-file release-notes.md \\"
echo "     sensors/blackbox/blackbox-${VERSION}.bin"
echo ""
echo "Or use the web UI: https://github.com/jctoledo/blackbox/releases/new?tag=${VERSION}"
