# Release Process

Quick guide for creating Blackbox releases.

## Prerequisites

Install required tools:
```bash
# Install espflash if not already installed
cargo install espflash

# Install GitHub CLI (optional, for command-line releases)
# Ubuntu/Debian:
sudo apt install gh
# macOS:
brew install gh

# Authenticate with GitHub
gh auth login
```

## Release Steps

### 1. Build the Release Binary

```bash
# Run the release script
./scripts/release.sh v0.0.1
```

This will:
- Build optimized firmware
- Generate flashable `.bin` file with bootloader
- Show binary size
- Print next steps

### 2. Test the Binary (Recommended)

```bash
# Flash to your ESP32-C3 for testing
espflash write-bin 0x0 sensors/blackbox/blackbox-v0.0.1.bin

# Or monitor while flashing
espflash write-bin 0x0 sensors/blackbox/blackbox-v0.0.1.bin --monitor
```

Verify:
- ✅ Device boots and shows LED sequence
- ✅ WiFi AP "Blackbox" appears
- ✅ Dashboard loads at 192.168.4.1
- ✅ Telemetry updates at ~20Hz
- ✅ Settings save correctly

### 3. Commit and Push

```bash
# Add the release script and notes
git add scripts/release.sh release-notes.md RELEASE.md

# Commit
git commit -m "Add release tooling for v0.0.1"

# Push to PR branch
git push origin ap_enabled
```

### 4. Merge PR on GitHub

Go to your PR and click "Merge pull request"

### 5. Create Git Tag

```bash
# Switch to main and pull
git checkout main
git pull origin main

# Create annotated tag
git tag -a v0.0.1 -m "Release v0.0.1 - Access Point Mode + Mobile Dashboard"

# Push tag
git push origin v0.0.1
```

### 6. Create GitHub Release

**Option A: Using GitHub CLI (Recommended)**
```bash
gh release create v0.0.1 \
  --title "v0.0.1 - Access Point Mode + Mobile Dashboard" \
  --notes-file release-notes.md \
  sensors/blackbox/blackbox-v0.0.1.bin
```

**Option B: Using GitHub Web UI**
1. Go to https://github.com/jctoledo/blackbox/releases/new
2. Choose tag: `v0.0.1`
3. Title: `v0.0.1 - Access Point Mode + Mobile Dashboard`
4. Description: Copy from `release-notes.md`
5. Attach file: `sensors/blackbox/blackbox-v0.0.1.bin`
6. Click "Publish release"

### 7. Verify Release

Check that:
- ✅ Release appears at https://github.com/jctoledo/blackbox/releases
- ✅ Binary is downloadable
- ✅ Release notes are formatted correctly
- ✅ Tag is visible in repository

## Release Checklist

Before releasing:
- [ ] All tests pass (`cargo test`)
- [ ] No clippy warnings (`cargo clippy -- -D warnings`)
- [ ] Code is formatted (`cargo fmt`)
- [ ] CI checks pass on GitHub
- [ ] Binary tested on actual hardware
- [ ] Dashboard loads and works
- [ ] Settings persist correctly
- [ ] Documentation is up to date
- [ ] CHANGELOG.md updated (if exists)

## Version Numbering

We use [Semantic Versioning](https://semver.org/):
- `v0.0.x` - Pre-release, experimental
- `v0.x.0` - Minor feature additions
- `v1.0.0` - First stable release
- `vX.Y.Z` - Major.Minor.Patch

Examples:
- `v0.0.1` - First pre-release
- `v0.0.2` - Bug fixes
- `v0.1.0` - New features added
- `v1.0.0` - Production-ready

## Troubleshooting

**Binary too large:**
```bash
# Check size
ls -lh sensors/blackbox/blackbox-v*.bin

# ESP32-C3 has 4MB flash, binary should be < 2MB
```

**espflash not found:**
```bash
cargo install espflash --force
```

**gh command not found:**
Install GitHub CLI or use the web UI instead.

**Permission denied on release.sh:**
```bash
chmod +x scripts/release.sh
```

## Next Release

For the next release, just increment the version:
```bash
./scripts/release.sh v0.0.2
# ... follow the same steps
```
