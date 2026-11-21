# Contributing to Blackbox

First off, thanks for taking the time to contribute! 🎉

Blackbox is a community-driven project, and we welcome contributions from everyone—whether you're a seasoned embedded developer, a car enthusiast learning to code, or somewhere in between.

---

## Code of Conduct

### Our Pledge

We pledge to make participation in the Blackbox community a harassment-free experience for everyone, regardless of age, body size, visible or invisible disability, ethnicity, sex characteristics, gender identity and expression, level of experience, education, socio-economic status, nationality, personal appearance, race, religion, or sexual identity and orientation.

### Our Standards

**Examples of behavior that contributes to a positive environment:**
- Being respectful of differing viewpoints and experiences
- Giving and accepting constructive feedback gracefully
- Showing empathy towards other community members
- Focusing on collaboration rather than competition
- Acknowledging contributions from all skill levels
- Being patient with beginners and those learning
- Sharing knowledge freely and enthusiastically

**Examples of unacceptable behavior:**
- Sexualized language/imagery or unwelcome sexual attention
- Trolling, insulting comments, or personal/political attacks
- Public or private harassment
- Publishing others' private information without permission
- Dismissing or attacking people based on their skill level
- Gatekeeping or elitism
- Other conduct inappropriate in a professional setting

### Enforcement

Instances of abusive, harassing, or otherwise unacceptable behavior may be reported to project maintainers. All complaints will be reviewed and investigated promptly and fairly.

Violations may result in:
1. **Correction** - Private warning
2. **Warning** - Warning with consequences for continued behavior
3. **Temporary Ban** - Temporary ban from community interaction
4. **Permanent Ban** - Permanent ban from the community

**The Bottom Line:** We're all here because we love cars, technology, and making cool stuff. Let's treat each other with respect and focus on building something awesome together. 🏁

---

## Table of Contents

- [Getting Started](#getting-started)
- [How Can I Contribute?](#how-can-i-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Features](#suggesting-features)
  - [Contributing Code](#contributing-code)
  - [Improving Documentation](#improving-documentation)
- [Development Setup](#development-setup)
- [Pull Request Process](#pull-request-process)
- [Style Guidelines](#style-guidelines)
- [Testing](#testing)
- [Community](#community)

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/blackbox.git
   cd blackbox
   ```
3. **Add upstream remote**:
   ```bash
   git remote add upstream https://github.com/jctoledo/blackbox.git
   ```
4. **Create a branch** for your work:
   ```bash
   git checkout -b feature/my-awesome-feature
   ```

## How Can I Contribute?

### Reporting Bugs

Before submitting a bug report:
- Check the [existing issues](https://github.com/jctoledo/blackbox/issues) to avoid duplicates
- Try the latest version from `main` branch
- Collect information about your setup

**Good bug reports include:**

- **Clear title** - Describe the problem concisely
- **Environment details**:
  - ESP32 variant (ESP32-C3, ESP32-S3, etc.)
  - IMU model (WT901, MPU6050, etc.)
  - GPS module (NEO-6M, etc.)
  - Firmware version or git commit hash
- **Steps to reproduce** - Detailed, numbered steps
- **Expected behavior** - What should happen
- **Actual behavior** - What actually happens
- **Logs/screenshots** - Error messages, serial output, LED patterns
- **Additional context** - Anything else that might help

### Suggesting Features

We love new ideas! Before suggesting a feature:

- Check if it's already been proposed in [issues](https://github.com/jctoledo/blackbox/issues)
- Consider if it fits the project's scope (vehicle telemetry/data acquisition)
- Think about how it benefits the community

**Good feature requests include:**

- **Use case** - Why is this feature needed?
- **Proposed solution** - How would it work?
- **Alternatives considered** - Other approaches you've thought about
- **Implementation ideas** - If you have technical suggestions

### Contributing Code

#### Priority Areas

We especially welcome contributions in these areas:

1. **Sensor Drivers** (in `drivers/`)
   - Additional IMU driver crates (MPU6050, BMI088, LSM6DSO)
   - GPS improvements to ublox-gps (RTK support, SBAS, additional NMEA sentences)
   - New driver crates (barometer, temperature, pressure sensors)
   - All drivers should be no-std compatible with zero dependencies

2. **Communication**
   - BLE streaming for mobile apps
   - LoRa for long-range telemetry
   - CAN bus integration for OBD2

3. **Data Processing**
   - Kalman filter improvements (UKF, adaptive tuning)
   - Additional driving modes (drifting, off-road)
   - Track mapping and lap detection

4. **Storage & Visualization**
   - SD card logging
   - Web-based dashboards
   - Mobile apps (iOS/Android)

5. **Hardware**
   - PCB designs for integrated boards
   - 3D-printable enclosures
   - Wiring harnesses

#### Before Starting

- **Check existing issues/PRs** - Someone might already be working on it
- **Open an issue** for major changes to discuss approach first
- **Keep scope focused** - Smaller PRs are easier to review and merge

### Improving Documentation

Documentation is just as important as code! Help us by:

- Fixing typos, broken links, or unclear explanations
- Adding examples and use cases
- Creating tutorials for common modifications
- Improving API documentation in code comments
- Translating docs to other languages

## Development Setup

### Maintained workflow

Use the root README for the ESP-IDF prerequisites. All project compilation, automated tests, documentation generation and media encoding run on lat. A maintainer reviews branch code and dispatches `.github/workflows/ci.yml` on `blackbox-lat`; fork pull requests must not automatically run on this persistent runner.

Host checks (repository root, on lat):

```bash
cargo +stable fmt --all -- --check
cargo +stable test --locked -p sensor-fusion -p wt901 -p ublox-gps -p blackbox-host-tests
cargo +stable check --locked -p ublox-gps --no-default-features
npm ci --ignore-scripts
node --test tests/*.test.cjs
python3 -m unittest discover -s tests -p 'test_*.py'
python3 scripts/embed_dashboard_fonts.py --check
```

Firmware checks (from `sensors/blackbox`, on lat):

```bash
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 clippy --release --locked -- -D warnings
GPS_MODEL=m9n GPS_RATE=25 cargo +nightly-2025-10-18 build --release --locked
GPS_MODEL=6m GPS_RATE=5 cargo +nightly-2025-10-18 build --release --locked
```

CI also validates Python tools, cross-language packet fixtures, both GPS variants with Clippy, generated guides and Pages packaging. Python tools live in `tools/python/`. The production UI is `sensors/blackbox/web/dashboard.html`; the demo is generated from it, not ported from the historical dashboard simulator.

## Pull Request Process

### Before Submitting

- [ ] Relevant host/firmware targets compile without warnings on lat
- [ ] Relevant host, browser-logic and protocol tests pass on lat
- [ ] Code is formatted (`cargo fmt`)
- [ ] Clippy linter passes (`cargo clippy`)
- [ ] Documentation updated (README, CLAUDE.md if needed)
- [ ] Commit messages are clear and descriptive

### Submitting Your PR

1. **Push your branch** to your fork:
   ```bash
   git push origin feature/my-awesome-feature
   ```

2. **Open a Pull Request** on GitHub with:
   - **Clear title** - What does this PR do?
   - **Description** - Why is this change needed?
   - **Testing done** - How did you verify it works?
   - **Related issues** - Link to any related issues (#123)

3. **Respond to feedback** - Maintainers may request changes
4. **Keep PR updated** - Rebase on latest `main` if needed

### What Happens Next

- A maintainer will review your PR (as maintainer availability allows)
- They may request changes or ask questions
- Once approved, your PR will be merged!
- Your contribution will be credited in release notes

## Style Guidelines

### Rust Code

- Follow standard Rust conventions (use `cargo fmt`)
- Use meaningful variable names
- Add comments for complex logic
- Keep functions focused and short
- Prefer `Result` over `panic!` for error handling

**Example:**
```rust
/// Calculate vehicle speed from velocity components
///
/// # Arguments
/// * `vx` - Velocity in X direction (m/s)
/// * `vy` - Velocity in Y direction (m/s)
///
/// # Returns
/// Speed magnitude in m/s
pub fn calculate_speed(vx: f32, vy: f32) -> f32 {
    (vx * vx + vy * vy).sqrt()
}
```

### Python Code

- Follow PEP 8 style guide
- Use type hints where appropriate
- Add docstrings for functions/classes
- Keep it simple and readable

**Example:**
```python
def decode_telemetry(payload: bytes) -> dict | None:
    """
    Decode binary telemetry packet.

    Args:
        payload: Raw packet bytes (82 bytes for current v4)

    Returns:
        Dictionary with decoded values, or None if invalid
    """
    if len(payload) != 82:
        return None
    # ... decode logic
```

### Commit Messages

Use descriptive commit messages:

- **Good**: `Add support for MPU6050 IMU sensor`
- **Good**: `Fix GPS parser crash on malformed NMEA sentences`
- **Bad**: `update code`
- **Bad**: `fix bug`

Format:
```
Short summary (50 chars or less)

More detailed explanation if needed. Wrap at 72 characters.
Explain the problem this commit solves and why you chose
this approach.

Fixes #123
```

## Testing

### Hardware Testing

If your changes affect hardware interaction:

1. Test on actual ESP32 hardware (not just compilation)
2. Verify with different sensor modules if changing drivers
3. Test edge cases (sensor disconnection, bad GPS fix, etc.)
4. Monitor serial output for errors

### Software Testing

For algorithm changes:

1. Add unit tests for new functions
2. Test with recorded sensor data if available
3. Verify numerical stability (no NaN/Inf values)
4. Check performance impact (timing, memory usage)

### Documentation Testing

- Ensure code examples compile and run
- Test installation instructions on fresh system
- Verify links aren't broken

## Community

### Communication Channels

- **GitHub Issues** - Bug reports, feature requests, discussions
- **Pull Requests** - Code review and collaboration
- (Future: Discord/Slack for real-time chat)

### Recognition

Contributors are recognized in:
- Release notes for each version
- README acknowledgments section
- Git history (your commits stay forever!)

### Questions?

Don't hesitate to ask questions:
- Check existing documentation:
  - [Driver crates](drivers/) - Current sensor parser implementations
  - [Framework guide](framework/README.md) - Current integration boundaries
  - [CLAUDE.md](CLAUDE.md) - Architecture overview
- Open a GitHub issue with the `question` label
- Comment on relevant PRs or issues

## Getting Help

Stuck on something? Here are resources:

- **[docs/FAQ.md](docs/FAQ.md)**: Common questions and troubleshooting
- **[CLAUDE.md](CLAUDE.md)**: Architecture overview of this project
- **ESP32 Rust**: [esp-rs documentation](https://esp-rs.github.io/book/)
- **Kalman Filters**: [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)

---

**Thank you for contributing to Blackbox!** Every contribution, no matter how small, makes this project better for everyone. 🏁
