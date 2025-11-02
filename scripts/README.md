# Build Scripts

## generate_flasher_config.py

Automatically generates `docs/projects-config.js` for the web flasher by scanning `sensors/` directory.

### Usage

```bash
# Generate configuration (outputs to stdout and writes to docs/projects-config.js)
python3 scripts/generate_flasher_config.py

# With custom version/repo
VERSION=v1.0.0 GITHUB_REPOSITORY=youruser/yourrepo python3 scripts/generate_flasher_config.py
```

The script automatically writes to `docs/projects-config.js` when run locally (unless `CI=true` is set).

### How it works

1. Scans all directories in `sensors/`
2. Looks for `project.json` in each directory
3. Validates required fields (name, id, description, hardware, software)
4. Generates JavaScript configuration with firmware URLs pointing to GitHub releases
5. Outputs JavaScript that can be loaded by `index.html`

### Adding a new sensor project

1. Create a new directory under `sensors/`, e.g., `sensors/my-sensor/`
2. Add a `project.json` file with this structure:

```json
{
  "name": "My Sensor Project",
  "id": "my-sensor",
  "description": "A description of what this sensor does...",
  "hardware": [
    "ESP32-S3 Development Board",
    "Some IMU sensor",
    "Other hardware requirements"
  ],
  "software": [
    "Chrome browser",
    "Windows, macOS, or Linux"
  ],
  "chip": "esp32s3",
  "target": "riscv32imac-esp-espidf"
}
```

3. Build your project (the CI will automatically discover it)
4. The web flasher will automatically include it in the dropdown

### CI Integration

The release workflow automatically:
1. Discovers all projects with `project.json` files
2. Builds firmware for each project
3. Generates `projects-config.js` with correct release URLs
4. Deploys the flasher to GitHub Pages

No manual updates needed!
