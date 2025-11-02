#!/usr/bin/env python3
"""
Generate flasher configuration from sensor projects.

This script scans the sensors/ directory for project.json files and generates
a JavaScript configuration file that the web flasher can use.
"""

import json
import sys
from pathlib import Path


def find_sensor_projects(sensors_dir: Path) -> list[dict]:
    """Find all sensor projects with project.json metadata."""
    projects = []

    if not sensors_dir.exists():
        print(f"Error: sensors directory not found: {sensors_dir}", file=sys.stderr)
        return projects

    for project_dir in sensors_dir.iterdir():
        if not project_dir.is_dir():
            continue

        project_json = project_dir / "project.json"
        if not project_json.exists():
            print(f"Warning: Skipping {project_dir.name} - no project.json found", file=sys.stderr)
            continue

        try:
            with open(project_json, 'r') as f:
                project_data = json.load(f)

            # Validate required fields
            required_fields = ['name', 'id', 'description', 'hardware', 'software']
            missing = [f for f in required_fields if f not in project_data]
            if missing:
                print(f"Warning: {project_dir.name}/project.json missing fields: {missing}", file=sys.stderr)
                continue

            projects.append(project_data)
            print(f"✓ Found project: {project_data['name']}", file=sys.stderr)

        except json.JSONDecodeError as e:
            print(f"Error: Failed to parse {project_json}: {e}", file=sys.stderr)
            continue

    return projects


def generate_flasher_js(projects: list[dict], repo: str, version: str) -> str:
    """Generate JavaScript configuration for the web flasher."""

    # Build projects object
    projects_js = []
    for project in projects:
        project_id = project['id']

        # Generate firmware URL based on GitHub release
        firmware_url = f"https://github.com/{repo}/releases/download/{version}/{project_id}.bin"

        project_js = f"""        '{project_id}': {{
            name: {json.dumps(project['name'])},
            description: {json.dumps(project['description'])},
            hardware: {json.dumps(project['hardware'])},
            software: {json.dumps(project['software'])},
            firmwareUrl: {json.dumps(firmware_url)},
            chip: {json.dumps(project.get('chip', 'esp32c3'))},
            target: {json.dumps(project.get('target', 'riscv32imc-esp-espidf'))}
        }}"""

        projects_js.append(project_js)

    projects_obj = ",\n".join(projects_js)

    return f"""// Auto-generated project configuration
// Generated from sensors/*/project.json
// DO NOT EDIT MANUALLY - your changes will be overwritten

const PROJECTS = {{
{projects_obj}
}};

// Export for use in index.html
if (typeof module !== 'undefined' && module.exports) {{
    module.exports = PROJECTS;
}}
"""


def main():
    # Determine repository root
    script_dir = Path(__file__).parent
    repo_root = script_dir.parent
    sensors_dir = repo_root / "sensors"
    output_file = repo_root / "docs" / "projects-config.js"

    # Get version and repo from environment or use defaults
    import os
    version = os.environ.get('VERSION', 'latest')
    repo = os.environ.get('GITHUB_REPOSITORY', 'jctoledo/active_wing')

    print(f"Generating flasher config for {repo} version {version}", file=sys.stderr)
    print(f"Scanning: {sensors_dir}", file=sys.stderr)

    # Find all sensor projects
    projects = find_sensor_projects(sensors_dir)

    if not projects:
        print("Error: No valid sensor projects found!", file=sys.stderr)
        sys.exit(1)

    print(f"\nFound {len(projects)} project(s)", file=sys.stderr)

    # Generate JavaScript configuration
    config_js = generate_flasher_js(projects, repo, version)

    # Write to stdout (can be redirected to file)
    print(config_js)

    # Also write to docs/projects-config.js if not in CI
    if os.environ.get('CI') != 'true':
        output_file.parent.mkdir(parents=True, exist_ok=True)
        output_file.write_text(config_js)
        print(f"\n✓ Also wrote to: {output_file}", file=sys.stderr)

    print(f"\n✓ Configuration generated successfully", file=sys.stderr)


if __name__ == '__main__':
    main()
