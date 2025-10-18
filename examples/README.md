# Examples

Example code, data, and integrations for Active Wing.

## Contents

### Data (examples/data/)
Example telemetry datasets for testing and development:
- Sample CSV exports
- Binary packet captures
- Reference runs (track, street, etc.)

### Integration Examples (coming soon)
- **dashboard/** - Web dashboard example
- **logger/** - SD card logging example
- **can-bus/** - OBD2/CAN integration
- **mobile/** - Mobile app integration
- **analysis/** - Data analysis scripts

## Using Example Data

Example datasets help you:
- Test your visualization without hardware
- Develop analysis algorithms
- Learn typical data patterns
- Compare different scenarios

## Contributing Examples

Want to contribute an example?

### Code Examples
1. Create a directory for your example
2. Include a README with:
   - What it demonstrates
   - Dependencies required
   - How to run it
   - Expected output
3. Keep it simple and well-commented

### Data Examples
1. Anonymize GPS coordinates if needed
2. Include metadata:
   - Date/time of recording
   - Vehicle type
   - Scenario (track, street, etc.)
   - Duration and conditions
3. Provide multiple formats (binary + CSV)
4. Keep file sizes reasonable (<10 MB)

See [CONTRIBUTING.md](../CONTRIBUTING.md) for submission guidelines.

## Example Scenarios

Useful reference datasets to contribute:

- **Highway cruise** - Steady speed, minimal dynamics
- **City driving** - Stop-and-go, turns, traffic
- **Track day** - High g-forces, consistent laps
- **Autocross** - Tight turns, acceleration zones
- **Parking lot** - Low speed, GPS challenges
- **Stationary calibration** - For testing ZUPT
- **GPS dropout** - Tunnel or parking garage

## License

Example code and data should be under a permissive license (MIT, CC0, or CC BY).

Specify the license in each example's README.
