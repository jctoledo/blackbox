/// Coordinate frame transformations
const G: f32 = 9.80665; // m/s²
const METERS_PER_DEGREE_LAT: f64 = 111320.0; // Approximate meters per degree latitude

/// Transform body-frame acceleration to earth frame (ENU)
///
/// # Arguments
/// * `ax_body` - X acceleration in body frame (m/s²)
/// * `ay_body` - Y acceleration in body frame (m/s²)
/// * `az_body` - Z acceleration in body frame (m/s²)
/// * `roll_deg` - Roll angle in degrees
/// * `pitch_deg` - Pitch angle in degrees
/// * `yaw_rad` - Yaw angle in radians
///
/// # Returns
/// * `(ax_earth, ay_earth)` - Horizontal acceleration in earth frame (m/s²)
pub fn body_to_earth(
    ax_body: f32,
    ay_body: f32,
    az_body: f32,
    roll_deg: f32,
    pitch_deg: f32,
    yaw_rad: f32,
) -> (f32, f32) {
    let cr = (roll_deg.to_radians()).cos();
    let sr = (roll_deg.to_radians()).sin();
    let cp = (pitch_deg.to_radians()).cos();
    let sp = (pitch_deg.to_radians()).sin();
    let cy = yaw_rad.cos();
    let sy = yaw_rad.sin();

    // Rotation matrix multiplication
    let ax_earth =
        cy * cp * ax_body + (cy * sp * sr - sy * cr) * ay_body + (cy * sp * cr + sy * sr) * az_body;
    let ay_earth =
        sy * cp * ax_body + (sy * sp * sr + cy * cr) * ay_body + (sy * sp * cr - cy * sr) * az_body;

    (ax_earth, ay_earth)
}

/// Remove gravity from body-frame acceleration
///
/// # Arguments
/// * `ax_raw` - Raw X acceleration (m/s²)
/// * `ay_raw` - Raw Y acceleration (m/s²)
/// * `az_raw` - Raw Z acceleration (m/s²)
/// * `roll_deg` - Roll angle in degrees
/// * `pitch_deg` - Pitch angle in degrees
///
/// # Returns
/// * `(ax_nograv, ay_nograv, az_nograv)` - Acceleration with gravity removed
///   (m/s²)
pub fn remove_gravity(
    ax_raw: f32,
    ay_raw: f32,
    az_raw: f32,
    roll_deg: f32,
    pitch_deg: f32,
) -> (f32, f32, f32) {
    let cr = (roll_deg.to_radians()).cos();
    let sr = (roll_deg.to_radians()).sin();
    let cp = (pitch_deg.to_radians()).cos();
    let sp = (pitch_deg.to_radians()).sin();

    // Gravity vector in body frame
    let g_bx = -sp * G;
    let g_by = sr * cp * G;
    let g_bz = cr * cp * G;

    // Subtract gravity
    (ax_raw - g_bx, ay_raw - g_by, az_raw - g_bz)
}

/// Convert car-frame acceleration to longitudinal/lateral
///
/// # Arguments
/// * `ax_earth` - X acceleration in earth frame (m/s²)
/// * `ay_earth` - Y acceleration in earth frame (m/s²)
/// * `yaw_rad` - Car yaw angle in radians
///
/// # Returns
/// * `(a_lon, a_lat)` - Longitudinal and lateral acceleration (m/s²)
pub fn earth_to_car(ax_earth: f32, ay_earth: f32, yaw_rad: f32) -> (f32, f32) {
    let cos_yaw = yaw_rad.cos();
    let sin_yaw = yaw_rad.sin();

    let a_lon = cos_yaw * ax_earth + sin_yaw * ay_earth;
    let a_lat = -sin_yaw * ax_earth + cos_yaw * ay_earth;

    (a_lon, a_lat)
}

/// Convert latitude difference to meters
///
/// # Arguments
/// * `dlat` - Latitude difference in degrees
///
/// # Returns
/// * Distance in meters (north positive)
pub fn lat_to_meters(dlat: f64) -> f32 {
    (dlat * METERS_PER_DEGREE_LAT) as f32
}

/// Convert longitude difference to meters
///
/// # Arguments
/// * `dlon` - Longitude difference in degrees
/// * `ref_lat` - Reference latitude in degrees
///
/// # Returns
/// * Distance in meters (east positive)
pub fn lon_to_meters(dlon: f64, ref_lat: f64) -> f32 {
    (dlon * METERS_PER_DEGREE_LAT * ref_lat.to_radians().cos()) as f32
}

/// Convert GPS coordinates to local ENU frame
///
/// # Arguments
/// * `lat` - Current latitude (degrees)
/// * `lon` - Current longitude (degrees)
/// * `ref_lat` - Reference latitude (degrees)
/// * `ref_lon` - Reference longitude (degrees)
///
/// # Returns
/// * `(east, north)` - Position in meters relative to reference point
pub fn gps_to_local(lat: f64, lon: f64, ref_lat: f64, ref_lon: f64) -> (f32, f32) {
    let dlat = lat - ref_lat;
    let dlon = lon - ref_lon;

    let north = lat_to_meters(dlat);
    let east = lon_to_meters(dlon, ref_lat);

    (east, north)
}

/// Calculate distance between two GPS positions
///
/// # Arguments
/// * `lat1, lon1` - First position (degrees)
/// * `lat2, lon2` - Second position (degrees)
/// * `ref_lat` - Reference latitude for longitude scaling (degrees)
///
/// # Returns
/// * Distance in meters
pub fn gps_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64, ref_lat: f64) -> f32 {
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;

    let dy = lat_to_meters(dlat);
    let dx = lon_to_meters(dlon, ref_lat);

    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_body_to_earth_no_rotation() {
        // No rotation, should pass through
        let (ax, ay) = body_to_earth(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!((ax - 1.0).abs() < 0.001);
        assert!((ay - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_remove_gravity_level() {
        // Level sensor, gravity on Z axis
        let (ax, ay, az) = remove_gravity(0.0, 0.0, G, 0.0, 0.0);
        assert!(ax.abs() < 0.001);
        assert!(ay.abs() < 0.001);
        assert!(az.abs() < 0.001);
    }

    #[test]
    fn test_gps_to_local() {
        // Test small displacement (~1 meter north)
        let ref_lat = 37.0;
        let ref_lon = -122.0;
        let (east, north) = gps_to_local(
            ref_lat + 0.00001, // ~1.1m north
            ref_lon,
            ref_lat,
            ref_lon,
        );
        assert!(east.abs() < 0.1); // Should be near zero
        assert!((north - 1.11).abs() < 0.1); // Should be ~1.11m
    }
}
