//! Lap timer with line crossing detection
//!
//! Detects when vehicle crosses timing lines and tracks lap times.
//! Designed to run at 200Hz for sub-sample precision crossing detection.
//!
//! ## Features
//!
//! - Millisecond-precision lap timing
//! - Support for loop tracks (single start/finish line) and point-to-point
//! - Direction validation to prevent backwards crossings
//! - Configurable debounce and minimum lap time
//! - Per-frame flags for telemetry integration
//!
//! ## Usage
//!
//! ```rust,ignore
//! use sensor_fusion::{LapTimer, TimingLine, CROSSED_START};
//! use std::f32::consts::FRAC_PI_2;
//!
//! let mut timer = LapTimer::new();
//!
//! // Configure a timing line crossing northward
//! timer.configure_loop(TimingLine {
//!     p1: (-10.0, 0.0),
//!     p2: (10.0, 0.0),
//!     direction: FRAC_PI_2,  // Crossing direction: north (+Y)
//! });
//!
//! // In main loop at 200Hz:
//! let flags = timer.update((x, y), (vx, vy), timestamp_ms, speed_mps);
//! if flags & CROSSED_START != 0 {
//!     // Crossed start line this frame
//! }
//! ```

use core::f32::consts::PI;

/// A timing line defined by two endpoints and a valid crossing direction
#[derive(Clone, Copy, Debug)]
pub struct TimingLine {
    /// First endpoint in local meters
    pub p1: (f32, f32),
    /// Second endpoint in local meters
    pub p2: (f32, f32),
    /// Valid crossing direction in radians (0 = +X, π/2 = +Y)
    pub direction: f32,
}

impl TimingLine {
    /// Create a new timing line
    pub fn new(p1: (f32, f32), p2: (f32, f32), direction: f32) -> Self {
        Self { p1, p2, direction }
    }
}

/// Track configuration
#[derive(Clone, Copy, Debug)]
pub enum TrackType {
    /// Single start/finish line (circuit racing)
    Loop { line: TimingLine },
    /// Separate start and finish lines (hill climb, rally stage)
    PointToPoint {
        start: TimingLine,
        finish: TimingLine,
    },
}

/// Lap timer state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LapTimerState {
    /// No track configured
    Idle,
    /// Track configured, waiting for first line crossing
    Armed,
    /// Timing active, counting lap time
    Timing,
}

// Lap flag bits for telemetry
/// No flags set
pub const FLAG_NONE: u8 = 0;
/// Crossed start line this frame
pub const CROSSED_START: u8 = 1 << 0;
/// Crossed finish line this frame
pub const CROSSED_FINISH: u8 = 1 << 1;
/// New lap completed this frame
pub const NEW_LAP: u8 = 1 << 2;
/// New best lap this frame
pub const NEW_BEST: u8 = 1 << 3;
/// Lap was invalid (too short, wrong direction)
pub const INVALID_LAP: u8 = 1 << 4;

/// Default debounce time between line crossings (milliseconds)
pub const DEFAULT_CROSSING_DEBOUNCE_MS: u32 = 500;

/// Default minimum valid lap time (milliseconds)
pub const DEFAULT_MIN_LAP_TIME_MS: u32 = 10_000;

/// Default direction tolerance (radians, ±90° from expected)
pub const DEFAULT_DIRECTION_TOLERANCE: f32 = PI / 2.0;

/// Mutable timing state (separate from track config to avoid borrow conflicts)
struct TimingState {
    state: LapTimerState,
    lap_start_ms: u32,
    current_lap_ms: u32,
    lap_count: u16,
    best_lap_ms: Option<u32>,
    frame_flags: u8,
    last_crossing_ms: u32,
    crossed_start_this_run: bool,
    crossing_debounce_ms: u32,
    min_lap_time_ms: u32,
    direction_tolerance: f32,
}

impl TimingState {
    fn new() -> Self {
        Self {
            state: LapTimerState::Idle,
            lap_start_ms: 0,
            current_lap_ms: 0,
            lap_count: 0,
            best_lap_ms: None,
            frame_flags: FLAG_NONE,
            last_crossing_ms: 0,
            crossed_start_this_run: false,
            crossing_debounce_ms: DEFAULT_CROSSING_DEBOUNCE_MS,
            min_lap_time_ms: DEFAULT_MIN_LAP_TIME_MS,
            direction_tolerance: DEFAULT_DIRECTION_TOLERANCE,
        }
    }

    fn reset(&mut self) {
        self.lap_start_ms = 0;
        self.current_lap_ms = 0;
        self.lap_count = 0;
        self.best_lap_ms = None;
        self.frame_flags = FLAG_NONE;
        self.last_crossing_ms = 0;
        self.crossed_start_this_run = false;
    }

    /// Check for crossing on a loop track
    fn check_loop_crossing(
        &mut self,
        prev: (f32, f32),
        pos: (f32, f32),
        velocity: (f32, f32),
        timestamp_ms: u32,
        line: &TimingLine,
    ) {
        // Check if movement segment crosses timing line
        if line_segment_intersection(prev, pos, line.p1, line.p2).is_some() {
            // Validate direction
            if !direction_valid(velocity, line.direction, self.direction_tolerance) {
                self.frame_flags |= INVALID_LAP;
                return;
            }

            // Check debounce (only applies after first crossing, when timing is active)
            if self.state == LapTimerState::Timing
                && timestamp_ms.saturating_sub(self.last_crossing_ms) < self.crossing_debounce_ms
            {
                return;
            }
            self.last_crossing_ms = timestamp_ms;

            // Handle based on current state
            match self.state {
                LapTimerState::Idle => {}
                LapTimerState::Armed => {
                    // First crossing starts timing
                    self.frame_flags |= CROSSED_START;
                    self.lap_start_ms = timestamp_ms;
                    self.current_lap_ms = 0;
                    self.state = LapTimerState::Timing;
                }
                LapTimerState::Timing => {
                    // Subsequent crossing completes lap
                    let lap_time = timestamp_ms.saturating_sub(self.lap_start_ms);

                    // Validate lap time
                    if lap_time < self.min_lap_time_ms {
                        self.frame_flags |= INVALID_LAP;
                        return;
                    }

                    // Record lap
                    self.frame_flags |= CROSSED_START | NEW_LAP;
                    self.lap_count = self.lap_count.saturating_add(1);

                    // Check for new best
                    if self.best_lap_ms.is_none_or(|best| lap_time < best) {
                        self.best_lap_ms = Some(lap_time);
                        self.frame_flags |= NEW_BEST;
                    }

                    // Start new lap
                    self.lap_start_ms = timestamp_ms;
                    self.current_lap_ms = 0;
                }
            }
        }
    }

    /// Check for crossing on a point-to-point track
    fn check_point_to_point_crossing(
        &mut self,
        prev: (f32, f32),
        pos: (f32, f32),
        velocity: (f32, f32),
        timestamp_ms: u32,
        start: &TimingLine,
        finish: &TimingLine,
    ) {
        // Check start line crossing
        if line_segment_intersection(prev, pos, start.p1, start.p2).is_some()
            && direction_valid(velocity, start.direction, self.direction_tolerance)
        {
            // Debounce only applies if we're already timing (re-crossing start)
            if self.state == LapTimerState::Timing {
                let since_last = timestamp_ms.saturating_sub(self.last_crossing_ms);
                if since_last < self.crossing_debounce_ms {
                    return;
                }
            }
            self.last_crossing_ms = timestamp_ms;
            self.frame_flags |= CROSSED_START;
            self.crossed_start_this_run = true;
            self.lap_start_ms = timestamp_ms;
            self.current_lap_ms = 0;
            self.state = LapTimerState::Timing;
        }

        // Check finish line crossing (only if we crossed start)
        if self.crossed_start_this_run
            && line_segment_intersection(prev, pos, finish.p1, finish.p2).is_some()
            && direction_valid(velocity, finish.direction, self.direction_tolerance)
        {
            let lap_time = timestamp_ms.saturating_sub(self.lap_start_ms);

            // Validate lap time
            if lap_time < self.min_lap_time_ms {
                self.frame_flags |= INVALID_LAP;
                return;
            }

            self.frame_flags |= CROSSED_FINISH | NEW_LAP;
            self.lap_count = self.lap_count.saturating_add(1);

            // Check for new best
            if self.best_lap_ms.is_none_or(|best| lap_time < best) {
                self.best_lap_ms = Some(lap_time);
                self.frame_flags |= NEW_BEST;
            }

            // Reset for next run
            self.crossed_start_this_run = false;
            self.state = LapTimerState::Armed;
            self.current_lap_ms = 0;
        }
    }
}

/// Main lap timer struct
pub struct LapTimer {
    track: Option<TrackType>,
    timing: TimingState,
    prev_pos: Option<(f32, f32)>,
}

impl LapTimer {
    /// Create a new lap timer in idle state
    pub fn new() -> Self {
        Self {
            track: None,
            timing: TimingState::new(),
            prev_pos: None,
        }
    }

    /// Configure for loop track (single timing line for start/finish)
    pub fn configure_loop(&mut self, line: TimingLine) {
        self.track = Some(TrackType::Loop { line });
        self.timing.state = LapTimerState::Armed;
        self.timing.reset();
        self.prev_pos = None;
    }

    /// Configure for point-to-point (separate start and finish lines)
    pub fn configure_point_to_point(&mut self, start: TimingLine, finish: TimingLine) {
        self.track = Some(TrackType::PointToPoint { start, finish });
        self.timing.state = LapTimerState::Armed;
        self.timing.reset();
        self.prev_pos = None;
    }

    /// Clear configuration and return to idle
    pub fn clear(&mut self) {
        self.track = None;
        self.timing.state = LapTimerState::Idle;
        self.timing.reset();
        self.prev_pos = None;
    }

    /// Update with new position. Returns flags for this frame.
    /// Call at telemetry rate (20-30Hz) from main loop.
    ///
    /// # Arguments
    /// * `pos` - Current position in local meters (x, y)
    /// * `velocity` - Current velocity in m/s (vx, vy) for direction validation
    /// * `timestamp_ms` - Current timestamp in milliseconds
    /// * `speed_mps` - Current speed in m/s (for minimum speed check)
    ///
    /// # Returns
    /// Bitfield of flags (CROSSED_START, CROSSED_FINISH, NEW_LAP, NEW_BEST, INVALID_LAP)
    pub fn update(
        &mut self,
        pos: (f32, f32),
        velocity: (f32, f32),
        timestamp_ms: u32,
        speed_mps: f32,
    ) -> u8 {
        // Clear per-frame flags
        self.timing.frame_flags = FLAG_NONE;

        // Update current lap time if timing
        if self.timing.state == LapTimerState::Timing {
            self.timing.current_lap_ms = timestamp_ms.saturating_sub(self.timing.lap_start_ms);
        }

        // Need previous position for crossing detection
        let Some(prev) = self.prev_pos else {
            self.prev_pos = Some(pos);
            return self.timing.frame_flags;
        };

        // Update previous position for next frame
        self.prev_pos = Some(pos);

        // Skip if no track configured
        let Some(track) = &self.track else {
            return self.timing.frame_flags;
        };

        // Skip if nearly stationary (avoid false crossings while parked on line)
        if speed_mps < 0.5 {
            return self.timing.frame_flags;
        }

        // Check for line crossings based on track type
        // Borrow track immutably, timing mutably - no conflict since they're separate fields
        match *track {
            TrackType::Loop { ref line } => {
                self.timing
                    .check_loop_crossing(prev, pos, velocity, timestamp_ms, line);
            }
            TrackType::PointToPoint {
                ref start,
                ref finish,
            } => {
                self.timing
                    .check_point_to_point_crossing(prev, pos, velocity, timestamp_ms, start, finish);
            }
        }

        self.timing.frame_flags
    }

    /// Get current lap time in milliseconds (0 if not timing)
    pub fn current_lap_ms(&self) -> u32 {
        self.timing.current_lap_ms
    }

    /// Get completed lap count
    pub fn lap_count(&self) -> u16 {
        self.timing.lap_count
    }
}

impl Default for LapTimer {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Geometry helpers (pure functions, easy to test)
// ============================================================================

/// Line segment intersection test using parametric form
///
/// Tests if segment (p1, p2) intersects segment (p3, p4).
/// Returns Some(t) where t is the interpolation factor along (p1, p2)
/// if they intersect (0 <= t <= 1 means intersection is on segment).
///
/// Uses the formula:
/// ```text
/// P1 + t(P2-P1) = P3 + u(P4-P3)
/// ```
/// Solving for t and u, intersection occurs if 0 <= t <= 1 and 0 <= u <= 1.
pub fn line_segment_intersection(
    p1: (f32, f32),
    p2: (f32, f32),
    p3: (f32, f32),
    p4: (f32, f32),
) -> Option<f32> {
    let d1x = p2.0 - p1.0;
    let d1y = p2.1 - p1.1;
    let d2x = p4.0 - p3.0;
    let d2y = p4.1 - p3.1;

    // Cross product of direction vectors
    let cross = d1x * d2y - d1y * d2x;

    // Parallel or nearly parallel lines (no intersection)
    if cross.abs() < 1e-10 {
        return None;
    }

    // Vector from p1 to p3
    let dx = p3.0 - p1.0;
    let dy = p3.1 - p1.1;

    // Solve for t and u
    let t = (dx * d2y - dy * d2x) / cross;
    let u = (dx * d1y - dy * d1x) / cross;

    // Check if intersection is within both segments
    // Use small epsilon for numerical stability at endpoints
    const EPSILON: f32 = 1e-6;
    let valid_range = -EPSILON..=(1.0 + EPSILON);
    if valid_range.contains(&t) && valid_range.contains(&u) {
        Some(t.clamp(0.0, 1.0))
    } else {
        None
    }
}

/// Normalize angle to [-π, π]
fn wrap_angle(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

/// Check if velocity direction is within tolerance of expected crossing direction
///
/// # Arguments
/// * `velocity` - Current velocity vector (vx, vy) in m/s
/// * `expected_dir` - Expected crossing direction in radians (0 = +X, π/2 = +Y)
/// * `tolerance` - Maximum deviation from expected direction in radians
///
/// # Returns
/// true if velocity direction is within tolerance of expected direction
pub fn direction_valid(velocity: (f32, f32), expected_dir: f32, tolerance: f32) -> bool {
    // Calculate velocity direction
    let speed_sq = velocity.0 * velocity.0 + velocity.1 * velocity.1;
    if speed_sq < 0.01 {
        // Too slow to determine direction
        return false;
    }

    let vel_dir = velocity.1.atan2(velocity.0);
    let diff = wrap_angle(vel_dir - expected_dir);

    diff.abs() <= tolerance
}

/// Calculate perpendicular timing line from two path points
///
/// Given two consecutive points on a path, creates a timing line perpendicular
/// to the direction of travel, centered on the first point.
///
/// # Arguments
/// * `p0` - Position where timing line should be placed
/// * `p1` - Next position along the path (defines direction of travel)
/// * `width` - Half-width of timing line (line extends this far on each side)
///
/// # Returns
/// TimingLine perpendicular to travel direction, with direction set to travel direction
pub fn timing_line_from_path(p0: (f32, f32), p1: (f32, f32), width: f32) -> TimingLine {
    let dx = p1.0 - p0.0;
    let dy = p1.1 - p0.1;
    let len = (dx * dx + dy * dy).sqrt();

    if len < 1e-6 {
        // Points too close, create horizontal line facing north
        return TimingLine {
            p1: (p0.0 - width, p0.1),
            p2: (p0.0 + width, p0.1),
            direction: PI / 2.0,
        };
    }

    // Perpendicular direction (90° counterclockwise)
    let perp_x = -dy / len;
    let perp_y = dx / len;

    TimingLine {
        p1: (p0.0 + perp_x * width, p0.1 + perp_y * width),
        p2: (p0.0 - perp_x * width, p0.1 - perp_y * width),
        direction: dy.atan2(dx),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    // ========================================================================
    // Geometry tests
    // ========================================================================

    #[test]
    fn test_line_segment_intersection_perpendicular() {
        // Vertical line crossing horizontal line
        let t = line_segment_intersection(
            (0.0, -1.0),
            (0.0, 1.0), // Movement: vertical up
            (-1.0, 0.0),
            (1.0, 0.0), // Line: horizontal
        );
        assert!(t.is_some());
        let t = t.unwrap();
        assert!((t - 0.5).abs() < 1e-6, "Expected t=0.5, got {}", t);
    }

    #[test]
    fn test_line_segment_intersection_parallel_no_cross() {
        // Two parallel horizontal lines
        let t = line_segment_intersection(
            (0.0, 0.0),
            (1.0, 0.0), // First line
            (0.0, 1.0),
            (1.0, 1.0), // Second line (parallel, offset)
        );
        assert!(t.is_none());
    }

    #[test]
    fn test_line_segment_intersection_diagonal() {
        // Diagonal crossing
        let t = line_segment_intersection(
            (0.0, 0.0),
            (2.0, 2.0), // Movement: diagonal
            (0.0, 2.0),
            (2.0, 0.0), // Line: opposite diagonal
        );
        assert!(t.is_some());
        let t = t.unwrap();
        assert!((t - 0.5).abs() < 1e-6, "Expected t=0.5, got {}", t);
    }

    #[test]
    fn test_line_segment_intersection_no_cross_segments_too_short() {
        // Lines would intersect if extended, but segments don't reach
        let t = line_segment_intersection(
            (0.0, 0.0),
            (1.0, 0.0), // Horizontal line ending at x=1
            (2.0, -1.0),
            (2.0, 1.0), // Vertical line at x=2 (doesn't reach)
        );
        assert!(t.is_none());
    }

    #[test]
    fn test_line_segment_intersection_at_endpoint() {
        // Crossing exactly at endpoint of timing line
        let t = line_segment_intersection(
            (0.0, 0.0),
            (1.0, 0.0), // Movement: horizontal
            (0.5, 0.0),
            (0.5, 1.0), // Line: starts at movement path
        );
        assert!(t.is_some());
        let t = t.unwrap();
        assert!((t - 0.5).abs() < 1e-6, "Expected t=0.5, got {}", t);
    }

    #[test]
    fn test_line_segment_intersection_collinear_no_overlap() {
        // Same line but segments don't overlap
        let t = line_segment_intersection(
            (0.0, 0.0),
            (1.0, 0.0), // First segment
            (2.0, 0.0),
            (3.0, 0.0), // Second segment (same line, no overlap)
        );
        // Collinear lines should return None (cross product is 0)
        assert!(t.is_none());
    }

    #[test]
    fn test_wrap_angle() {
        assert!((wrap_angle(0.0) - 0.0).abs() < 1e-6);
        assert!((wrap_angle(PI) - PI).abs() < 1e-6);
        assert!((wrap_angle(-PI) - (-PI)).abs() < 1e-6);
        assert!((wrap_angle(2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((wrap_angle(-2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((wrap_angle(3.0 * PI) - PI).abs() < 1e-6);
        assert!((wrap_angle(1.5 * PI) - (-0.5 * PI)).abs() < 1e-6);
    }

    #[test]
    fn test_direction_valid_exact_match() {
        // Moving exactly in expected direction (+X)
        assert!(direction_valid((1.0, 0.0), 0.0, FRAC_PI_2));
        // Moving exactly in expected direction (+Y)
        assert!(direction_valid((0.0, 1.0), FRAC_PI_2, FRAC_PI_2));
    }

    #[test]
    fn test_direction_valid_opposite() {
        // Moving opposite to expected direction
        assert!(!direction_valid((-1.0, 0.0), 0.0, FRAC_PI_2)); // Moving -X, expected +X
        assert!(!direction_valid((0.0, -1.0), FRAC_PI_2, FRAC_PI_2)); // Moving -Y, expected +Y
    }

    #[test]
    fn test_direction_valid_within_tolerance() {
        // Moving diagonally, within ±90° of expected
        assert!(direction_valid((1.0, 1.0), 0.0, FRAC_PI_2)); // 45° from +X
        assert!(direction_valid((1.0, -1.0), 0.0, FRAC_PI_2)); // -45° from +X
    }

    #[test]
    fn test_direction_valid_too_slow() {
        // Nearly stationary - direction can't be determined
        assert!(!direction_valid((0.0001, 0.0001), 0.0, FRAC_PI_2));
    }

    #[test]
    fn test_direction_valid_uses_math_convention() {
        // IMPORTANT: direction uses MATH convention, NOT compass convention
        // Math: 0 = East (+X), π/2 = North (+Y), counter-clockwise positive
        // Compass: 0 = North, π/2 = East, clockwise positive
        //
        // Dashboard calculates direction via atan2(dy,dx) in _calculateTimingLine(),
        // which is ALREADY math convention. Do NOT convert - pass directly to firmware.

        // Driving East (vx=1, vy=0) should match direction=0 (East in math convention)
        assert!(direction_valid((1.0, 0.0), 0.0, FRAC_PI_2));

        // Driving North (vx=0, vy=1) should match direction=π/2 (North in math convention)
        assert!(direction_valid((0.0, 1.0), FRAC_PI_2, FRAC_PI_2));

        // Driving West (vx=-1, vy=0) should match direction=π (West in math convention)
        assert!(direction_valid((-1.0, 0.0), PI, FRAC_PI_2));

        // Driving South (vx=0, vy=-1) should match direction=-π/2 (South in math convention)
        assert!(direction_valid((0.0, -1.0), -FRAC_PI_2, FRAC_PI_2));

        // Cross-check: driving North should NOT match direction=0 (that's East!)
        // This would fail if someone mistakenly used compass convention
        assert!(!direction_valid((0.0, 1.0), 0.0, FRAC_PI_2 - 0.01));
    }

    #[test]
    fn test_timing_line_from_path_horizontal() {
        let line = timing_line_from_path((0.0, 0.0), (10.0, 0.0), 5.0);

        // Traveling +X, perpendicular line should be vertical
        assert!((line.p1.0 - 0.0).abs() < 1e-6);
        assert!((line.p2.0 - 0.0).abs() < 1e-6);
        assert!((line.p1.1 - 5.0).abs() < 1e-6);
        assert!((line.p2.1 - (-5.0)).abs() < 1e-6);
        assert!((line.direction - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_timing_line_from_path_vertical() {
        let line = timing_line_from_path((0.0, 0.0), (0.0, 10.0), 5.0);

        // Traveling +Y, perpendicular line should be horizontal
        assert!((line.p1.1 - 0.0).abs() < 1e-6);
        assert!((line.p2.1 - 0.0).abs() < 1e-6);
        assert!((line.direction - FRAC_PI_2).abs() < 1e-6);
    }

    // ========================================================================
    // Lap timer state machine tests
    // ========================================================================

    #[test]
    fn test_lap_timer_initial_state() {
        let timer = LapTimer::new();
        assert_eq!(timer.timing.state, LapTimerState::Idle);
        assert!(timer.track.is_none());
        assert_eq!(timer.lap_count(), 0);
    }

    #[test]
    fn test_lap_timer_configure_loop() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        assert_eq!(timer.timing.state, LapTimerState::Armed);
        assert!(timer.track.is_some());
    }

    #[test]
    fn test_lap_timer_loop_first_crossing() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Start below line
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Armed);

        // Cross line moving north
        let flags = timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);

        assert!(flags & CROSSED_START != 0, "Should have CROSSED_START flag");
        assert_eq!(timer.timing.state, LapTimerState::Timing);
        assert_eq!(timer.lap_count(), 0); // Lap not complete yet
    }

    #[test]
    fn test_lap_timer_loop_complete_lap() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000; // 1 second minimum
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // First crossing - starts timing
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Simulate driving around (position changes but doesn't cross line)
        timer.update((50.0, 50.0), (10.0, 0.0), 30_000, 10.0);
        timer.update((50.0, -50.0), (0.0, -10.0), 45_000, 10.0);

        // Complete lap - cross line again
        timer.update((0.0, -5.0), (0.0, 10.0), 59_000, 10.0);
        let flags = timer.update((0.0, 5.0), (0.0, 10.0), 60_000, 10.0);

        assert!(flags & NEW_LAP != 0, "Should have NEW_LAP flag");
        assert!(flags & NEW_BEST != 0, "Should have NEW_BEST flag (first lap)");
        assert_eq!(timer.lap_count(), 1);
    }

    #[test]
    fn test_lap_timer_wrong_direction_rejected() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Try to cross going south (wrong direction)
        timer.update((0.0, 5.0), (0.0, -10.0), 0, 10.0);
        let flags = timer.update((0.0, -5.0), (0.0, -10.0), 100, 10.0);

        assert!(flags & INVALID_LAP != 0, "Should have INVALID_LAP flag");
        assert_eq!(
            timer.timing.state,
            LapTimerState::Armed,
            "Should still be Armed"
        );
    }

    #[test]
    fn test_lap_timer_debounce() {
        let mut timer = LapTimer::new();
        timer.timing.crossing_debounce_ms = 500;
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // First crossing
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        let flags1 = timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert!(flags1 & CROSSED_START != 0);

        // Try to cross again too soon (within debounce)
        timer.update((0.0, -5.0), (0.0, -10.0), 200, 10.0); // Go back
        let flags2 = timer.update((0.0, 5.0), (0.0, 10.0), 300, 10.0); // Try crossing again
        assert_eq!(flags2, FLAG_NONE, "Should be rejected by debounce");
    }

    #[test]
    fn test_lap_timer_min_lap_time() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 10_000; // 10 second minimum
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Start timing - cross line at t=100
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive around track (positions that don't cross line at y=0)
        timer.update((50.0, 50.0), (10.0, 0.0), 2000, 10.0);
        timer.update((50.0, -50.0), (0.0, -10.0), 4000, 10.0);

        // Try to complete lap too fast (only ~5 seconds since start)
        // Cross from south to north
        timer.update((0.0, -5.0), (0.0, 10.0), 5000, 10.0);
        let flags = timer.update((0.0, 5.0), (0.0, 10.0), 5100, 10.0);

        assert!(flags & INVALID_LAP != 0, "Should have INVALID_LAP flag");
        assert_eq!(timer.lap_count(), 0, "Lap should not be counted");
    }

    #[test]
    fn test_lap_timer_best_lap_tracking() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Lap 1: Start timing (cross at t=100)
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive around track (positions away from finish line)
        timer.update((100.0, 100.0), (10.0, 0.0), 30_000, 10.0);
        timer.update((100.0, -100.0), (0.0, -10.0), 50_000, 10.0);

        // Complete lap 1: 60 seconds (cross at t=60100)
        timer.update((0.0, -5.0), (0.0, 10.0), 60_000, 10.0);
        let flags1 = timer.update((0.0, 5.0), (0.0, 10.0), 60_100, 10.0);
        assert!(flags1 & NEW_LAP != 0, "Should complete lap");
        assert!(flags1 & NEW_BEST != 0, "First lap should be best");
        assert_eq!(timer.lap_count(), 1);
        assert_eq!(timer.timing.best_lap_ms, Some(60_000));

        // Drive lap 2 (positions away from finish line)
        timer.update((100.0, 100.0), (10.0, 0.0), 90_000, 10.0);
        timer.update((100.0, -100.0), (0.0, -10.0), 110_000, 10.0);

        // Complete lap 2: 55 seconds (new best)
        timer.update((0.0, -5.0), (0.0, 10.0), 115_000, 10.0);
        let flags2 = timer.update((0.0, 5.0), (0.0, 10.0), 115_100, 10.0);
        assert!(flags2 & NEW_LAP != 0);
        assert!(flags2 & NEW_BEST != 0, "55s should be new best");
        assert_eq!(timer.lap_count(), 2);
        assert_eq!(timer.timing.best_lap_ms, Some(55_000));

        // Drive lap 3 (positions away from finish line)
        timer.update((100.0, 100.0), (10.0, 0.0), 150_000, 10.0);
        timer.update((100.0, -100.0), (0.0, -10.0), 170_000, 10.0);

        // Complete lap 3: 58 seconds (NOT a new best)
        timer.update((0.0, -5.0), (0.0, 10.0), 173_000, 10.0);
        let flags3 = timer.update((0.0, 5.0), (0.0, 10.0), 173_100, 10.0);
        assert!(flags3 & NEW_LAP != 0);
        assert!(flags3 & NEW_BEST == 0, "58s should NOT be new best");
        assert_eq!(timer.lap_count(), 3);
        assert_eq!(timer.timing.best_lap_ms, Some(55_000)); // Still 55 seconds
    }

    #[test]
    fn test_lap_timer_point_to_point() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        timer.configure_point_to_point(
            TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2),   // Start
            TimingLine::new((-10.0, 100.0), (10.0, 100.0), FRAC_PI_2), // Finish
        );

        assert_eq!(timer.timing.state, LapTimerState::Armed);

        // Cross start line
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        let flags1 = timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);
        assert!(flags1 & CROSSED_START != 0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive towards finish
        timer.update((0.0, 50.0), (0.0, 10.0), 15_000, 10.0);

        // Cross finish line
        timer.update((0.0, 95.0), (0.0, 10.0), 29_000, 10.0);
        let flags2 = timer.update((0.0, 105.0), (0.0, 10.0), 30_000, 10.0);

        assert!(flags2 & CROSSED_FINISH != 0);
        assert!(flags2 & NEW_LAP != 0);
        assert_eq!(timer.lap_count(), 1);
        assert_eq!(timer.timing.state, LapTimerState::Armed); // Ready for next run
    }

    #[test]
    fn test_lap_timer_clear() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Do some timing
        timer.update((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update((0.0, 5.0), (0.0, 10.0), 100, 10.0);

        // Clear
        timer.clear();

        assert_eq!(timer.timing.state, LapTimerState::Idle);
        assert!(timer.track.is_none());
        assert_eq!(timer.lap_count(), 0);
    }

    #[test]
    fn test_lap_timer_stationary_ignored() {
        let mut timer = LapTimer::new();
        timer.configure_loop(TimingLine::new((-10.0, 0.0), (10.0, 0.0), FRAC_PI_2));

        // Try crossing while nearly stationary
        timer.update((0.0, -0.1), (0.0, 0.1), 0, 0.1);
        let flags = timer.update((0.0, 0.1), (0.0, 0.1), 100, 0.1);

        assert_eq!(flags, FLAG_NONE, "Should ignore crossing when stationary");
        assert_eq!(timer.timing.state, LapTimerState::Armed);
    }
}
