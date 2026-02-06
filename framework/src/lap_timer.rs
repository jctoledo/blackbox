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

/// A timing line defined by two GPS endpoints and a valid crossing direction
///
/// Uses GPS lat/lon coordinates directly for session-independent timing.
/// This eliminates coordinate frame mismatches between firmware sessions.
#[derive(Clone, Copy, Debug)]
pub struct TimingLine {
    /// First endpoint latitude (degrees)
    pub p1_lat: f64,
    /// First endpoint longitude (degrees)
    pub p1_lon: f64,
    /// Second endpoint latitude (degrees)
    pub p2_lat: f64,
    /// Second endpoint longitude (degrees)
    pub p2_lon: f64,
    /// Valid crossing direction in radians (math convention: 0 = East, π/2 = North)
    pub direction: f32,
}

impl TimingLine {
    /// Create a new timing line from GPS coordinates
    pub fn new(p1_lat: f64, p1_lon: f64, p2_lat: f64, p2_lon: f64, direction: f32) -> Self {
        Self {
            p1_lat,
            p1_lon,
            p2_lat,
            p2_lon,
            direction,
        }
    }

    /// Convert GPS coordinates to local meters for intersection test
    /// Uses the line's center as reference to minimize projection error
    fn to_local(self, lat: f64, lon: f64) -> (f64, f64) {
        let ref_lat = (self.p1_lat + self.p2_lat) / 2.0;
        let cos_lat = (ref_lat * core::f64::consts::PI / 180.0).cos();
        let x = (lon - self.p1_lon) * cos_lat * 111320.0;
        let y = (lat - self.p1_lat) * 111320.0;
        (x, y)
    }

    /// Get line endpoints in local meters (relative to p1)
    fn endpoints_local(&self) -> ((f64, f64), (f64, f64)) {
        let p1 = (0.0, 0.0); // p1 is origin
        let p2 = self.to_local(self.p2_lat, self.p2_lon);
        (p1, p2)
    }

    /// Convert GPS timing line to local meters relative to a GPS reference point
    ///
    /// Call when track is configured AND GPS origin is available.
    /// Returns a LocalTimingLine for use with high-frequency EKF position updates.
    pub fn to_local_line(&self, ref_lat: f64, ref_lon: f64) -> LocalTimingLine {
        let cos_lat = (ref_lat * core::f64::consts::PI / 180.0).cos();
        LocalTimingLine {
            p1: (
                ((self.p1_lon - ref_lon) * cos_lat * 111320.0) as f32,
                ((self.p1_lat - ref_lat) * 111320.0) as f32,
            ),
            p2: (
                ((self.p2_lon - ref_lon) * cos_lat * 111320.0) as f32,
                ((self.p2_lat - ref_lat) * 111320.0) as f32,
            ),
            direction: self.direction,
        }
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

/// A timing line in local meter coordinates (cached for 200Hz updates)
#[derive(Clone, Copy, Debug)]
pub struct LocalTimingLine {
    /// Endpoint 1 in local meters (x, y)
    pub p1: (f32, f32),
    /// Endpoint 2 in local meters (x, y)
    pub p2: (f32, f32),
    /// Valid crossing direction (radians, math convention: 0 = East, π/2 = North)
    pub direction: f32,
}

/// Track with cached local coordinates for high-frequency updates
#[derive(Clone, Copy, Debug)]
pub enum LocalTrackType {
    /// Single start/finish line (circuit racing)
    Loop { line: LocalTimingLine },
    /// Separate start and finish lines (hill climb, rally stage)
    PointToPoint {
        start: LocalTimingLine,
        finish: LocalTimingLine,
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
        prev: (f64, f64),
        pos: (f64, f64),
        velocity: (f32, f32),
        timestamp_ms: u32,
        line: &TimingLine,
    ) {
        // Convert GPS positions to local meters for intersection test
        let prev_local = line.to_local(prev.0, prev.1);
        let pos_local = line.to_local(pos.0, pos.1);
        let (line_p1, line_p2) = line.endpoints_local();

        // Check if movement segment crosses timing line
        if line_segment_intersection_f64(prev_local, pos_local, line_p1, line_p2).is_some() {
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
        prev: (f64, f64),
        pos: (f64, f64),
        velocity: (f32, f32),
        timestamp_ms: u32,
        start: &TimingLine,
        finish: &TimingLine,
    ) {
        // Convert GPS positions to local meters for start line intersection test
        let prev_start = start.to_local(prev.0, prev.1);
        let pos_start = start.to_local(pos.0, pos.1);
        let (start_p1, start_p2) = start.endpoints_local();

        // Check start line crossing
        if line_segment_intersection_f64(prev_start, pos_start, start_p1, start_p2).is_some()
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
        // Convert GPS positions to local meters for finish line intersection test
        let prev_finish = finish.to_local(prev.0, prev.1);
        let pos_finish = finish.to_local(pos.0, pos.1);
        let (finish_p1, finish_p2) = finish.endpoints_local();

        if self.crossed_start_this_run
            && line_segment_intersection_f64(prev_finish, pos_finish, finish_p1, finish_p2)
                .is_some()
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

    /// Check for crossing on a loop track using local coordinates with timestamp interpolation
    ///
    /// Uses f32 local meter positions for 200Hz EKF updates.
    /// Interpolates crossing timestamp for sub-millisecond precision.
    fn check_local_loop_crossing(
        &mut self,
        prev: (f32, f32),
        pos: (f32, f32),
        velocity: (f32, f32),
        prev_ts: u32,
        timestamp_ms: u32,
        line: &LocalTimingLine,
    ) {
        // Check if movement segment crosses timing line
        if let Some(t) = line_segment_intersection(prev, pos, line.p1, line.p2) {
            // Validate direction
            if !direction_valid(velocity, line.direction, self.direction_tolerance) {
                self.frame_flags |= INVALID_LAP;
                return;
            }

            // Interpolate crossing timestamp using the intersection fraction
            let dt = timestamp_ms.saturating_sub(prev_ts);
            let crossing_time = prev_ts + (t * dt as f32) as u32;

            // Check debounce (only applies after first crossing, when timing is active)
            if self.state == LapTimerState::Timing
                && crossing_time.saturating_sub(self.last_crossing_ms) < self.crossing_debounce_ms
            {
                return;
            }
            self.last_crossing_ms = crossing_time;

            // Handle based on current state
            match self.state {
                LapTimerState::Idle => {}
                LapTimerState::Armed => {
                    // First crossing starts timing - use interpolated time
                    self.frame_flags |= CROSSED_START;
                    self.lap_start_ms = crossing_time;
                    self.current_lap_ms = 0;
                    self.state = LapTimerState::Timing;
                }
                LapTimerState::Timing => {
                    // Subsequent crossing completes lap - use interpolated time
                    let lap_time = crossing_time.saturating_sub(self.lap_start_ms);

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

                    // Start new lap with interpolated time
                    self.lap_start_ms = crossing_time;
                    self.current_lap_ms = 0;
                }
            }
        }
    }

    /// Check for crossing on a point-to-point track using local coordinates with timestamp interpolation
    ///
    /// Uses f32 local meter positions for 200Hz EKF updates.
    /// Interpolates crossing timestamp for sub-millisecond precision.
    ///
    /// # Arguments
    /// * `timestamps` - (prev_ts, current_ts) for interpolation
    /// * `lines` - (start, finish) timing lines
    #[allow(clippy::too_many_arguments)]
    fn check_local_p2p_crossing(
        &mut self,
        prev: (f32, f32),
        pos: (f32, f32),
        velocity: (f32, f32),
        prev_ts: u32,
        timestamp_ms: u32,
        start: &LocalTimingLine,
        finish: &LocalTimingLine,
    ) {
        // Calculate dt once for interpolation
        let dt = timestamp_ms.saturating_sub(prev_ts);

        // Check start line crossing
        if let Some(t) = line_segment_intersection(prev, pos, start.p1, start.p2) {
            if direction_valid(velocity, start.direction, self.direction_tolerance) {
                // Interpolate crossing timestamp
                let crossing_time = prev_ts + (t * dt as f32) as u32;

                // Debounce only applies if we're already timing (re-crossing start)
                if self.state == LapTimerState::Timing {
                    let since_last = crossing_time.saturating_sub(self.last_crossing_ms);
                    if since_last < self.crossing_debounce_ms {
                        return;
                    }
                }
                self.last_crossing_ms = crossing_time;
                self.frame_flags |= CROSSED_START;
                self.crossed_start_this_run = true;
                self.lap_start_ms = crossing_time;
                self.current_lap_ms = 0;
                self.state = LapTimerState::Timing;
            }
        }

        // Check finish line crossing (only if we crossed start)
        if self.crossed_start_this_run {
            if let Some(t) = line_segment_intersection(prev, pos, finish.p1, finish.p2) {
                if direction_valid(velocity, finish.direction, self.direction_tolerance) {
                    // Interpolate crossing timestamp
                    let crossing_time = prev_ts + (t * dt as f32) as u32;
                    let lap_time = crossing_time.saturating_sub(self.lap_start_ms);

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
    }
}

/// Main lap timer struct
pub struct LapTimer {
    track: Option<TrackType>,
    timing: TimingState,
    /// Previous GPS position (lat, lon) for crossing detection
    prev_pos: Option<(f64, f64)>,
    /// Cached local coordinates for high-frequency updates (200Hz)
    local_track: Option<LocalTrackType>,
    /// Previous EKF position for 200Hz crossing detection
    prev_ekf_pos: Option<(f32, f32)>,
    /// Previous EKF timestamp for interpolation
    prev_ekf_ts: u32,
    /// Accumulated flags from 200Hz updates (cleared on read)
    accumulated_flags: u8,
}

impl LapTimer {
    /// Create a new lap timer in idle state
    pub fn new() -> Self {
        Self {
            track: None,
            timing: TimingState::new(),
            prev_pos: None,
            local_track: None,
            prev_ekf_pos: None,
            prev_ekf_ts: 0,
            accumulated_flags: FLAG_NONE,
        }
    }

    /// Configure for loop track (single timing line for start/finish)
    pub fn configure_loop(&mut self, line: TimingLine) {
        self.track = Some(TrackType::Loop { line });
        self.timing.state = LapTimerState::Armed;
        self.timing.reset();
        self.prev_pos = None;
        // Reset local track (will be set when GPS reference becomes available)
        self.local_track = None;
        self.prev_ekf_pos = None;
        self.prev_ekf_ts = 0;
        self.accumulated_flags = FLAG_NONE;
    }

    /// Configure for point-to-point (separate start and finish lines)
    pub fn configure_point_to_point(&mut self, start: TimingLine, finish: TimingLine) {
        self.track = Some(TrackType::PointToPoint { start, finish });
        self.timing.state = LapTimerState::Armed;
        self.timing.reset();
        self.prev_pos = None;
        // Reset local track (will be set when GPS reference becomes available)
        self.local_track = None;
        self.prev_ekf_pos = None;
        self.prev_ekf_ts = 0;
        self.accumulated_flags = FLAG_NONE;
    }

    /// Clear configuration and return to idle
    pub fn clear(&mut self) {
        self.track = None;
        self.timing.state = LapTimerState::Idle;
        self.timing.reset();
        self.prev_pos = None;
        self.local_track = None;
        self.prev_ekf_pos = None;
        self.prev_ekf_ts = 0;
        self.accumulated_flags = FLAG_NONE;
    }

    /// Update with new GPS position. Returns flags for this frame.
    /// Call at GPS update rate (typically 25Hz) from main loop.
    ///
    /// # Arguments
    /// * `lat` - Current GPS latitude (degrees)
    /// * `lon` - Current GPS longitude (degrees)
    /// * `velocity` - Current velocity in m/s (vx, vy) for direction validation
    /// * `timestamp_ms` - Current timestamp in milliseconds
    /// * `speed_mps` - Current speed in m/s (for minimum speed check)
    ///
    /// # Returns
    /// Bitfield of flags (CROSSED_START, CROSSED_FINISH, NEW_LAP, NEW_BEST, INVALID_LAP)
    pub fn update(
        &mut self,
        lat: f64,
        lon: f64,
        velocity: (f32, f32),
        timestamp_ms: u32,
        speed_mps: f32,
    ) -> u8 {
        let pos = (lat, lon);
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
                self.timing.check_point_to_point_crossing(
                    prev,
                    pos,
                    velocity,
                    timestamp_ms,
                    start,
                    finish,
                );
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

    /// Check if track configuration exists
    pub fn has_track(&self) -> bool {
        self.track.is_some()
    }

    /// Convert GPS timing lines to local meters using GPS reference point
    ///
    /// Call when track is configured AND GPS origin is available.
    /// Returns true if local track was successfully created.
    pub fn set_gps_reference(&mut self, ref_lat: f64, ref_lon: f64) -> bool {
        let Some(track) = &self.track else {
            return false;
        };

        self.local_track = Some(match track {
            TrackType::Loop { line } => LocalTrackType::Loop {
                line: line.to_local_line(ref_lat, ref_lon),
            },
            TrackType::PointToPoint { start, finish } => LocalTrackType::PointToPoint {
                start: start.to_local_line(ref_lat, ref_lon),
                finish: finish.to_local_line(ref_lat, ref_lon),
            },
        });

        // Reset EKF tracking state for fresh start
        self.prev_ekf_pos = None;
        self.prev_ekf_ts = 0;
        self.accumulated_flags = FLAG_NONE;

        true
    }

    /// Check if local track is established (ready for 200Hz updates)
    pub fn has_local_track(&self) -> bool {
        self.local_track.is_some()
    }

    /// High-frequency update using EKF position (local meters)
    ///
    /// Call at 200Hz from IMU polling block for sub-millisecond precision.
    /// Uses timestamp interpolation for precise crossing time.
    ///
    /// # Arguments
    /// * `pos` - EKF position in local meters (x, y)
    /// * `velocity` - EKF velocity in m/s (vx, vy) for direction validation
    /// * `timestamp_ms` - Current timestamp in milliseconds
    /// * `speed_mps` - Current speed in m/s (for minimum speed check)
    ///
    /// # Returns
    /// Bitfield of flags (CROSSED_START, CROSSED_FINISH, NEW_LAP, NEW_BEST, INVALID_LAP)
    pub fn update_ekf(
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
        let Some(prev) = self.prev_ekf_pos else {
            self.prev_ekf_pos = Some(pos);
            self.prev_ekf_ts = timestamp_ms;
            return self.timing.frame_flags;
        };

        let prev_ts = self.prev_ekf_ts;

        // Update previous position for next frame
        self.prev_ekf_pos = Some(pos);
        self.prev_ekf_ts = timestamp_ms;

        // Skip if no local track configured
        let Some(local_track) = &self.local_track else {
            return self.timing.frame_flags;
        };

        // Skip if nearly stationary (avoid false crossings while parked on line)
        if speed_mps < 0.5 {
            return self.timing.frame_flags;
        }

        // Check for line crossings with interpolation
        match local_track {
            LocalTrackType::Loop { line } => {
                self.timing
                    .check_local_loop_crossing(prev, pos, velocity, prev_ts, timestamp_ms, line);
            }
            LocalTrackType::PointToPoint { start, finish } => {
                self.timing.check_local_p2p_crossing(
                    prev,
                    pos,
                    velocity,
                    prev_ts,
                    timestamp_ms,
                    start,
                    finish,
                );
            }
        }

        // Accumulate flags for telemetry block
        self.accumulated_flags |= self.timing.frame_flags;

        self.timing.frame_flags
    }

    /// Take accumulated flags from 200Hz updates (clears on read)
    ///
    /// Call from telemetry block to get all flags since last read.
    pub fn take_accumulated_flags(&mut self) -> u8 {
        let flags = self.accumulated_flags;
        self.accumulated_flags = FLAG_NONE;
        flags
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

/// Line segment intersection test using parametric form (f64 version)
///
/// Tests if segment (p1, p2) intersects segment (p3, p4).
/// Returns Some(t) where t is the interpolation factor along (p1, p2)
/// if they intersect (0 <= t <= 1 means intersection is on segment).
fn line_segment_intersection_f64(
    p1: (f64, f64),
    p2: (f64, f64),
    p3: (f64, f64),
    p4: (f64, f64),
) -> Option<f64> {
    let d1x = p2.0 - p1.0;
    let d1y = p2.1 - p1.1;
    let d2x = p4.0 - p3.0;
    let d2y = p4.1 - p3.1;

    // Cross product of direction vectors
    let cross = d1x * d2y - d1y * d2x;

    // Parallel or nearly parallel lines (no intersection)
    if cross.abs() < 1e-12 {
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
    const EPSILON: f64 = 1e-9;
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

    // ========================================================================
    // GPS coordinate test helpers
    // ========================================================================
    // Using coordinates near equator where 0.00001° ≈ 1.1m
    // Line at lat=0 from lon=-0.0001 to lon=0.0001 (about 22m wide E-W line)
    // Direction FRAC_PI_2 = crossing North (increasing lat)

    /// Create a test timing line at the equator
    /// Line runs East-West at lat=0, valid crossing direction is North
    fn test_line() -> TimingLine {
        // p1: lat=0, lon=-0.0001 (west endpoint)
        // p2: lat=0, lon=0.0001 (east endpoint)
        // direction: π/2 = North (increasing latitude)
        TimingLine::new(0.0, -0.0001, 0.0, 0.0001, FRAC_PI_2)
    }

    /// Create a test timing line at a specific latitude
    fn test_line_at_lat(lat: f64) -> TimingLine {
        TimingLine::new(lat, -0.0001, lat, 0.0001, FRAC_PI_2)
    }

    #[test]
    fn test_lap_timer_configure_loop() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        assert_eq!(timer.timing.state, LapTimerState::Armed);
        assert!(timer.track.is_some());
    }

    #[test]
    fn test_lap_timer_loop_first_crossing() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Start south of line (lat=-0.00005 ≈ 5.5m south)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Armed);

        // Cross line moving north (lat=0.00005 ≈ 5.5m north)
        let flags = timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);

        assert!(flags & CROSSED_START != 0, "Should have CROSSED_START flag");
        assert_eq!(timer.timing.state, LapTimerState::Timing);
        assert_eq!(timer.lap_count(), 0); // Lap not complete yet
    }

    #[test]
    fn test_lap_timer_loop_complete_lap() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000; // 1 second minimum
        timer.configure_loop(test_line());

        // First crossing - starts timing
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Simulate driving around (position changes but doesn't cross line at lat=0)
        timer.update(0.0005, 0.0005, (10.0, 0.0), 30_000, 10.0);
        timer.update(-0.0005, 0.0005, (0.0, -10.0), 45_000, 10.0);

        // Complete lap - cross line again
        timer.update(-0.00005, 0.0, (0.0, 10.0), 59_000, 10.0);
        let flags = timer.update(0.00005, 0.0, (0.0, 10.0), 60_000, 10.0);

        assert!(flags & NEW_LAP != 0, "Should have NEW_LAP flag");
        assert!(
            flags & NEW_BEST != 0,
            "Should have NEW_BEST flag (first lap)"
        );
        assert_eq!(timer.lap_count(), 1);
    }

    #[test]
    fn test_lap_timer_wrong_direction_rejected() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Try to cross going south (wrong direction)
        timer.update(0.00005, 0.0, (0.0, -10.0), 0, 10.0);
        let flags = timer.update(-0.00005, 0.0, (0.0, -10.0), 100, 10.0);

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
        timer.configure_loop(test_line());

        // First crossing
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        let flags1 = timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);
        assert!(flags1 & CROSSED_START != 0);

        // Try to cross again too soon (within debounce)
        timer.update(-0.00005, 0.0, (0.0, -10.0), 200, 10.0); // Go back
        let flags2 = timer.update(0.00005, 0.0, (0.0, 10.0), 300, 10.0); // Try crossing again
        assert_eq!(flags2, FLAG_NONE, "Should be rejected by debounce");
    }

    #[test]
    fn test_lap_timer_min_lap_time() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 10_000; // 10 second minimum
        timer.configure_loop(test_line());

        // Start timing - cross line at t=100
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive around track (positions that don't cross line at lat=0)
        timer.update(0.0005, 0.0005, (10.0, 0.0), 2000, 10.0);
        timer.update(-0.0005, 0.0005, (0.0, -10.0), 4000, 10.0);

        // Try to complete lap too fast (only ~5 seconds since start)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 5000, 10.0);
        let flags = timer.update(0.00005, 0.0, (0.0, 10.0), 5100, 10.0);

        assert!(flags & INVALID_LAP != 0, "Should have INVALID_LAP flag");
        assert_eq!(timer.lap_count(), 0, "Lap should not be counted");
    }

    #[test]
    fn test_lap_timer_best_lap_tracking() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        timer.configure_loop(test_line());

        // Lap 1: Start timing (cross at t=100)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive around track (positions away from finish line)
        timer.update(0.001, 0.001, (10.0, 0.0), 30_000, 10.0);
        timer.update(-0.001, 0.001, (0.0, -10.0), 50_000, 10.0);

        // Complete lap 1: 60 seconds (cross at t=60100)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 60_000, 10.0);
        let flags1 = timer.update(0.00005, 0.0, (0.0, 10.0), 60_100, 10.0);
        assert!(flags1 & NEW_LAP != 0, "Should complete lap");
        assert!(flags1 & NEW_BEST != 0, "First lap should be best");
        assert_eq!(timer.lap_count(), 1);
        assert_eq!(timer.timing.best_lap_ms, Some(60_000));

        // Drive lap 2 (positions away from finish line)
        timer.update(0.001, 0.001, (10.0, 0.0), 90_000, 10.0);
        timer.update(-0.001, 0.001, (0.0, -10.0), 110_000, 10.0);

        // Complete lap 2: 55 seconds (new best)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 115_000, 10.0);
        let flags2 = timer.update(0.00005, 0.0, (0.0, 10.0), 115_100, 10.0);
        assert!(flags2 & NEW_LAP != 0);
        assert!(flags2 & NEW_BEST != 0, "55s should be new best");
        assert_eq!(timer.lap_count(), 2);
        assert_eq!(timer.timing.best_lap_ms, Some(55_000));

        // Drive lap 3 (positions away from finish line)
        timer.update(0.001, 0.001, (10.0, 0.0), 150_000, 10.0);
        timer.update(-0.001, 0.001, (0.0, -10.0), 170_000, 10.0);

        // Complete lap 3: 58 seconds (NOT a new best)
        timer.update(-0.00005, 0.0, (0.0, 10.0), 173_000, 10.0);
        let flags3 = timer.update(0.00005, 0.0, (0.0, 10.0), 173_100, 10.0);
        assert!(flags3 & NEW_LAP != 0);
        assert!(flags3 & NEW_BEST == 0, "58s should NOT be new best");
        assert_eq!(timer.lap_count(), 3);
        assert_eq!(timer.timing.best_lap_ms, Some(55_000)); // Still 55 seconds
    }

    #[test]
    fn test_lap_timer_point_to_point() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        // Start line at lat=0, finish line at lat=0.001 (~111m north)
        timer.configure_point_to_point(
            test_line_at_lat(0.0),   // Start
            test_line_at_lat(0.001), // Finish (~111m north)
        );

        assert_eq!(timer.timing.state, LapTimerState::Armed);

        // Cross start line
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        let flags1 = timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);
        assert!(flags1 & CROSSED_START != 0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive towards finish
        timer.update(0.0005, 0.0, (0.0, 10.0), 15_000, 10.0);

        // Cross finish line (lat=0.001)
        timer.update(0.00095, 0.0, (0.0, 10.0), 29_000, 10.0);
        let flags2 = timer.update(0.00105, 0.0, (0.0, 10.0), 30_000, 10.0);

        assert!(flags2 & CROSSED_FINISH != 0);
        assert!(flags2 & NEW_LAP != 0);
        assert_eq!(timer.lap_count(), 1);
        assert_eq!(timer.timing.state, LapTimerState::Armed); // Ready for next run
    }

    #[test]
    fn test_lap_timer_clear() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Do some timing
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);

        // Clear
        timer.clear();

        assert_eq!(timer.timing.state, LapTimerState::Idle);
        assert!(timer.track.is_none());
        assert_eq!(timer.lap_count(), 0);
    }

    #[test]
    fn test_lap_timer_stationary_ignored() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Try crossing while nearly stationary
        timer.update(-0.000001, 0.0, (0.0, 0.1), 0, 0.1);
        let flags = timer.update(0.000001, 0.0, (0.0, 0.1), 100, 0.1);

        assert_eq!(flags, FLAG_NONE, "Should ignore crossing when stationary");
        assert_eq!(timer.timing.state, LapTimerState::Armed);
    }

    // ========================================================================
    // 200Hz EKF update tests (local coordinates)
    // ========================================================================

    #[test]
    fn test_timing_line_to_local_line() {
        // Create a timing line at the equator
        let line = test_line();

        // Reference point at equator origin
        let local = line.to_local_line(0.0, 0.0);

        // At equator, 0.0001° longitude ≈ 11.132 meters
        // p1 is at lon=-0.0001, so x should be about -11.1m
        // p2 is at lon=0.0001, so x should be about +11.1m
        // Both at lat=0 (y=0)
        assert!(
            (local.p1.0 - (-11.132)).abs() < 0.1,
            "p1.x expected ~-11.1, got {}",
            local.p1.0
        );
        assert!(
            local.p1.1.abs() < 0.1,
            "p1.y expected ~0, got {}",
            local.p1.1
        );
        assert!(
            (local.p2.0 - 11.132).abs() < 0.1,
            "p2.x expected ~11.1, got {}",
            local.p2.0
        );
        assert!(
            local.p2.1.abs() < 0.1,
            "p2.y expected ~0, got {}",
            local.p2.1
        );
        assert_eq!(local.direction, FRAC_PI_2);
    }

    #[test]
    fn test_timing_line_to_local_with_offset_reference() {
        // Create a timing line at equator
        let line = test_line();

        // Reference point offset 0.001° south (about 111m south)
        let local = line.to_local_line(-0.001, 0.0);

        // Line is at lat=0, reference is at lat=-0.001
        // So line should be about 111m NORTH of reference (positive Y)
        assert!(
            (local.p1.1 - 111.32).abs() < 1.0,
            "p1.y expected ~111, got {}",
            local.p1.1
        );
        assert!(
            (local.p2.1 - 111.32).abs() < 1.0,
            "p2.y expected ~111, got {}",
            local.p2.1
        );
    }

    #[test]
    fn test_set_gps_reference() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Before setting reference, no local track
        assert!(!timer.has_local_track());

        // Set GPS reference
        assert!(timer.set_gps_reference(0.0, 0.0));
        assert!(timer.has_local_track());

        // Verify local track was created correctly
        match timer.local_track {
            Some(LocalTrackType::Loop { line }) => {
                assert!(
                    (line.p1.0 - (-11.132)).abs() < 0.1,
                    "p1.x expected ~-11.1, got {}",
                    line.p1.0
                );
            }
            _ => panic!("Expected LocalTrackType::Loop"),
        }
    }

    #[test]
    fn test_update_ekf_crossing_interpolation() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        timer.configure_loop(test_line());
        timer.set_gps_reference(0.0, 0.0);

        // Line is at y=0 (East-West), crossing direction is North (+Y)
        // Local coords: line from (-11.1, 0) to (11.1, 0)

        // First position: south of line at y=-5m
        timer.update_ekf((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Armed);

        // Cross line: move from y=-5 to y=+5 (10m total)
        // Crossing happens at y=0, which is 50% of the way (t=0.5)
        // With timestamps 0 to 10ms, crossing should be at 5ms
        let flags = timer.update_ekf((0.0, 5.0), (0.0, 10.0), 10, 10.0);

        assert!(flags & CROSSED_START != 0, "Should have CROSSED_START flag");
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // lap_start_ms should be interpolated to ~5ms (halfway between 0 and 10)
        assert_eq!(
            timer.timing.lap_start_ms, 5,
            "lap_start_ms should be interpolated to 5"
        );
    }

    #[test]
    fn test_update_ekf_complete_lap_with_interpolation() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        timer.configure_loop(test_line());
        timer.set_gps_reference(0.0, 0.0);

        // First crossing at t=0-10ms (interpolated to 5ms)
        // From y=-5 to y=+5, crossing at y=0 is fraction t=0.5
        // crossing_time = 0 + 0.5 * (10-0) = 5ms
        timer.update_ekf((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update_ekf((0.0, 5.0), (0.0, 10.0), 10, 10.0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);
        assert_eq!(timer.timing.lap_start_ms, 5);

        // Drive around track (all positions stay north of y=0 to avoid crossing)
        timer.update_ekf((50.0, 50.0), (10.0, 0.0), 30_000, 10.0);
        timer.update_ekf((100.0, 100.0), (10.0, 10.0), 40_000, 10.0);
        timer.update_ekf((50.0, 100.0), (-10.0, 0.0), 50_000, 10.0);
        timer.update_ekf((0.0, 50.0), (0.0, -10.0), 55_000, 10.0);

        // Approach from south to cross going north again
        timer.update_ekf((0.0, -5.0), (0.0, -10.0), 60_000, 10.0);

        // Complete lap: from y=-5 to y=+5 at t=60000-60010ms
        // Crossing at y=0 is fraction t=0.5
        // crossing_time = 60000 + 0.5 * (60010-60000) = 60005ms
        // Lap time = 60005 - 5 = 60000ms
        let flags = timer.update_ekf((0.0, 5.0), (0.0, 10.0), 60_010, 10.0);

        assert!(flags & NEW_LAP != 0, "Should have NEW_LAP flag");
        assert!(flags & NEW_BEST != 0, "Should have NEW_BEST flag");
        assert_eq!(timer.lap_count(), 1);

        // Lap time should be 60005 - 5 = 60000ms
        assert_eq!(timer.timing.best_lap_ms, Some(60_000));
    }

    #[test]
    fn test_accumulated_flags() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());
        timer.set_gps_reference(0.0, 0.0);

        // Initially no flags
        assert_eq!(timer.take_accumulated_flags(), FLAG_NONE);

        // Cross line - should accumulate CROSSED_START
        timer.update_ekf((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        timer.update_ekf((0.0, 5.0), (0.0, 10.0), 10, 10.0);

        // Flags should be accumulated
        let flags = timer.take_accumulated_flags();
        assert!(flags & CROSSED_START != 0);

        // After taking, should be cleared
        assert_eq!(timer.take_accumulated_flags(), FLAG_NONE);
    }

    #[test]
    fn test_fallback_without_local_track() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());

        // Don't set GPS reference - should use GPS-based update
        assert!(!timer.has_local_track());

        // GPS-based update should work
        timer.update(-0.00005, 0.0, (0.0, 10.0), 0, 10.0);
        let flags = timer.update(0.00005, 0.0, (0.0, 10.0), 100, 10.0);

        assert!(flags & CROSSED_START != 0, "GPS fallback should work");
        assert_eq!(timer.timing.state, LapTimerState::Timing);
    }

    #[test]
    fn test_update_ekf_stationary_ignored() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());
        timer.set_gps_reference(0.0, 0.0);

        // Try crossing while nearly stationary
        timer.update_ekf((0.0, -0.1), (0.0, 0.1), 0, 0.1);
        let flags = timer.update_ekf((0.0, 0.1), (0.0, 0.1), 10, 0.1);

        assert_eq!(flags, FLAG_NONE, "Should ignore crossing when stationary");
        assert_eq!(timer.timing.state, LapTimerState::Armed);
    }

    #[test]
    fn test_update_ekf_point_to_point() {
        let mut timer = LapTimer::new();
        timer.timing.min_lap_time_ms = 1000;
        // Start line at lat=0, finish line at lat=0.001 (~111m north)
        timer.configure_point_to_point(test_line_at_lat(0.0), test_line_at_lat(0.001));
        timer.set_gps_reference(0.0, 0.0);

        // Start line is at y=0, finish line is at y≈111m

        // Cross start line
        timer.update_ekf((0.0, -5.0), (0.0, 10.0), 0, 10.0);
        let flags1 = timer.update_ekf((0.0, 5.0), (0.0, 10.0), 10, 10.0);
        assert!(flags1 & CROSSED_START != 0);
        assert_eq!(timer.timing.state, LapTimerState::Timing);

        // Drive towards finish
        timer.update_ekf((0.0, 50.0), (0.0, 10.0), 15_000, 10.0);
        timer.update_ekf((0.0, 105.0), (0.0, 10.0), 28_000, 10.0);

        // Cross finish line (at y≈111m)
        timer.update_ekf((0.0, 106.0), (0.0, 10.0), 29_000, 10.0);
        let flags2 = timer.update_ekf((0.0, 116.0), (0.0, 10.0), 30_000, 10.0);

        assert!(flags2 & CROSSED_FINISH != 0);
        assert!(flags2 & NEW_LAP != 0);
        assert_eq!(timer.lap_count(), 1);
        assert_eq!(timer.timing.state, LapTimerState::Armed);
    }

    #[test]
    fn test_clear_resets_local_track() {
        let mut timer = LapTimer::new();
        timer.configure_loop(test_line());
        timer.set_gps_reference(0.0, 0.0);

        assert!(timer.has_local_track());

        timer.clear();

        assert!(!timer.has_local_track());
        assert!(timer.prev_ekf_pos.is_none());
        assert_eq!(timer.accumulated_flags, FLAG_NONE);
    }
}
