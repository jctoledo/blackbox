# Data Source and Processing
The original center lines were fetched as GPS points from the OpenStreetMap project (https://www.openstreetmap.org).
We applied a smoothing algorithm to the center lines. The track widths were extracted from satellite images using an
image processing algorithm. The race lines were calculated using our minimum curvature optimization algorithm, which is
also available as open source software on GitHub (https://github.com/TUMFTM/global_racetrajectory_optimization).

# Content and Data Format
- tracks: [x_m, y_m, w_tr_right_m, w_tr_left_m] These files contain the center line (x, y) and the track widths to the
right (w_tr_right) and left (w_tr_left). The center lines were smoothed. Therefore, they do not lie perfectly in the
middle of the track anymore.
- racelines: [x_m, y_m] The provided race lines were calculated minimizing the summed curvature. They lie within the
track boundaries. The racelines folder also contains a plot of the optimized race line and a plot of the according
curvature profile for every track.

The quality of the source data (GPS points and satellite images) varies greatly depending on the location. Accordingly,
the quality of the results varies as well. Please bear this in mind and check whether the accuracy is sufficient for the
desired application.
