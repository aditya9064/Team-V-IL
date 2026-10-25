# Observation-only camera diagnostic

CPU-reviewed, unrun diagnostic for the provided table/side/wrist views at640×360. Six global render-only refreshes,21 capture transactions, no pickup or physics after observation starts. It records native image/reference-clock association without inventing an offset. Pixel freshness and real-camera calibration remain unqualified.

The earlier pickup experiment stopped before dispatch when saved camera metadata was100ms old against the unchanged33.334ms limit. This diagnostic investigates that association; it does not claim the renderer is faulty or repair old reports. It depends on the common modules/fixtures in ../historical-assembly-baseline and the separately obtained official SDK.
