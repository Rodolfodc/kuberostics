# vacuum_coverage

Real coverage of spec 005. Contains the pure-Python cores only (no ROS imports, D-18/D-19):

| Class | File | Role |
|---|---|---|
| `RegionBuilder` | `core/region_builder.py` | which cells a `CoverRoom` goal cleans (005-R16) |
| `CoveragePlanner` | `core/coverage_planner.py` | boustrophedon waypoints (005-R17) |
| `CoverageTracker` | `core/coverage_tracker.py` | covered mask and `covered_ratio` (005-R21) |
| `CoverageSession`, `ResumeStore` | `core/coverage_session.py` | retry-once-then-skip, timeouts, early success, resume (005-R20, R21, R32) |

Grid code is reused from `vacuum_mapping` (`Raster`, `erode`); `core/grid_ops.py` adds `dilate` and the
reachable/coverable masks. A shared `vacuum_grid` package is planned by T309 of spec 003.

The `coverage_server` LifecycleNode (T511 node half) is not here yet: it waits for the contract change of plan 005
(parameters, `coverage/path`, `coverage/covered`, room messages) and for the Nav2 bring-up (T510). The stub in
`vacuum_stubs` stays the default.

Test: `cd src/vacuum_coverage && PYTHONPATH=../vacuum_mapping python3 -m pytest test`.
