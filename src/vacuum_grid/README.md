# vacuum_grid

Shared occupancy-grid toolbox, pure Python (numpy only, no ROS). `GridView(cells, resolution, origin_x,
origin_y)` gives world/cell conversion (row 0 = lowest y, cell centres, floor) and
`inflated_obstacles(radius_m, include_unknown=False)` (Euclidean disk around OCCUPIED cells; UNKNOWN only on
request; the grid border is not an obstacle). Used by `vacuum_mapping` (`FrontierFinder`) and, later, by the
coverage planner of spec 005. Tests: `cd src/vacuum_grid && python3 -m pytest -q test`.
