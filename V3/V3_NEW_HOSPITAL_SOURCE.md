# V3 New Hospital: Source

| Item | Value |
|---|---|
| Building / floor plan | AWS RoboMaker Hospital World |
| URL | https://github.com/aws-robotics/aws-robomaker-hospital-world |
| Version | branch `ros2`, commit `7161eb8448f5cba6a469da7a79ae5d660b0b7f58` (2021-07-29), vendored in `V3/third_party/aws-robomaker-hospital-world` |
| License | MIT-0 (MIT No Attribution): free use, modification and redistribution |
| Furniture / people | OpenRobotics collection on Gazebo Fuel (https://fuel.gazebosim.org/1.0/OpenRobotics), 43 models, CC-BY 4.0 (attribution in `V3/hospital/models/ATTRIBUTION.md`) |
| Format | Gazebo Classic world plus COLLADA/OBJ meshes, converted to Gazebo Sim 10 by `V3/scripts/build_aws_hospital_world.py` |

## Why it was selected

- It's a complete, realistic single-floor hospital (25 × 56 m). It has a main entrance and elevator lobby, a reception desk, waiting areas, a nursing station, patient rooms, curtained wards, surgery and X-ray, restrooms, a recovery bay, a staff lounge and storage.
- It's made for robot simulation: its collision meshes match the visuals, it has real doorways, corridors and intersections, and there are several routes between wings.
- Its furniture is real hospital furniture: beds with patients, wheelchairs, IV stands, surgical and instrument trolleys, monitors, cabinets, waiting seating, and staff and visitors.
- Its license is permissive and clear (MIT-0, plus CC-BY 4.0 for the Fuel models), so it can be used and modified.
- It converts to Gazebo Sim without Classic plugins: static includes only, with Fuel URIs rewritten to local paths so it works offline.
- It's an entirely different building from the V2.6/old V3 facility. No old geometry, map, topology or coordinates are reused.

## Changes made for V3

- Collada centimetre units are honoured in map generation. Gazebo applies `<unit>` itself.
- Three upstream props saved at z = −7.6e8 were restored to the floor. The trolley bed was re-parked so it doesn't block the east corridor.
- The ceiling was left out, and point lights were added in its place.
- The NE storage room's warehouse clutter was replaced by laboratory furnishing.
- Added: logistics plaques, a collection table with payload, directional signs and moving traffic. These come from `V3/scripts/v3_hospital_additions.py` and `v3_hospital_layout.py`.
