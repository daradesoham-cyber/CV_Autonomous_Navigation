# V3 Realistic Hospital Asset Inventory (Phase 1)

Date: 2026-09-30 · Branch: `v3.0-dev`
Target environment: Ubuntu 26.04, ROS 2 Lyrical, Gazebo Sim 10.5.0.

## 1. Source and license verification

| Source | What it provides | License (verified) | Redistribution / modification | Used in V3 |
|---|---|---|---|---|
| Gazebo Fuel, `OpenRobotics` collection (`https://fuel.gazebosim.org/1.0/OpenRobotics/models/<name>`) | The medical props that the reference repo and the AWS hospital world download at setup time (`aws-robomaker-hospital-world/setup.sh`, `fuel_utility.py`) | **CC-BY 4.0**, read per model from the Fuel API `license_name` field | Allowed with attribution. Attribution is in `V3/models/ATTRIBUTION.md`. | **Yes, 16 models** |
| `aws-robotics/aws-robomaker-hospital-world` (`ros2` branch; the repo was archived 2025-09-10) | Building shell, nurses station, hospital sign, curtains, elevator, residential décor | MIT-0 (`LICENSE`: no attribution clause) | Allowed | **No.** The nurses station is 4.8 × 7.0 × 3.0 m (Collada, Y-up, cm), far too large for the V2.6 reception slot (1.8 × 0.6 m). The remaining assets are building shell or décor with no CV or navigation value. |
| `TommasoVandermeer/Hospitalbot-Path-Planning` | Copies of the above plus Pioneer 3AT and Hokuyo models | **No LICENSE file** | Not established | **No.** The assets were taken from the upstream sources instead. |

Every Fuel model was downloaded with `gz fuel download` (Gazebo Sim 10 toolchain) and vendored by `V3/scripts/vendor_hospital_assets.py`.

## 2. Compatibility audit of the Fuel models

I checked all 17 downloaded models (16 used; `CGMClassic` was downloaded but not used):

| Check | Result |
|---|---|
| Gazebo plugins | **0** in every model |
| OGRE-1 `<material><script>` (Classic-only) | **0** |
| Mesh format | Wavefront OBJ + MTL + PNG textures (loads in gz-rendering / ogre2) |
| Up-axis / origin | Z-up, origin on the floor for every model (checked from vertex bounds) |
| Scale | Physically plausible, measured from the collision meshes (see table) |
| Collision geometry | Every model has its own collision mesh (a `_Col.obj` simplified mesh or the visual mesh) |
| Mesh URIs | Absolute `https://fuel.gazebosim.org/...` URIs, so the models depend on the network |

Conversion and adaptation performed (the only changes made):
1. Mesh URIs rewritten to relative `meshes/<file>`, so the models load offline.
2. `<static>true</static>` forced on every model, because props are scenery.
3. **PatientWheelChair**: the upstream collision mesh is rotated 90° relative to the visual mesh (1.13 × 0.65 m versus 0.64 × 1.12 m) and is 0.5 m shorter. It was replaced with a box fitted to the visual mesh bounds (1.118 × 0.637 × 1.779 m).

I did not import any Classic launch files, `gazebo_ros` plugins, `SpawnEntity` services or `.world` files.

## 3. Placed objects

Footprints are the world-frame collision bounds computed from the vendored collision geometry (`V3/worlds/hospital_logistics_props.json`). Height is the collision-mesh top.

| Object (world name) | Model | Source / license | Visual mesh | Collision | Size L×W×H (m) | Pose x, y, yaw | Static / dynamic | Expected CV role | Placement / replaces |
|---|---|---|---|---|---|---|---|---|---|
| v3_patient_bed_1 | MalePatientBed | Fuel / CC-BY 4.0 | MalePatientBed.obj | same mesh | 2.14×1.09×1.08 | 7.45, −8.8, 0 | static | hospital bed with patient (target classes `hospital_bed`, `person`) | ward; replaces `hospital_bed_1` |
| v3_trolley_bed_2 | TrolleyBed | Fuel / CC-BY 4.0 | TrolleyBed.obj | same mesh | 1.98×0.85×0.85 | 7.5, −11.0, 0 | static | hospital bed / trolley | ward; replaces `hospital_bed_2` |
| v3_bedside_drawer_1/2 | Drawer | Fuel / CC-BY 4.0 | Drawer.obj | same mesh | 0.41×0.50×0.63 | 8.95, −8.8 / −11.0 | static | bedside cabinet (clutter) | ward; replaces `bedside_table_1/2` |
| v3_iv_stand_ward | IVStand | Fuel / CC-BY 4.0 | IVStand.obj | same mesh | 0.44×0.45×1.41 | 9.1, −9.9, 0 | static | IV stand; thin-pole LiDAR case | ward (new) |
| v3_collection_table | AdjTable | Fuel / CC-BY 4.0 | AdjTable.obj | AdjTable_Col.obj | 0.81×1.60×0.81 | 4.1, −10.6, 90° | static | sample collection station | ward west wall (new) |
| v3_collection_bp_monitor | BloodPressureMonitor | Fuel / CC-BY 4.0 | BloodPressureMonitor.obj | _Col.obj | 0.57×0.54×1.48 | 4.0, −8.2, 0 | static | medical device landmark at the collection station | ward west wall (new) |
| v3_ward_bp_cart | BPCart | Fuel / CC-BY 4.0 | Cart_BP.obj | Cart_BP_Col.obj | 0.59×0.69×1.60 | 5.6, −12.2, 0 | static | medical cart | ward south wall (new) |
| v3_ward_nurse | Scrubs | Fuel / CC-BY 4.0 | scrubs.obj | Scrubs_Col.obj | 0.56×0.32×1.62 | 4.3, −12.2, 90° | static | person (nurse) | ward (new) |
| v3_waiting_chair_w1/w2 | Chair | Fuel / CC-BY 4.0 | Chair.obj | same mesh | 0.51×0.58×0.82 | −2.95 / −2.30, −10.0 | static | hospital chair (no chair class in V2.5 model) | reception; replaces `waiting_bench_w` |
| v3_waiting_chair_e1/e2 | Chair | Fuel / CC-BY 4.0 | Chair.obj | same mesh | 0.51×0.58×0.82 | 2.40 / 3.05, −9.9 | static | hospital chair | reception; replaces `waiting_bench_e` (moved 0.4 m south, see §5) |
| v3_reception_nurse | Scrubs | Fuel / CC-BY 4.0 | scrubs.obj | Scrubs_Col.obj | 0.57×0.32×1.62 | −2.2, −7.4, −90° | static | person behind the reception desk | reception (new) |
| v3_visitor_phone | MaleVisitorOnPhone | Fuel / CC-BY 4.0 | MaleVisitorStatic.obj | _Col.obj | 0.45×0.50×1.75 | 2.9, −10.8, 180° | static | person (visitor) | reception (new) |
| v3_parked_wheelchair | PatientWheelChair | Fuel / CC-BY 4.0 | PatientWheelChair.obj | **box (fixed)** | 1.12×0.64×1.78 | 2.8, −11.9, 0 | static | wheelchair with patient | reception (new) |
| v3_lab_instrument_cart | InstrumentCart1 | Fuel / CC-BY 4.0 | InstrumentCart1.obj | same mesh | 0.61×0.52×1.38 | −9.0, 12.3, 0 | static | laboratory instrument cart | lab north wall (new) |
| v3_lab_cabinet | MetalCabinet | Fuel / CC-BY 4.0 | MetalCabinet.obj | same mesh | 0.81×0.45×1.68 | −8.0, 12.55, 0 | static | medical/lab cabinet | lab north wall (new) |
| v3_lab_surgical_trolley | SurgicalTrolley | Fuel / CC-BY 4.0 | trolley.obj | same mesh | 0.88×0.37×0.70 | −10.1, 12.4, 0 | static | medical trolley | lab north wall (new) |
| v3_storage_rack | StorageRack | Fuel / CC-BY 4.0 | StorageRack.obj | same mesh | 1.26×0.71×1.74 | −10.0, 3.0, 0 | static | supply storage rack | storage; replaces `storage_box_stack` |
| dynamic_person | Scrubs (visual only) | Fuel / CC-BY 4.0 | scrubs.obj | V2.6 cylinder r = 0.25 (unchanged) | 1.7 m | V2.6 pose / patrol | **dynamic** (V2.6 VelocityControl, `/person/cmd_vel`) | moving person (YOLO `person`) | V2.6 model, visual replaced |
| dynamic_hospital_trolley | SurgicalTrolley (visual only) | Fuel / CC-BY 4.0 | trolley.obj | V2.6 box 0.9×0.6×0.7 (unchanged) | 0.7 m | V2.6 pose / patrol | **dynamic** (`/trolley/cmd_vel`) | moving medical trolley | V2.6 model, visual replaced |

**Logistics markers** (V3-authored, primitives plus generated textures):

| Object | Pose | Collision | Purpose |
|---|---|---|---|
| v3_floor_collection_point | 5.0, −9.0 (floor, 0.9 × 0.9 m, red) | none (visual only, 4 mm) | COLLECTION_POINT stop marker |
| v3_floor_laboratory_test_point | −7.0, 9.5 (floor, blue) | none | LABORATORY / TEST_POINT stop marker |
| v3_sign_collection_point | 3.62, −9.6, z 0.85, on the ward west wall | 0.8 × 0.03 × 0.4 box | "COLLECTION / SAMPLE PICKUP" plaque (`V3/models/v3_markers/collection_point.png`) |
| v3_sign_laboratory_test_point | −7.6, 10.95, z 1.12, standing on lab_bench_2 | 0.8 × 0.03 × 0.4 box | "LABORATORY / TEST POINT" plaque |
| v3_payload_blood_sample_carrier | 4.1, −10.3, on the collection table | 0.26 × 0.18 × 0.14 box | logical payload marker (red carrier with white lid band) |

**Kept from V2.6 as primitives (no suitable realistic asset in the verified CC-BY set):** reception desk (the AWS nurses station was rejected because of its size), lab benches (the Fuel set has no lab bench), warehouse racks, pallets, office furniture, cafeteria furniture, the `dynamic_warehouse_cart` and `dynamic_forklift` (no forklift or industrial-cart asset in the set), walls and pillars. Doors in V2.6 are open doorways; no door models were added in Phase 1.

## 4. Logistics locations

Both locations reuse V2.6 topology destination nodes that were already validated:

| Location | x | y | Stop yaw | V2.6 node |
|---|---|---|---|---|
| COLLECTION_POINT | 5.0 | −9.0 | π (facing the collection wall) | `Hospital Medical Ward` |
| LABORATORY / TEST_POINT | −7.0 | 9.5 | π/2 | `Research Laboratory` |

## 5. Placement corrections made during validation

| Issue found by `V3/tests/test_v3_world_geometry.py` | Fix |
|---|---|
| `v3_waiting_chair_e1` at (2.2, −9.5) intersected `pillar_rec_2` (V2.6's `waiting_bench_e` also overlapped this pillar) | east chairs moved to y = −9.9 |
| `v3_visitor_phone` at (3.3, −10.8) intersected `wall_hosp_ward_w` | moved to x = 2.9 |
| Collection plaque at z = 1.35 was outside the camera's vertical FOV from the collection point (image std 9.0, blank wall) | lowered to z = 0.85 |

## 6. Phase 1 validation results (measured on 2026-09-30)

All results come from running the actual V3 world (`ros2 launch hospital_logistics v3_world.launch.py headless:=true`) on this machine (RTX 3050 Laptop GPU).

| # | Test | Command | Result |
|---|---|---|---|
| T1 | Build | `colcon build --packages-select hospital_logistics autonomous_robot_gazebo autonomous_robot_description` | **PASS**: 3 packages built. Stderr contains only CMake `cmake_minimum_required` deprecation warnings that were already there in V2.6. |
| T2 | SDF validity | `GZ_SIM_RESOURCE_PATH=V3/models gz sdf -k V3/worlds/hospital_logistics_world.sdf` | **PASS**: `Valid.` |
| T3 | Gazebo startup and robot spawn | V3 world launch | **PASS**: `Entity creation successful`. The log has no model/URI errors; its only warnings are `libEGL warning: failed to create dri2 screen` (headless EGL, harmless). |
| T4 | Camera | `V3/tests/test_v3_world_sensors.py` | **PASS**: `/camera/image_raw` at 19.2–23.2 Hz. Ten viewpoint frames were saved; `docs/evidence/phase1_camera_contact_sheet.png`. |
| T5 | LiDAR | same | **PASS**: `/scan` at 20.0 Hz, 538–701 finite beams of 720 per viewpoint. |
| T6 | Props render | visual inspection of the 10 frames | **PASS**: beds, IV stand, BP monitor, BP cart, collection table, chairs, wheelchair, nurses, visitor, storage rack, lab plaque, collection plaque (after the z fix) and payload carrier render with their textures. The dynamic person and trolley render as the Scrubs and SurgicalTrolley meshes. |
| T7 | LiDAR geometry of props | same (ray against the collision footprint, with walls, furniture and other props as occluders) | Props with line of sight: **VISIBLE** means within 0.30 m of the geometric distance; the counts per viewpoint are in `phase1_sensor_validation.json`. **8 MISMATCH cases** (patient bed, IV stand, BP cart, standing visitor) all measure 0.31–0.51 m **further** than the bounding box, because the 2D LiDAR plane (z = 0.235 m) passes between legs and wheels. There were no phantom returns nearer than the geometry. |
| T8 | Static geometry | `V3/tests/test_v3_world_geometry.py` | **PASS**: G1 prop intersections 23/23, G2 topology node clearance 53/53, G3 topology edge clearance 55/55 (worst 0.866 m, BP monitor vs. the ward-entry edge), G4 V2.6 doorways 8/8 unobstructed, G5 logistics points 2/2 (collection 0.877 m, test point 0.791 m), G6 static map 108/108. |
| T9 | Nav2 through the new layout | `V3/tests/test_v3_world_navigation.py --reset` (V2.6 AMCL + Nav2, V3 map) | **PASS**. spawn → COLLECTION_POINT: SUCCEEDED, 33.1 s, 11.99 m, final map error 0.29 m. COLLECTION_POINT → LAB/TEST_POINT: SUCCEEDED, 49.6 s, 23.43 m, final error 0.305 m. The V2.6 decision engine logged the node sequence reception → ward entry → ward, then ward entry → east corridor → central spine → lab bypass → junction 8 → lab. This is a single run and a smoke test, not a benchmark. |

**CV-readiness observation (not an accuracy result).** The validated V2.5 model (`models/yolov8n_v25.pt`, conf 0.35) was run on the ten frames (`V3/tests/test_v3_yolo_frames.py`, `docs/evidence/phase1_yolo_contact_sheet.png`). The notes below come from my visual inspection of the annotated frames; no labels exist yet:
- `person` fired on all six rendered humans (two nurses, the visitor, the wheelchair occupant, the dynamic nurse, and the reception nurse).
- `directional_sign` fired on the corridor signs and both V3 plaques, plus a false positive on the BP monitor.
- `hospital_bed` **missed both realistic beds**. It fired instead on lab benches, the reception desk, a blank bench face (0.94) and the storage rack.
- Chairs were labelled `cart` and the surgical trolley `forklift`. The model has no wheelchair or IV-stand class.

This demonstrates a synthetic-to-realistic domain gap in the V2.5 model and is the evidence-based requirement for model adaptation in Phase 2.

## 7. Remaining asset issues

1. The V2.5 YOLO classes do not cover wheelchair, IV stand, chair or medical cart, and `hospital_bed` does not transfer to realistic beds (§6). This is a Phase 2 item.
2. The 2D LiDAR sees only the legs of beds and carts. The global costmap is protected by the re-stamped static map, but the **local costmap has no static layer**, so overhang protection near beds depends on the `semantic_obstacles` (camera fusion) observation source. This is to be verified in Phase 2.
3. The minimum LiDAR range during both Nav2 legs (0.276 m and 0.283 m) was at the V2.6 ward north doorway (≈ 5.1, −6.7), a V2.6 wall corner rather than a V3 prop. It needs watching in later mission runs.
4. At the test point stop pose the camera faces the side of `lab_bench_2` at about 1.1 m, so the lab plaque sits above the camera's view there. It is visible on the approach (`lab_junction_northwest` frame).
5. Doors are open doorways as in V2.6. No door models were added.
6. The reception desk, lab benches, warehouse furniture, forklift and warehouse cart remain V2.6 primitives, because the verified CC-BY set has no suitable asset for them.
