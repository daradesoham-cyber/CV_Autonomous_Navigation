# Nav2 Navigation & Path Planning

## Configuration
* **Global Planner**: NavfnPlanner (`nav2_navfn_planner/NavfnPlanner`) utilizing Dijkstra/A* on grid costmaps.
* **Local Controller**: RegulatedPurePursuitController (`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`) with adaptive velocity scaling based on path curvature and proximity to obstacles.
* **Costmaps**:
  * Global Costmap: Static layer (`complex_map.yaml`), Obstacle layer (`/scan`), and Inflation layer ($r_{\text{inf}} = 0.55\text{ m}$).
  * Local Costmap: Rolling window ($6\text{ m} \times 6\text{ m}$), Obstacle layer, and Inflation layer.
* **Recovery Behaviors**: Spin, BackUp, and Wait plugins configured in `behavior_server`.
