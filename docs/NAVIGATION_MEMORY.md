# Navigation Memory System & Topological Graph

## 1. Overview

The Navigation Memory System enables persistent spatial-semantic reasoning for autonomous mobile robots. Instead of treating every navigation run as a tabula rasa, the robot retains knowledge of:
1. **Topological Map**: Junctions, rooms, waypoints, and dead-end cul-de-sacs.
2. **Corridor Connectivity & Traversal Counts**: Frequency of usage and familiarity bonuses.
3. **Observed Semantic Signs**: Associations between junctions and directional guidance.
4. **Dynamic Obstacles & Blockages**: Persisted impassable corridors requiring detour/bypass routing.

---

## 2. SQLite Database Schema (`navigation_memory.db`)

The database is stored at:
`~/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db`

### Table: `nodes`
Stores topological vertices in map frame metric coordinates:
```sql
CREATE TABLE nodes (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    theta REAL DEFAULT 0.0,
    node_type TEXT DEFAULT 'junction',     -- 'start', 'junction', 'destination', 'dead_end', 'waypoint'
    semantic_label TEXT DEFAULT ''        -- e.g. 'HOSPITAL', 'WAREHOUSE', 'EXIT'
);
```

### Table: `edges`
Stores corridor segments with dynamic costs and blockage flags:
```sql
CREATE TABLE edges (
    from_node TEXT NOT NULL,
    to_node TEXT NOT NULL,
    distance REAL NOT NULL,
    cost REAL NOT NULL,
    traversal_count INTEGER DEFAULT 0,
    is_blocked INTEGER DEFAULT 0,
    is_dead_end INTEGER DEFAULT 0,
    PRIMARY KEY (from_node, to_node),
    FOREIGN KEY (from_node) REFERENCES nodes(id),
    FOREIGN KEY (to_node) REFERENCES nodes(id)
);
```

### Table: `signs`
Stores real-time visual sign detections:
```sql
CREATE TABLE signs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    node_id TEXT NOT NULL,
    text TEXT NOT NULL,                  -- 'HOSPITAL', 'WAREHOUSE', etc.
    direction TEXT NOT NULL,             -- 'RIGHT', 'LEFT', 'STRAIGHT'
    confidence REAL NOT NULL,
    timestamp REAL NOT NULL
);
```

### Table: `traversals`
Historical records of edge traversals:
```sql
CREATE TABLE traversals (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    from_node TEXT NOT NULL,
    to_node TEXT NOT NULL,
    success INTEGER NOT NULL,            -- 1: succeeded, 0: aborted/blocked
    duration REAL,
    timestamp REAL NOT NULL
);
```

### Table: `obstacles`
Stores dynamic obstacles that caused edge blockages:
```sql
CREATE TABLE obstacles (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    edge_from TEXT NOT NULL,
    edge_to TEXT NOT NULL,
    obstacle_class TEXT NOT NULL,        -- 'cart', 'box', 'pallet', etc.
    x REAL NOT NULL,
    y REAL NOT NULL,
    timestamp REAL NOT NULL
);
```

---

## 3. Cost Function & Dead-End Avoidance

The effective cost of traversing an edge $e = (u, v)$ is calculated as:
$$C_{\text{eff}}(e) = \begin{cases}
10^6 & \text{if } e.\text{is\_blocked} \\
10^5 & \text{if } e.\text{is\_dead\_end and target} \neq v \\
\max(0.1, e.\text{cost} - \min(0.15 \cdot \text{count}, 0.3) \cdot e.\text{distance}) & \text{otherwise}
\end{cases}$$

- **Dead-end avoidance**: The $10^5$ penalty prevents Dijkstra/A* from selecting any path containing a cul-de-sac unless the goal explicitly is that node.
- **Dynamic replanning**: When an obstacle is detected in an active corridor, the edge is set to `is_blocked = True` ($10^6$ cost), triggering Dijkstra to immediately compute an alternate route (e.g. West Bypass).
- **Familiarity bonus**: Corridors with high successful traversal counts receive a slight discount, encouraging reliable routes.

---

## 4. Resetting Navigation Memory

To clear dynamic observations (signs, traversals, obstacle blockages) while preserving topological structure:
```bash
./scripts/reset_navigation_memory.sh
```
Or in Python:
```python
from autonomous_robot_navigation.navigation_memory import NavigationMemory
mem = NavigationMemory()
mem.reset_memory()
```
