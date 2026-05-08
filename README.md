# Lunar Rover Hazard-Aware Navigation and Simulation Using MATLAB

**Author:** Ayush Bhatt  
**Platform:** MATLAB R2025a  
**Project Type:** Lunar surface image simulation, hazard modeling, autonomous rover path planning, and visualization

---

## Overview

This project presents a MATLAB-based simulation framework for autonomous lunar rover navigation across cratered lunar terrain. The system generates or loads a lunar surface image, models craters and boulders, builds multiple hazard layers, creates a weighted terrain cost map, and computes an optimized rover route using A* path planning. Dijkstra's algorithm is also implemented for comparison.

The project improves upon a basic crater-detection and shortest-path model by treating craters as hazards instead of navigation nodes. Rather than simply connecting crater centers, the rover plans a safer route through the terrain by considering crater interiors, crater rims, boulders, roughness, slope, shadow risk, and distance from hazardous regions.

---

## Key Features

- Synthetic lunar terrain generation from scratch
- Optional real lunar image input support
- Crater modeling and crater feature extraction
- Crater radius, rim strength, contrast, and risk-score analysis
- Crater core, crater rim, boulder, slope, and shadow hazard maps
- Weighted terrain cost-map generation
- Hazard-clearance estimation using distance from hazardous regions
- A* path planning implemented from scratch
- Dijkstra path planning implemented from scratch for comparison
- 2D final route visualization
- 3D lunar terrain reconstruction with rover path
- Rover telemetry analysis along the route
- Animated rover mission simulation
- Automatic CSV, PNG, and text-report export

---

## Project Motivation

Future lunar exploration missions require autonomous systems that can safely navigate unknown and hazardous terrain. Lunar rovers must avoid craters, steep slopes, rocks, rough patches, and shadowed regions where sensing and movement can become risky.

This project simulates that problem using MATLAB. It converts a lunar surface into a risk-aware navigation environment and demonstrates how a rover can choose a safer route from a landing site to a target site.

---

## Methodology

The simulation follows this complete pipeline:

1. **Input or Terrain Generation**  
   The program either loads a real lunar image named `chandrayaan_image.jpg` or generates a synthetic lunar surface with craters, rocks, shadows, terrain noise, and elevation features.

2. **Image Preprocessing**  
   The surface image is smoothed, contrast-stretched, enhanced, and converted into gradient and edge maps.

3. **Crater Modeling and Feature Extraction**  
   Crater centers and radii are modeled or detected. Each crater is evaluated using radius, rim strength, contrast, area, and risk score.

4. **Hazard Map Construction**  
   The system creates separate hazard layers for crater cores, crater rims, boulders, steep slopes, and shadow-risk areas.

5. **Cost Map Generation**  
   A final rover cost map is generated using terrain roughness, slope, shadow risk, and distance from hazards.

6. **Path Planning**  
   A* search is used to compute the main rover route. Dijkstra's algorithm is also used for performance comparison.

7. **Visualization and Simulation**  
   The project generates 2D maps, 3D terrain, mission dashboard, telemetry plots, and an animated rover simulation.

---

## Algorithms Used

### A* Search

A* is used as the primary path-planning algorithm. It combines the accumulated travel cost from the start point with a heuristic estimate of the remaining distance to the goal.

The planner prefers low-cost terrain and avoids high-risk regions.

### Dijkstra's Algorithm

Dijkstra's algorithm is implemented as a baseline comparison. Since Dijkstra does not use a goal-directed heuristic, it usually expands more nodes than A*.

### Cost Function

The rover movement cost is based on:

- Terrain roughness
- Terrain slope
- Shadow risk
- Near-hazard penalty
- Crater and boulder hazard zones

This makes the route more realistic than a simple shortest-distance path.

---

## Repository Structure

```text
Lunar-Rover-Hazard-Aware-Navigation/
│
├── README.md
├── lunar_rover_sim.m
├── .gitignore
├── LICENSE
│
├── docs/
│   ├── Ayush_Bhatt_Lunar_Rover_Research_Paper.pdf
│   └── Ayush_Bhatt_Lunar_Rover_Research_Paper.docx
│
├── results/
│   ├── 01_preprocessing_pipeline.png
│   ├── 02_crater_detection_features.png
│   ├── 03_hazard_model.png
│   ├── 04_cost_map_components.png
│   ├── 05_final_2d_path.png
│   ├── 06_3d_terrain_route.png
│   ├── 07_mission_dashboard.png
│   ├── 08_route_telemetry.png
│   └── 09_rover_animation_final_frame.png
│
├── data/
│   └── chandrayaan_image.jpg
│
└── legacy/
    ├── old_python_code.py
    └── old_project_presentation.pdf
```

---

## How to Run

1. Open MATLAB R2025a.
2. Place `lunar_rover_sim.m` in your project folder.
3. Optional: place a real lunar image in the same folder and name it:

```text
chandrayaan_image.jpg
```

4. Run this command in the MATLAB Command Window:

```matlab
lunar_rover_sim
```

The project will automatically create an output folder named:

```text
results_lunar_rover_peak
```

---

## Output Files

After running the simulation, the following files are generated:

```text
results_lunar_rover_peak/
├── 01_preprocessing_pipeline.png
├── 02_crater_detection_features.png
├── 03_hazard_model.png
├── 04_cost_map_components.png
├── 05_final_2d_path.png
├── 06_3d_terrain_route.png
├── 07_mission_dashboard.png
├── 08_route_telemetry.png
├── 09_rover_animation_final_frame.png
├── detected_craters.csv
├── planned_rover_path.csv
├── mission_metrics.csv
└── project_summary.txt
```

---

## Sample Results

### Preprocessing Pipeline

![Preprocessing Pipeline](results/01_preprocessing_pipeline.png)

### Crater Detection and Feature Extraction

![Crater Detection](results/02_crater_detection_features.png)

### Hazard Model

![Hazard Model](results/03_hazard_model.png)

### Terrain Cost Map

![Cost Map](results/04_cost_map_components.png)

### Final Rover Route

![Final Rover Path](results/05_final_2d_path.png)

### 3D Terrain Route

![3D Terrain Route](results/06_3d_terrain_route.png)

### Mission Dashboard

![Mission Dashboard](results/07_mission_dashboard.png)

### Route Telemetry

![Route Telemetry](results/08_route_telemetry.png)

### Rover Animation Final Frame

![Rover Animation](results/09_rover_animation_final_frame.png)

---

## Mission Metrics

The simulation reports:

- Number of modeled or detected craters
- Planned path length
- A* path cost
- Estimated rover energy usage
- Estimated mission time
- Average terrain roughness
- Average slope
- Minimum hazard clearance
- Hard-hazard crossing percentage
- A* nodes expanded
- Dijkstra nodes expanded
- A* efficiency improvement over Dijkstra

---

## Research Contribution

The main contribution of this project is a complete MATLAB-based lunar rover navigation pipeline that transforms crater detection into hazard-aware route planning.

Instead of treating craters as points in a graph, the system treats them as unsafe regions. This allows the rover to plan a route that considers both distance and terrain safety.

The project demonstrates how image processing, terrain modeling, graph search, and simulation can be combined to support autonomous navigation research for planetary exploration.

---

## Technologies Used

- MATLAB R2025a
- Image processing using matrix operations
- Synthetic terrain modeling
- A* search
- Dijkstra search
- 2D visualization
- 3D terrain visualization
- Rover telemetry simulation

---

## Main File

```text
lunar_rover_sim.m
```

This is the main MATLAB script. It is designed as a single-file project so that it can be copied, shared, and run easily without missing helper-function errors.

---

## Optional Input Image

The project works without external data because it can generate its own synthetic lunar terrain.

To test the model on a real lunar image, add:

```text
chandrayaan_image.jpg
```

in the same folder as the MATLAB script.

---

## Limitations

- The synthetic lunar terrain is a simulation and not a true digital elevation model.
- The 3D elevation map is estimated from image intensity and procedural terrain generation.
- Real mission-grade rover planning would require calibrated sensor data, DEM maps, rover dynamics, slip modeling, and physical validation.
- The hazard map is based on visual and simulated terrain features rather than real rover sensor fusion.

---

## Future Work

Possible future improvements include:

- Using real lunar DEM data
- Adding rover kinematics and wheel slip modeling
- Adding multi-objective optimization
- Adding dynamic obstacle detection
- Using deep learning for crater segmentation
- Adding Simulink-based rover motion control
- Exporting animation videos automatically
- Adding GUI controls for start and goal selection

---

## License

This project is intended for academic, research, and educational demonstration purposes.

---

## Author

**Ayush Bhatt**

Project: Lunar Rover Hazard-Aware Navigation and Simulation Using MATLAB
