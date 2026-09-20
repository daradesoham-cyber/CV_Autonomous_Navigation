# LiDAR-Camera Sensor Fusion

## Theory & Angular Projection
A 2D LiDAR provides high-precision radial distance measurements across $360^\circ$ but lacks semantic class information. Conversely, an RGB camera provides rich semantic classifications and 2D bounding boxes but lacks direct depth data.

The fusion node performs **angular sector association**:
1. Camera horizontal field of view: $\text{HFOV} = 1.089\text{ rad} \approx 62.4^\circ$.
2. For each detected bounding box with horizontal boundaries $[x_{\min}, x_{\max}]$ on an image of width $W$:
   $$\theta_{\text{center}} = \left(0.5 - \frac{x_{\min} + x_{\max}}{2W}\right) \times \text{HFOV}$$
   $$\Delta\theta = \left(\frac{x_{\max} - x_{\min}}{W}\right) \times \text{HFOV}$$
3. LiDAR scan index bounds are computed:
   $$i_{\text{start}} = \left\lfloor \frac{(\theta_{\text{center}} - \Delta\theta/2) - \theta_{\min}}{\Delta\theta_{\text{scan}}} \right\rfloor$$
   $$i_{\text{end}} = \left\lfloor \frac{(\theta_{\text{center}} + \Delta\theta/2) - \theta_{\min}}{\Delta\theta_{\text{scan}}} \right\rfloor$$
4. Valid ranges within $[i_{\text{start}}, i_{\text{end}}]$ are filtered for outliers.
5. The 20th percentile distance is extracted to accurately represent the front surface of the obstacle.

## Output
Publishes `/vision/objects` with physical distance and bearing, and `/vision/obstacles` with clearance requirement flags.
