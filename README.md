# Modular Pipeline Report Summary

## **🏆 Achieved 1st Place!**
Through careful implementation and optimization, our modular pipeline outperformed CNN-based models, securing **1st place** in the competition.

## 1. Overall Flow
- Inspired by human driving behavior, the process follows **lane_detection → waypoint_prediction → lateral_control & longitudinal_control**.
- **lane_detection.py**: Detects road conditions (lanes, obstacles, etc.).
- **waypoint_prediction.py**: Generates a driving path based on detected information.
- **lateral_control.py**: Adjusts steering.
- **longitudinal_control.py**: Controls acceleration and braking.

## 2. Module Descriptions
### lane_detection
- **cut_gray**: Crops and converts the front-view image to grayscale.
- **edge_detection**: Identifies lane boundaries using gradient-based edge detection.
- **find_maxima_gradient_rowwise**: Detects local maxima per row to locate lane edges.
- **find_first_lane_point**: Identifies the initial lane boundary points.

### waypoint_prediction
- **curvature**: Calculates the curvature of waypoints to evaluate path smoothness.
- **waypoint_prediction**: Generates waypoints using two methods: ‘center’ (direct midpoint) & ‘smooth’ (optimized via smoothing_objective function).
- **target_speed_prediction**: Predicts target speed based on waypoints and curvature.

### lateral_control
- **stanley_control**: Implements the Stanley controller to compute steering angles based on crosstrack and orientation errors.

### longitudinal_control
- **PID_step**: Uses a PID controller to regulate vehicle speed towards the target speed.
- **control**: Converts PID control signals into acceleration/braking inputs.

## 3. Agent Performance Analysis
- **Target Speed 60 vs 80**: Higher max_speed allows quicker adaptation to curves.
- **PID Parameter Tuning**
  - **Increased P**: Higher risk of lane departure on sharp turns.
  - **Increased I**: Helps correct accumulated errors but struggles in curves.
  - **Increased D**: Slows down adjustments.
  - **Lower damping**: Improves sharp turn adaptation.

## 4. Challenges & Solutions
- **lane_detection**: Adjusted cut_size to 67 to prevent vehicle body interference.
- **waypoint_prediction**: Set offset_speed to 20 to better handle sharp turns.
- **lateral_control**: Added crosstrack error correction to prevent lane departure.

This modular pipeline proved to be significantly superior to CNN-based models, leading to our **1st place** victory!
