# ArmKinect: Human Arm Movement Reconstruction using Inverse Kinematics

This repository contains code for recreating human arm movement using motion capture data and implementing inverse kinematics. Below is a guide to the functionality and limitations of the code.

### Functionality

1. **Data Import and Filtering**: The code imports motion capture and electromyography (EMG) data from a text file. It applies low-pass and high-pass filters to the data for delete noise.

| Markers | EMG data |
|---------|----------|
| ![img1.png](/img/img1.png) | ![img2.png](/img/img2.png) |

2. **Movement Recreation**: The `recreate_mouvement` function reconstructs arm movement based on filtered motion capture data. It visualizes the movement in 3D space and calculates segment lengths.

<center>

![vid2.gif](/img/vid2.gif)

</center>

3. **Inverse Kinematics**: The `inverse_kinematics` function computes the joint angles necessary to reach a target position using the calculated segment lengths and the desired end-effector position. It employs numerical methods to iteratively refine the joint angles.

4. **Optimal Jerk Smoothing**: The `optimal_jerk` function applies optimal jerk smoothing to the computed joint angles to achieve smoother movement trajectories.

5. **Data Visualization**: Various plotting functions are provided to visualize raw and filtered data, arm movement trajectories, and the results of inverse kinematics.

<center>

![vid3.gif](/img/vid3.gif)

</center>

### Limitations

* **2D Movement Reconstruction**: The current implementation only supports 2D arm movement reconstruction. While the code calculates segment lengths and joint angles, it does not fully utilize 3D motion capture data.

* **Sensor Placement Sensitivity**: The accuracy of the inverse kinematics solution heavily depends on the precise placement of motion capture sensors. Variability in sensor placement can lead to discrepancies between the reconstructed movement and the actual movement.

### Usage

To use the code:

1. Ensure Python and required libraries (NumPy, Matplotlib, SciPy) are installed.
2. Place motion capture and EMG data files in the specified format.
3. Adjust parameters such as start and end times, filter types, and timestep numbers as needed.
4. Run the `main()` function to execute the desired functionality.

---

### Report

#### Questions

1. **Filtering EMG Data**: The code implements two accurate filtering techniques for EMG data: low-pass and high-pass filtering (`filter_data` function). It compares the filtered data visually through plotting, allowing for an assessment of their effectiveness in noise reduction.

2. **Conversion of Marker Coordinates to Angles**: Through the function `convert_segments_to_angles`, the code converts marker coordinates to angles based on the motion trajectory. It calculates the angles between consecutive markers and plots the angle values to visualize the arm movement.

3. **Iterative Prediction of Motion**: The code performs an iterative prediction of motion using inverse kinematics (`inverse_kinematics` function) with the real start and end positions of the arm. It calculates joint angles iteratively and plots the predicted arm movement trajectory, allowing for a comparison with the actual motion.

4. **Jerk-Optimized Prediction of Motion**: Utilizing the `optimal_jerk` function, the code applies jerk optimization to predict arm motion with the real start and end positions. It smooths the trajectory for smoother movement and plots the optimized predicted motion alongside the actual motion for comparison.

5. **Comparison between Real and Predicted Motions**: The code facilitates a comparison between the real and predicted motions by plotting them together. This comparison allows for an evaluation of the accuracy and effectiveness of the prediction methods in reproducing the actual arm movement.

#### Functions

1. **main()**: This function serves as the entry point of the program. It orchestrates the entire process by importing data, filtering it, and then calling other functions to reconstruct arm movement under different conditions.

2. **import_data(filename)**: This function imports data from a specified file, parsing it into time, marker positions, and EMG signals. It handles NaN values by removing corresponding rows from the data.

3. **filter_data(data, type)**: This function filters the input data using a Butterworth filter. It accepts either 'low' or 'high' as the filter type and applies the appropriate filter to the data.

4. **cut_sequence(time_ms, markers, start_time, end_time)**: This function cuts the sequence of marker data and time data based on specified start and end times. It returns the trimmed time and marker data.

5. **get_end_effector_start_end_position(markers)**: This function extracts the start and end positions of the arm movement from the marker data. It returns the start and end positions separately.

6. **set_offset_auto(markers)**: This function sets the offset automatically for the marker data. It calculates the offset based on the first marker position and adjusts all markers accordingly.

7. **calculate_segment_lengths(markers_one_position)**: This function calculates the segment lengths of the arm based on the marker positions at a single point in time. It returns a list of segment lengths.

8. **convert_segments_to_angles(markers)**: This function converts the marker coordinates into angles representing the arm movement. It calculates the angle between consecutive markers and returns a list of angles.

9. **average_markers(markers, num_intervals)**: This function averages the marker positions over specified intervals to reduce noise and improve accuracy. It returns averaged marker positions.

10. **inverse_kinematics(init_angles, target, seg_lens, ts_num)**: This function performs inverse kinematics to predict arm movement based on initial joint angles, target position, segment lengths, and the number of time steps. It returns the predicted trajectory of the arm.

11. **optimal_jerk(seg_lens, alphas, ts_num)**: This function applies jerk optimization to smooth the predicted arm movement. It calculates optimal jerk values based on segment lengths, joint angles, and the number of time steps.

12. **plot_raw_filtered_data(time_ms, markers, low_pass_filtered_markers, high_pass_filtered_markers)**: This function plots the raw and filtered marker data for visualization and comparison.

13. **plot_marker_data_3D(markers, time_ms, step)**: This function plots the marker data in 3D space over time, allowing for visualization of the arm movement trajectory.

14. **plot_movement_2D(markers, E, E_smooth_jerk, ts_num)**: This function plots the 2D arm movement trajectory based on the predicted and smoothed jerk-optimized motion. It also includes the real start and end positions for comparison.

#### Limitations

The main limitations of the code include its focus on 2D movement reconstruction, sensitivity to sensor placement, complexity of the human arm model and captured data.

#### Future Work

To address the limitations, future work could involve:

- Extending functionality for 3D movement reconstruction.
- Implementing robust techniques to handle sensor placement variability (segment length).
- Enhancing data processing algorithms for improved accuracy.

#### Conclusion

Overall, the provided code offers valuable insights into recreating human arm movement and implementing inverse kinematics. Despite its limitations, it serves as a foundation for my further research and development in biomechanics.
