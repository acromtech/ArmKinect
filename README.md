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

#### Overview

This report summarizes the functionality and limitations of the provided code for recreating human arm movement and implementing inverse kinematics.

#### Functionality

The code effectively imports motion capture and EMG data, applies filters for data accuracy, and reconstructs arm movement trajectories in 2D space. It calculates segment lengths, computes joint angles using inverse kinematics, and applies optimal jerk smoothing for smoother movement. Various plotting functions visualize the data and computed results, aiding in analysis and interpretation.

#### Limitations

The main limitations of the code include its focus on 2D movement reconstruction, sensitivity to sensor placement, complexity of the human arm model and captured data.

#### Future Work

To address the limitations, future work could involve:

- Extending functionality for 3D movement reconstruction.
- Implementing robust techniques to handle sensor placement variability (segment length).
- Enhancing data processing algorithms for improved accuracy.

#### Conclusion

Overall, the provided code offers valuable insights into recreating human arm movement and implementing inverse kinematics. Despite its limitations, it serves as a foundation for my further research and development in biomechanics.