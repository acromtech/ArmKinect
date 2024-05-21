"""
Title: Jerk Decomposition Analysis

Description:
This program performs jerk decomposition analysis based on the methodology presented in the Neuromorph Movement Control course
It loads data containing joint angles, filters and calculates derivatives, computes Jacobian matrices, defines jerk components, and plots the results. 
The goal is to analyze the acceleration components contributing to jerk in a motion capture scenario.

Author: Alexis GIBERT

Date: 22/05/2024

Requirements:
- Python 3.x
- NumPy
- Matplotlib
- SciPy

Usage:
Run the program and ensure the data file 'data4.txt' is present in the same directory. Modify the file path if necessary.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def main():
    time_ms, markers, _ = import_data('data4.txt')
    thetas = convert_markers_to_angles(markers)
    seg_len = [300,300,300]
    thetas_d, thetas_dd, thetas_ddd = savitzky_golay_filter(thetas, time_ms)
    J, DJdt, D2Jdt2 = calculate_jacobians(seg_len, thetas, thetas_d, thetas_dd)
    all_norms = calculate_jerk_components(thetas, thetas_d, thetas_dd, thetas_ddd, J, DJdt, D2Jdt2)

def import_data(filename):
    """
    Import data from a text file.
    
    Parameters:
        filename (str): The path to the text file containing the data.
    
    Returns:
        tuple: A tuple containing time in milliseconds, marker positions, and muscle voltages.
    """
    header_lines = 13
    data = np.loadtxt(filename, skiprows=header_lines)

    # Delete NaN lines
    mask = ~np.any(np.isnan(data), axis=1)
    data = data[mask]

    time_ms = data[:, 0]
    marker_1 = data[:, 1:4]
    marker_2 = data[:, 4:7]
    marker_3 = data[:, 7:10]
    marker_4 = data[:, 10:13]
    biceps_mv = data[:, 13]
    triceps_mv = data[:, 14]
    flexor_digitorum_mv = data[:, 15]

    return time_ms, [marker_1, marker_2, marker_3, marker_4], [biceps_mv, triceps_mv, flexor_digitorum_mv]

def convert_markers_to_angles(markers, plot=True):
    """
    Convert marker positions to joint angles.
    
    Parameters:
        markers (list): List of marker positions.
        plot (bool): Whether to plot the angles or not.
    
    Returns:
        numpy.ndarray: Array of joint angles.
    """
    thetas = []
    v0 = np.zeros(3)    # Define the fictitious marker 0

    for i in range(len(markers[0])):
        marker_angles = []
        for j in range(len(markers) - 1):
            # Calculate vectors between consecutive markers
            v1 = markers[j][i] - v0 if j == 0 else markers[j][i] - markers[j - 1][i]
            # Calculate the vector for the next marker
            v2 = markers[j + 1][i] - markers[j][i]
            # Calculate the cosine of the angle between the vectors
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            # Calculate the angle and append to the list
            angle = np.arccos(cos_angle)
            marker_angles.append(angle)
        thetas.append(marker_angles)

    if plot==True:
        # Plot the angles
        plt.figure()
        plt.plot(thetas)
        plt.title('Angles between segments')
        plt.xlabel('Time (ms)')
        plt.ylabel('Angle (randian)')
        plt.legend()
        plt.show()

    return np.array(thetas).T

def savitzky_golay_filter(thetas, time_ms):
    """
    Apply Savitzky-Golay filter to smooth angle data and calculate derivatives.
    
    Parameters:
        thetas (numpy.ndarray): Array of joint angles.
        time_ms (numpy.ndarray): Array of time in milliseconds.
    
    Returns:
        tuple: Tuple containing first, second, and third derivatives of angles.
    """
    # Extract columns
    theta1 = thetas[0, :]
    theta2 = thetas[1, :]
    theta3 = thetas[2, :]

    # Calculate derivatives and filter with Savitzky-Golay filter
    theta1dot = savgol_filter(x=np.gradient(theta1) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta2dot = savgol_filter(x=np.gradient(theta2) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta3dot = savgol_filter(x=np.gradient(theta3) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    
    theta1dotdot = savgol_filter(x=np.gradient(theta1dot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta2dotdot = savgol_filter(x=np.gradient(theta2dot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta3dotdot = savgol_filter(x=np.gradient(theta3dot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)

    theta1dotdotdot = savgol_filter(x=np.gradient(theta1dotdot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta2dotdotdot = savgol_filter(x=np.gradient(theta2dotdot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    theta3dotdotdot = savgol_filter(x=np.gradient(theta3dotdot) / (time_ms[1] - time_ms[0]), window_length=8, polyorder=7)
    
    thetas = [theta1, theta2, theta3]
    thetas_d = [theta1dot, theta2dot, theta3dot]
    thetas_dd = [theta1dotdot, theta2dotdot, theta3dotdot]
    thetas_ddd = [theta1dotdotdot, theta2dotdotdot, theta3dotdotdot]

    # Print Thetas
    print("\n\nAngular configuration ...")
    print("\nTheta\n", np.array(thetas))
    print("\nTheta dot\n", np.array(thetas_d))
    print("\nTheta dot dot\n", np.array(thetas_dd))
    print("\nTheta dot dot dot\n", np.array(thetas_ddd))

    return thetas_d, thetas_dd, thetas_ddd

def calculate_jacobians(seg_len, thetas, thetas_d, thetas_dd):
    """
    Calculate Jacobian matrices and their derivatives.
    
    Parameters:
        seg_len (list): List of segment lengths.
        thetas (numpy.ndarray): Array of joint angles.
        thetas_d (numpy.ndarray): Array of first derivatives of joint angles.
        thetas_dd (numpy.ndarray): Array of second derivatives of joint angles.
    
    Returns:
        tuple: Tuple containing Jacobians, first derivatives, and second derivatives.
    """
    theta1, theta2, theta3 = thetas
    theta1dot, theta2dot, theta3dot = thetas_d
    theta1dotdot, theta2dotdot, theta3dotdot = thetas_dd

    # Initialize matrices
    num_samples = len(theta1)
    J = np.zeros((2, 3, num_samples))
    DJdt = np.zeros((2, 3, num_samples))
    D2Jdt2 = np.zeros((2, 3, num_samples))
    
    # Jacobian matrix + derivatives
    for i in range(num_samples):
        J[:, :, i] = np.array([
            [-seg_len[0] * np.sin(theta1[i]) - seg_len[1] * np.sin(theta1[i] + theta2[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]),
             -seg_len[1] * np.sin(theta1[i] + theta2[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]),
             -seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i])],
            [seg_len[0] * np.cos(theta1[i]) + seg_len[1] * np.cos(theta1[i] + theta2[i]) + seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]),
             seg_len[1] * np.cos(theta1[i] + theta2[i]) + seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]),
             seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i])]
        ])
        
        DJdt[:, :, i] = np.array([
            [-seg_len[0] * np.cos(theta1[i]) * theta1dot[i] - seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i]) - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i]),
             -seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i]) - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i]),
             -seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])],
            [-seg_len[0] * np.sin(theta1[i]) * theta1dot[i] - seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i]),
             -seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i]),
             -seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])]
        ])
        
        D2Jdt2[:, :, i] = np.array([
            [-seg_len[0] * np.cos(theta1[i]) * theta1dotdot[i] - seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dotdot[i] + theta2dotdot[i]) - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) + seg_len[0] * np.sin(theta1[i]) * theta1dot[i]**2 + seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i])**2 + seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2,
             -seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dotdot[i] + theta2dotdot[i]) - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) + seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i])**2 + seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2,
             -seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) + seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2],
            [-seg_len[0] * np.sin(theta1[i]) * theta1dotdot[i] - seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dotdot[i] + theta2dotdot[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) - seg_len[0] * np.cos(theta1[i]) * theta1dot[i]**2 - seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i])**2 - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2,
             -seg_len[1] * np.sin(theta1[i] + theta2[i]) * (theta1dotdot[i] + theta2dotdot[i]) - seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) - seg_len[1] * np.cos(theta1[i] + theta2[i]) * (theta1dot[i] + theta2dot[i])**2 - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2,
             -seg_len[2] * np.sin(theta1[i] + theta2[i] + theta3[i]) * (theta1dotdot[i] + theta2dotdot[i] + theta3dotdot[i]) - seg_len[2] * np.cos(theta1[i] + theta2[i] + theta3[i]) * (theta1dot[i] + theta2dot[i] + theta3dot[i])**2]
        ])
    
    # Print Jacobians
    print("\n\nJacobians + derivatives ...")
    print("\nJacobian\n", J)
    print("\nJacobian first derivative\n", DJdt)
    print("\nJacobian second derivative\n", D2Jdt2)

    return J, DJdt, D2Jdt2

def calculate_jerk_components(thetas, thetas_d, thetas_dd, thetas_ddd, J, DJdt, D2Jdt2, plot=True):
    """
    Calculate jerk acceleration components and their norms.
    
    Parameters:
        thetas (numpy.ndarray): Array of joint angles.
        thetas_d (numpy.ndarray): Array of first derivatives of joint angles.
        thetas_dd (numpy.ndarray): Array of second derivatives of joint angles.
        thetas_ddd (numpy.ndarray): Array of third derivatives of joint angles.
        J (numpy.ndarray): Jacobian matrices.
        DJdt (numpy.ndarray): First derivatives of Jacobians.
        D2Jdt2 (numpy.ndarray): Second derivatives of Jacobians.
        plot (bool): Whether to plot the jerk components or not.
    
    Returns:
        list: List containing the norms of jerk components.
    """ 
    theta1, _, _ = thetas
    theta1dot, theta2dot, theta3dot = thetas_d
    theta1dotdot, theta2dotdot, theta3dotdot = thetas_dd
    theta1dotdotdot, theta2dotdotdot, theta3dotdotdot = thetas_ddd
    num_samples = len(theta1)

    # Jerk acceleration components
    G3 = np.zeros((2, num_samples))
    G2 = np.zeros((2, num_samples))
    G1 = np.zeros((2, num_samples))
    GT = np.zeros((2, num_samples))
    GTsq = np.zeros(num_samples)
    G1sq = np.zeros(num_samples)
    G2sq = np.zeros(num_samples)
    G3sq = np.zeros(num_samples)
    G4sq = np.zeros(num_samples)
    
    for i in range(num_samples):
        G3[:, i] = J[:, :, i] @ np.array([theta1dotdotdot[i], theta2dotdotdot[i], theta3dotdotdot[i]])
        G2[:, i] = 2 * DJdt[:, :, i] @ np.array([theta1dotdot[i], theta2dotdot[i], theta3dotdot[i]])
        G1[:, i] = D2Jdt2[:, :, i] @ np.array([theta1dot[i], theta2dot[i], theta3dot[i]])
        GT[:, i] = G1[:, i] + G2[:, i] + G3[:, i]
        
        GTsq[i] = GT[:, i].T @ GT[:, i]
        G1sq[i] = G1[:, i].T @ G1[:, i]
        G2sq[i] = G2[:, i].T @ G2[:, i]
        G3sq[i] = G3[:, i].T @ G3[:, i]
        G4sq[i] = 2 * (G1[:, i].T @ G2[:, i] + G1[:, i].T @ G3[:, i] + G2[:, i].T @ G3[:, i])
    
    # Compute the percentage of each jerk component
    IntG1 = 100 * np.sum(G1sq[100:-100]) / np.sum(GTsq[100:-100])
    IntG2 = 100 * np.sum(G2sq[100:-100]) / np.sum(GTsq[100:-100])
    IntG3 = 100 * np.sum(G3sq[100:-100]) / np.sum(GTsq[100:-100])
    IntG4 = 100 * np.sum(G4sq[100:-100]) / np.sum(GTsq[100:-100])
    AllNorms = [IntG1, IntG2, IntG3, IntG4]
    
    # Print Jacobians
    print("\n\nPercentage of each jerk component...")
    print("G1 percentage:\t", IntG1)
    print("G2 percentage:\t", IntG2)
    print("G3 percentage:\t", IntG3)
    print("G4 percentage:\t", IntG4)

    # Plot
    if plot == True:
        plt.figure()
        plt.plot(G1[0, :], 'b', label='G1-x')
        plt.plot(G1[1, :], 'b--', label='G1-y')
        plt.plot(G2[0, :], 'r', label='G2-x')
        plt.plot(G2[1, :], 'r--', label='G2-y')
        plt.plot(G3[0, :], 'k', label='G3-x')
        plt.plot(G3[1, :], 'k--', label='G3-y')
        plt.xlabel('Time Step')
        plt.ylabel('Acceleration')
        plt.title('Jerk Components')
        plt.legend()
        plt.show()

    return AllNorms

main()
