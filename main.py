import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def main():
    time_ms, markers, emg_data = import_data('data4.txt')
    
    # ARM EXTENDED (use the end position of the markers (markers_end_2D) to calculate the segment lengths and reach the target position - because it's imposible with the start position of the markers (markers_start_2D))
    recreate_mouvement(time_ms, markers, emg_data, start_time=28500, end_time=31000, timesteps_num=100)

    # ARM AGAINST THE BODY (use the markers_start_2D to calculate the segment lengths)
    recreate_mouvement(time_ms, markers, emg_data, start_time=31000, end_time=34000, timesteps_num=100)

    # Given the variability in segment values due to imprecise sensor placement, 
    # it becomes evident that while the ideal arm spans from the starting point to the end effector (our hand / marker 4), 
    # it is expected that other markers (1,2,3) are not reached.

    # Note that this line "calculate_segment_lengths(np.maximum(markers_start_2D, markers_end_2D))" in "recreate_mouvement" function
    # So when you watch the plot, it's appear that the segment for the start point doesn't match it's normal because without that we couln't reach the target
    # An other possibility is to use 3D markers to mesure segment lengths but doesn't match also with the inverse_kinematic calculation (only in 2D)
    # The current program have sevrals limitation due to the model and the data. It's very complicated to match both.
    # But when I have time i would try to make it.

# PROCESSING FUNCTION ---------------------------------------------------------------------------------------------------------------------------  

def recreate_mouvement(time_ms, markers, emg_data, start_time, end_time, timesteps_num, segment_lengths=None):
    # Cut markers data and plot it to see which filtering technique is the best one
    _, markers = cut_sequence(time_ms, markers, start_time, end_time)
    time_ms, emg_data = cut_sequence(time_ms, emg_data, start_time, end_time)

    # Filter markers and EMG data - for accurate data
    low_pass_filtered_emg_data = filter_data(emg_data, 'low')
    high_pass_filtered_emg_data = filter_data(emg_data, 'high')
    low_pass_filtered_markers = filter_data(markers, 'low')
    high_pass_filtered_markers = filter_data(markers, 'high')

    # Plot markers and EMG data
    plot_raw_filtered_data(time_ms, markers, low_pass_filtered_markers, high_pass_filtered_markers)
    plot_raw_filtered_data(time_ms, emg_data, low_pass_filtered_emg_data, high_pass_filtered_emg_data)

    # Choose your filtered data and plot it in 3D and verify your movement sequence
    markers = low_pass_filtered_markers
    plot_marker_data_3D(markers, time_ms, timesteps_num)

    # Format data for inverse kinematic calculation
    markers_2D = [marker[:, :2] for marker in markers]
    markers_2D = set_offset_auto(markers_2D)
    markers_start_2D, markers_end_2D = get_end_effector_start_end_position(markers_2D) # 1 -> x ; 2 -> (x,y) ; 3 -> (x,y,z)

    # 2D Calculations
    if segment_lengths==None : segment_lengths = calculate_segment_lengths(np.maximum(markers_start_2D, markers_end_2D)) # IMPORTANT
    angles = convert_segments_to_angles(markers_start_2D)
    # E, final_a, alphas = inverse_kinematics(angles, markers_end_2D[-1], segment_lengths, timesteps_num)
    E, _, alphas = inverse_kinematics(angles, markers_end_2D[-1], segment_lengths, timesteps_num)
    E_smooth_jerk = optimal_jerk(segment_lengths, alphas, timesteps_num)
    plot_movement_2D(markers_2D, E, E_smooth_jerk, timesteps_num)

# TOOLS ---------------------------------------------------------------------------------------------------------------------------  

def import_data(filename):
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

def filter_data(data, type):
    fs = 450
    fc = 5
    b, a = butter(4, fc/(fs/2), type)
    filtered_data = []
    for i in range(len(data)):
        filtered_data.append(filtfilt(b, a, data[i], axis=0))
    return filtered_data

def cut_sequence(time_ms, markers, start_time, end_time):
    markers_cutted = []
    time_interval_indices = (time_ms >= start_time) & (time_ms <= end_time)
    time_ms = time_ms[time_interval_indices]
    for i in range(len(markers)): markers_cutted.append(markers[i][time_interval_indices])
    print("Size of the sequence: ", len(markers_cutted[0]))
    return time_ms, markers_cutted

def get_end_effector_start_end_position(markers):
    markers_start = [marker[0, :] for marker in markers]
    markers_end = [marker[-1, :] for marker in markers]
    return markers_start, markers_end

def set_offset_auto(markers):
    markers_offset = [markers[0] - marker for marker in markers]
    return markers_offset

def calculate_segment_lengths(markers_one_position):
    seg_len = []
    for i in range (len(markers_one_position)-1) : seg_len.append(np.linalg.norm(markers_one_position[i+1] - markers_one_position[i]))
    print("Segment Lengths: ", seg_len)
    return seg_len

def calculate_segment_lengths2(markers_2D):
    num_times = len(markers_2D[0])
    num_segments = len(markers_2D) - 1
    seg_lens_list = np.zeros((num_segments, num_times))
    for t in range(num_times):
        positions_at_t = [marker[t] for marker in markers_2D]
        for i in range(num_segments):
            seg_lens_list[i, t] = np.linalg.norm(positions_at_t[i + 1] - positions_at_t[i])
    return seg_lens_list

def convert_segments_to_angles(markers):
    marker_0 = np.copy(markers[0])
    marker_0[0] -= 100
    v = []
    for i in range (len(markers)):
        if i == 0 : v.append((markers[0] - marker_0))
        else : v.append((markers[i] - markers[i-1]))
    angles = []
    for i in range (len(markers)-1):
        cos_angle = np.dot(v[i], v[i+1]) / (np.linalg.norm(v[i]) * np.linalg.norm(v[i+1]))
        angles.append(np.arccos(cos_angle))
    print("Angles: ", angles)
    return angles

def average_data(data, num_intervals):
    num_values = len(data[0])  # Nombre total de valeurs pour chaque ensemble de données
    interval_size = num_values // num_intervals  # Taille de chaque intervalle

    averaged_data = []
    for dataset in data:
        averaged_values = []
        for i in range(num_intervals):
            start_index = i * interval_size
            end_index = min((i + 1) * interval_size, num_values)
            interval_mean = np.mean(dataset[start_index:end_index], axis=0)
            averaged_values.append(interval_mean)
        averaged_data.append(np.array(averaged_values))
    return averaged_data

# MODEL ---------------------------------------------------------------------------------------------------------------------------  

def inverse_kinematics(init_angles, target, seg_lens, ts_num):
    assert np.sqrt(target[0]**2 + target[1]**2) <= np.sum(seg_lens), "Target point out of reach"
    alphas = np.zeros((3, ts_num)) 
    alphas[:, 0] = init_angles
    a1 = alphas[0, 0]
    a12 = a1 + alphas[1, 0]
    a123 = a12 + alphas[2, 0]
    E1 = np.zeros((2, ts_num))
    E2 = np.zeros((2, ts_num))
    E3 = np.zeros((2, ts_num))
    E1[:, 0] = [seg_lens[0] * np.cos(a1), seg_lens[0] * np.sin(a1)]
    E2[:, 0] = [E1[0, 0] + seg_lens[1] * np.cos(a12), E1[1, 0] + seg_lens[1] * np.sin(a12)]
    E3[:, 0] = [E2[0, 0] + seg_lens[2] * np.cos(a123), E2[1, 0] + seg_lens[2] * np.sin(a123)]
    target_difference = [(target[0] - E3[0, 0]) / ts_num, (target[1] - E3[1, 0]) / ts_num]
    for i in range(ts_num - 1):
        a1 = alphas[0, i]
        a12 = a1 + alphas[1, i]
        a123 = a12 + alphas[2, i]
        J = np.array([[-seg_lens[0] * np.sin(a1) - seg_lens[1] * np.sin(a12) - seg_lens[2] * np.sin(a123),
                        -seg_lens[1] * np.sin(a12) - seg_lens[2] * np.sin(a123),
                        -seg_lens[2] * np.sin(a123)],
                    [seg_lens[0] * np.cos(a1) + seg_lens[1] * np.cos(a12) + seg_lens[2] * np.cos(a123),
                        seg_lens[1] * np.cos(a12) + seg_lens[2] * np.cos(a123),
                        seg_lens[2] * np.cos(a123)]])
        delta_a = np.linalg.pinv(J) @ target_difference
        alphas[:, i + 1] = alphas[:, i] + delta_a
    final_a = alphas[:, ts_num - 1]
    va1 = alphas[0, :]
    va12 = va1 + alphas[1, :]
    va123 = va12 + alphas[2, :]
    E1 = np.array([seg_lens[0] * np.cos(va1), seg_lens[0] * np.sin(va1)])
    E2 = np.array([E1[0, :] + seg_lens[1] * np.cos(va12), E1[1, :] + seg_lens[1] * np.sin(va12)])
    E3 = np.array([E2[0, :] + seg_lens[2] * np.cos(va123), E2[1, :] + seg_lens[2] * np.sin(va123)])
    return [E1, E2, E3], final_a, alphas

def optimal_jerk(seg_lens, alphas, ts_num):
    tau = np.linspace(0, 1, ts_num)
    beta = np.zeros((3, ts_num))
    beta0 = alphas[:, 0]
    betaT = alphas[:,-1]
    beta0 = beta0.reshape((3, 1))
    betaT = betaT.reshape((3, 1))
    beta = beta0 + (beta0 - betaT) * (15 * tau**4 - 6 * tau**5 - 10 * tau**3)
    b1 = beta[0, :]
    b12 = b1 + beta[1, :]
    b123 = b12 + beta[2, :]
    E1_smooth_jerk = np.array([seg_lens[0] * np.cos(b1), seg_lens[0] * np.sin(b1)])
    E2_smooth_jerk = np.array([E1_smooth_jerk[0, :] + seg_lens[1] * np.cos(b12), E1_smooth_jerk[1, :] + seg_lens[1] * np.sin(b12)])
    E3_smooth_jerk = np.array([E2_smooth_jerk[0, :] + seg_lens[2] * np.cos(b123), E2_smooth_jerk[1, :] + seg_lens[2] * np.sin(b123)])
    return [E1_smooth_jerk, E2_smooth_jerk, E3_smooth_jerk]

# PLOT ---------------------------------------------------------------------------------------------------------------------------  

def plot_raw_filtered_data(time_ms, markers, low_pass_filtered_markers, high_pass_filtered_markers):
    fig, axs = plt.subplots(3, len(markers), figsize=(18, 12))
    for i in range(len(markers)):
        axs[0, i].plot(time_ms, markers[i])
        axs[0, i].set_title(f'Raw - Marker {i+1}')
        axs[1, i].plot(time_ms, low_pass_filtered_markers[i])
        axs[1, i].set_title(f'Low pass - Marker {i+1}')
        axs[2, i].plot(time_ms, high_pass_filtered_markers[i])
        axs[2, i].set_title(f'High pass - Marker {i+1}')
    plt.tight_layout()
    plt.show()

def plot_marker_data_3D(markers, time_ms, step):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    num_images = markers[0].shape[0]

    min_x = np.min([marker[:, 0] for marker in markers])
    max_x = np.max([marker[:, 0] for marker in markers])
    min_y = np.min([marker[:, 1] for marker in markers])
    max_y = np.max([marker[:, 1] for marker in markers])
    min_z = np.min([marker[:, 2] for marker in markers])
    max_z = np.max([marker[:, 2] for marker in markers])

    for i in range(0, num_images, step):
        ax.clear()

        indices = range(i, min(i + step, num_images))

        mean_markers = [np.mean(marker[indices], axis=0) for marker in markers]

        for j, mean_marker in enumerate(mean_markers):
            marker_color = ['r', 'g', 'b', 'm'][j]
            ax.plot(mean_marker[0], mean_marker[1], mean_marker[2], marker_color + 'o')

            if j < len(mean_markers) - 1:
                next_mean_marker = mean_markers[j + 1]
                ax.plot([mean_marker[0], next_mean_marker[0]], [mean_marker[1], next_mean_marker[1]],
                        [mean_marker[2], next_mean_marker[2]], marker_color, linewidth=2)

        ax.set_xlabel('X Position (mm)')
        ax.set_ylabel('Y Position (mm)')
        ax.set_zlabel('Z Position (mm)')
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_zlim(min_z, max_z)
        ax.set_title('Arm t={} ms à t={} ms (Average of {} values)'
                     .format(time_ms[indices[0]], time_ms[indices[-1]], step))

        plt.pause(0.1)

    plt.close()

def plot_movement_2D(markers, E, E_smooth_jerk, ts_num):
    markers_start, markers_end = get_end_effector_start_end_position(markers)
    averaged_markers = average_data(markers, ts_num)
    plt.figure()

    # Plot the data start/end position
    for i in range (len(markers_start)):
        if i==len(markers_start)-1:
            plt.plot(markers_start[i][0], markers_start[i][1], 'mo', markersize=5, label='Start Point')
            plt.plot(markers_end[i][0], markers_end[i][1], 'co', markersize=5, label='Target Point')
        else :
            plt.plot(markers_start[i][0], markers_start[i][1], 'mo', markersize=3)
            plt.plot(markers_end[i][0], markers_end[i][1], 'co', markersize=3)
    marker_0, = plt.plot([markers_start[0][0], -100],[markers_start[0][1], markers_start[0][1]], 'k--')
    
    # For the given number of steps
    for i in range(ts_num):
        
        # Plot the real arm extract from data (green)
        arm_data, = plt.plot([averaged_markers[0][i, 0], averaged_markers[1][i, 0], averaged_markers[2][i, 0], averaged_markers[3][i, 0]],
                             [averaged_markers[0][i, 1], averaged_markers[1][i, 1], averaged_markers[2][i, 1], averaged_markers[3][i, 1]], 'g-', label='Real arm')
        arm_data_joint_1, = plt.plot(averaged_markers[1][i,0], averaged_markers[1][i,1], 'go', markersize=5)
        arm_data_joint_2, = plt.plot(averaged_markers[2][i,0], averaged_markers[2][i,1], 'go', markersize=5)
        arm_data_joint_3, = plt.plot(averaged_markers[3][i,0], averaged_markers[3][i,1], 'go', markersize=5)

        # Plot the inverse kinematic arm and adjust for the offset (red)
        arm_iter, = plt.plot([0, E[0][0, i], E[1][0, i], E[2][0, i], ], [0, E[0][1, i], E[1][1, i], E[2][1, i]], 'r-', label='IK arm')
        arm_iter_joint_1, = plt.plot(E[0][0,i], E[0][1,i], 'ro', markersize=5)
        arm_iter_joint_2, = plt.plot(E[1][0,i], E[1][1,i], 'ro', markersize=5)
        arm_iter_joint_3, = plt.plot(E[2][0,i], E[2][1,i], 'ro', markersize=5)

        # Plot the optimal jerk arm and adjust for the offset (blue)
        arm_jerk, = plt.plot([0, E_smooth_jerk[0][0, i], E_smooth_jerk[1][0, i], E_smooth_jerk[2][0, i]], 
                             [0, E_smooth_jerk[0][1, i], E_smooth_jerk[1][1, i], E_smooth_jerk[2][1, i]], 'b-', label='Optimal arm')
        arm_jerk_joint_1, = plt.plot(E_smooth_jerk[0][0,i], E_smooth_jerk[0][1,i], 'bo', markersize=5)
        arm_jerk_joint_2, = plt.plot(E_smooth_jerk[1][0,i], E_smooth_jerk[1][1,i], 'bo', markersize=5)
        arm_jerk_joint_3, = plt.plot(E_smooth_jerk[2][0,i], E_smooth_jerk[2][1,i], 'bo', markersize=5)

        # Calculate the plot limits based on the adjusted positions
        xlim = [min(np.min(E[0][0, :]), np.min(E[1][0, :]), np.min(E[2][0, :]), markers_end[-1][0], markers_start[-1][0], 0), 
                max(np.max(E[0][0, :]), np.max(E[1][0, :]), np.max(E[2][0, :]), markers_end[-1][0], markers_start[-1][0], 0)]
        ylim = [min(np.min(E[0][1, :]), np.min(E[1][1, :]), np.min(E[2][1, :]), markers_end[-1][1], markers_start[-1][1], 0), 
                max(np.max(E[0][1, :]), np.max(E[1][1, :]), np.max(E[2][1, :]), markers_end[-1][1], markers_start[-1][1], 0)]

        # Set the plot limits
        plt.xlim([xlim[0] - 100, xlim[1] + 100])
        plt.ylim([ylim[0] - 100, ylim[1] + 100])

        plt.legend()

        # Draw the plot
        plt.draw()

        # Pause for each step depending on the number of steps itself
        plt.pause(0.001 + 100 / (20 * ts_num))

        # Remove what has been plotted (the arm and the target position)
        arm_data.remove()
        arm_data_joint_1.remove()
        arm_data_joint_2.remove()
        arm_data_joint_3.remove()
        arm_iter.remove()
        arm_iter_joint_1.remove()
        arm_iter_joint_2.remove()
        arm_iter_joint_3.remove()
        arm_jerk.remove()
        arm_jerk_joint_1.remove()
        arm_jerk_joint_2.remove()
        arm_jerk_joint_3.remove()

    plt.close()

main()