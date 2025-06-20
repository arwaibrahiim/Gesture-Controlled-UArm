import sim
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import pyfirmata
import time

board = pyfirmata.Arduino('COM5')

it = pyfirmata.util.Iterator(board)
it.start()

servos = [board.get_pin('d:11:s'), 
          board.get_pin('d:10:s'), 
          board.get_pin('d:5:s'), 
          board.get_pin('d:6:s'), 
          board.get_pin('d:9:s')]
buzzer = board.get_pin('d:3:o')

servos[0].write(90)
servos[1].write(40)
servos[2].write(100)
servos[3].write(90)
servos[4].write(90)
time.sleep(0.5)
# Connect to CoppeliaSim
sim.simxFinish(-1)  # Close all opened connections
client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id != -1:
    print("Connected to CoppeliaSim")

    # Set up handles
    _, object_handle = sim.simxGetObjectHandle(client_id, 'Target', sim.simx_opmode_blocking)
    _, gripper_handle = sim.simxGetObjectHandle(client_id, 'uarmGripper', sim.simx_opmode_blocking)
    
    joint_handles = []
    for i in range(1, 4):  # Assuming a 6-DOF robot
        _, joint_handle = sim.simxGetObjectHandle(client_id, f'M_Joint{i}', sim.simx_opmode_blocking)
        joint_handles.append(joint_handle)

    # Initialize streaming
    _, initial_pos = sim.simxGetObjectPosition(client_id, object_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectPosition(client_id, gripper_handle, -1, sim.simx_opmode_streaming)
    for handle in joint_handles:
        sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_streaming)
    
    # Movement with position limits (x and z in cm)
    trajectory = []
    joint_angles = []
    timestamps = []
    start_time = time.time()
    for i in range(5):  # Example loop for 100 steps
        x = 15 #np.random.uniform(10, 25)  # x range: -10 cm to 10 cm
        y = 2*i #np.random.uniform(0, 10)
        z = 20 #np.random.uniform(10, 25)  # z range: 0 cm to 20 cm
        sim.simxSetObjectPosition(client_id, object_handle, -1, [x/100, y/100, z/100], sim.simx_opmode_oneshot)
        
        time.sleep(2)  # Allow time for motion
        # Get gripper position
        _, gripper_pos = sim.simxGetObjectPosition(client_id, gripper_handle, -1, sim.simx_opmode_buffer)
        trajectory.append(gripper_pos)
        # Get joint angles
        angles = []
        for handle in joint_handles:
            _, angle = sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_buffer)
            angles.append(math.degrees(angle))
        joint_angles.append(angles)
        timestamps.append(time.time() - start_time)
  
    # Disconnect
    sim.simxFinish(client_id)
    sim.simxFinish(client_id)
    print("Simulation stopped and connection closed.")
    # Convert data for plotting
    trajectory = np.array(trajectory)
    joint_angles = np.array(joint_angles)
    for i in range(joint_angles.shape[0]):
        servos[0].write(joint_angles[i,0])
        servos[1].write(joint_angles[i,2])
        servos[2].write(joint_angles[i,1])
        servos[3].write(joint_angles[i,0])
        time.sleep(2/joint_angles.shape[0])
    # Plot trajectory
    fig = plt.figure()
    fig2 = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='Gripper Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("Gripper Trajectory")
    ax.legend()
    aY = fig2.add_subplot(111)
    for i in range(joint_angles.shape[1]):
        aY.plot(timestamps, joint_angles[:, i], label=f'Joint {i+1}')
    aY.set_xlabel('Time (s)')
    aY.set_ylabel('Joint Angle (rad)')
    aY.set_title('Joint Angles Over Time')
    aY.legend()
    aY.grid()
    plt.show()
else:
    print("Failed to connect to CoppeliaSim")
    

