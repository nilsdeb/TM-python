import numpy as np
from scipy.spatial.transform import Rotation as R

# Define the desired roll, pitch, and yaw angles (in degrees)
desired_roll = 30.0
desired_pitch = 45.0
desired_yaw = 0.0  # Yaw angle set to 0

# Convert degrees to radians
roll_rad = np.radians(desired_roll)
pitch_rad = np.radians(desired_pitch)
yaw_rad = np.radians(desired_yaw)

# Construct rotation matrix R1 with the desired roll and pitch (and 0 yaw)
R1 = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad], degrees=False).as_matrix()

# Construct rotation matrix R2 with the same roll and pitch (and 0 yaw)
R2 = R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad], degrees=False).as_matrix()

# Convert R1 and R2 to Rotation objects to verify
rotation_r1 = R.from_matrix(R1)
rotation_r2 = R.from_matrix(R2)

# Extract the Euler angles (roll, pitch, yaw) for R1 and R2
euler_angles_r1 = rotation_r1.as_euler('xyz', degrees=True)
euler_angles_r2 = rotation_r2.as_euler('xyz', degrees=True)

print("Desired Roll (degrees):", desired_roll)
print("Desired Pitch (degrees):", desired_pitch)
print("Desired Yaw (degrees):", desired_yaw)
print("\nRotation Matrix R1:")
print(R1)
print("\nRotation Matrix R2:")
print(R2)
print("\nEuler Angles for R1 (degrees):", euler_angles_r1)
print("Euler Angles for R2 (degrees):", euler_angles_r2)