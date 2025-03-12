import numpy as np

np.set_printoptions(suppress=True)

def euler_to_rotation_matrix(roll, pitch, yaw):
    # Convert degrees to radians
    roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
    
    # Rotation matrices around x, y, and z axes
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))
    
    # Format the matrix in the desired format
    return f"[{', '.join(map(str, np.round(R.flatten(),8)))}]"

# Example usage:
roll = 180  # degrees
pitch = 0  # degrees
yaw = 180  # degrees

rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
print(rotation_matrix)
