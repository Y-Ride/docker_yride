import numpy as np
from scipy.spatial.transform import Rotation as R

# Create rotation matrices for x y and z axes
def rot_x(angle, degrees=True):
    if degrees:
        angle = np.radians(angle)
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rot_y(angle, degrees=True):
    if degrees:
        angle = np.radians(angle)
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rot_z(angle, degrees=True):
    if degrees:
        angle = np.radians(angle)
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

# Create homogeneous transformation matrix
def homogeneous_transform(rot_matrix, translation_vector):
    tf = np.block([[rot_matrix, translation_vector],
                   [0, 0, 0, 1]])
    return tf

def inches_to_meters(inches):
    return inches * 0.0254

# Create a transformation matrix for each lidar 
if __name__ == "__main__":
    # Define translations (x, y, z) and rotations (x, y, z) for each lidar
    # The rotations need to be in the order x, y, and then z all from the global frame
    is_degrees = True
    lidar_parameters = {
    "os_lidar_top":	[0.0, 	    0.0, 		0.0, 		0.0, 		0.0,		0.0],
    "os_lidar_left":   [-3.5,     	-28.75,	    -9.5, 	    48.9, 	    0.0, 		135.0], 
    "os_lidar_right":  [-3.5,     	28.75, 	    -9.5,	    -47.6, 	    0.0, 		-135.0],
    "os_lidar_front":  [-69.0,	    0.0, 		-8.0, 	    0.0, 		-37.6, 	    45.0],
    "os_lidar_back":   [69.0, 	    0.0, 		-8.0, 	    0.0, 		29.5, 	    135.],
    }

    # Transformation dictionary
    transformations = {}

    flatten = True


    for lidar, params in lidar_parameters.items():
        print(f"{lidar} parameters: {params}")
        rot = rot_x(params[3], is_degrees) @ rot_y(params[4], is_degrees) @ rot_z(params[5], is_degrees)
        trans = inches_to_meters(np.array([params[:3]]).T)
        print(f"Translation: \n{trans}")
        hom = homogeneous_transform(rot, trans)
        if flatten:
            hom = hom.reshape(-1)
            hom_str = ", ".join(f"{val:.1f}" if val == int(val) else f"{val:.8g}" for val in hom)
            print(f"{lidar} Transformation Matrix:\n[{hom_str}]\n")
            transformations[lidar] = hom_str
        else:
            print(f"{lidar} Transformation Matrix:\n{hom}\n")
            transformations[lidar] = hom

    print("\n-----------------------------------\n")
    print("Transformation dictionary:")

    for key, value in transformations.items():
        if flatten:
            print(f"{key}: \n\t[{value}]\n")
        else:
            print(f"{key}: \n{value}\n")

    