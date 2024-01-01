import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

class Trajectory:
    def __init__(self):
        self._tum_pose = []
        plt.ion()  # Turn on interactive mode
        self._fig, self._ax = plt.subplots()
        self._line, = self._ax.plot([], [], marker='o', color='b', label='Robot Trajectory')
        self._ax.set_xlabel('X-axis')
        self._ax.set_ylabel('Y-axis')
        self._ax.legend()

    def update(self, transformation, time_stamp):
        translation = transformation[:3, 3]  # Extract translation directly
        quaternion = self._rotation_matrix_to_quaternion(transformation[:3, :3])
        formatted_translation = [f"{val:.5f}" for val in translation]
        formatted_quaternion = [f"{val:.5f}" for val in quaternion]
        pose = [time_stamp, *formatted_translation, *formatted_quaternion]  # Unpack values
        self._tum_pose.append(pose)

        self._line.set_xdata(np.append(self._line.get_xdata(), translation[0]))
        self._line.set_ydata(np.append(self._line.get_ydata(), translation[1]))
        self._ax.relim()
        self._ax.autoscale_view()
        plt.draw()  # Refresh the plot

        # plt.pause(0.1)

    def save_as_tum(self, dir, id):
        os.makedirs(dir, exist_ok=True)
        file_path = os.path.join(dir, f"{id}.tum")  # Use f-string for clarity
        with open(file_path, 'w') as file:
            file.writelines(" ".join(map(str, pose)) + "\n" for pose in self._tum_pose)

    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        assert rotation_matrix.shape == (3, 3), "Input matrix must be a 3x3 rotation matrix"
        return Rotation.from_matrix(rotation_matrix.copy()).as_quat()

# import os
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation

# class Trajectory:
#     def __init__(self):
#         self.tum_pose = []
#         plt.ion()  # Turn on interactive mode
#         fig, self.ax = plt.subplots()
#         self.line, = self.ax.plot([], [], marker='o', color='b', label='Robot Trajectory')
#         self.ax.set_xlabel('X-axis')
#         self.ax.set_ylabel('Y-axis')
#         self.ax.legend()

#     def rotation_matrix_to_quaternion(self, rotation_matrix):
#         # Ensure the input matrix is a valid rotation matrix
#         assert rotation_matrix.shape == (3, 3), "Input matrix must be a 3x3 rotation matrix"
        
#         # Convert the rotation matrix to a quaternion
#         rotation = Rotation.from_matrix(rotation_matrix.copy())
#         quaternion = rotation.as_quat()
        
#         return quaternion

#     def rotation_matrix_to_euler(self, rotation_matrix):
#         rotation = Rotation.from_matrix(rotation_matrix.copy())
#         euler_angles = rotation.as_euler('xyz', degrees=True)
#         return euler_angles

#     def updata(self, transformation, time_stamp):
#         x, y, z = transformation[0,3], transformation[1,3], transformation[2,3]
#         rotation_matrix = transformation[:3,:3]
#         qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rotation_matrix)
#         pose = [time_stamp, x, y, z,  qx, qy, qz, qw]
#         self.tum_pose.append(pose)

#         # Update the plot
#         self.line.set_xdata(np.append(self.line.get_xdata(), x))
#         self.line.set_ydata(np.append(self.line.get_ydata(), y))

#         # Plot orientation as an arrow
#         # euler_angles = self.rotation_matrix_to_euler(rotation_matrix)
#         # arrow_length = 0.0
#         # orientation = euler_angles[0]
#         # arrow_dx = arrow_length * np.cos(orientation)
#         # arrow_dy = arrow_length * np.sin(orientation)
#         # self.ax.arrow(x, y, arrow_dx, arrow_dy, head_width=0.1, head_length=0.1, fc='r', ec='r')

#         # Adjust plot limits
#         self.ax.relim()
#         self.ax.autoscale_view()

#     def save_as_tum(self, dir, id):
#         os.makedirs(dir, exist_ok=True)
#         file_path = os.path.join(dir, id + '.tum')
#         # Open the file in write mode
#         with open(file_path, 'w') as file:
#             # Convert each element to a string and join them with a delimiter (e.g., comma)
#             file.write(','.join(map(str, self.tum_pose)))

