import os
import numpy as np
import open3d as o3d

from pathlib import Path
from vis import Visualizer
from traj import Trajectory
from scipy.spatial.transform import Rotation

# SYSTEM PARAMS
VOXEL_SIZE = 0.2
SEQUENCE = '20220714'
ROOT_DIR = '/home/ibrahim/Bacchus'

# Constants for readability and maintainability
MAP_PATH = os.path.join(ROOT_DIR, 'maps', 'base_map.asc.npy')
SCANS_PATH = os.path.join(ROOT_DIR, 'sequence', SEQUENCE, 'scans')

###############################
class Localizer:
    def __init__(self, max_iterations=30, threshold=0.2, downsample_voxel=VOXEL_SIZE):
        self.threshold = threshold
        self.max_iterations = max_iterations
        self.downsample_voxel = downsample_voxel
        self.target_cloud = None

    def preprocess_cloud(self, cloud):
        cloud  = cloud.voxel_down_sample(voxel_size=self.downsample_voxel)
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
        return cloud
    
    def set_target_cloud(self, target_cloud):
        self.target_cloud = self.preprocess_cloud(target_cloud)

    def align(self, source_cloud, guess):
        source_cloud = self.preprocess_cloud(source_cloud)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_cloud, self.target_cloud, self.threshold, guess,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iterations))
        return reg_p2p.transformation

    def load_cloud(self, path):
        cloud = np.load(path)[:, :3]  # Only keep the points
        return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cloud))

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        # Ensure the input matrix is a valid rotation matrix
        assert rotation_matrix.shape == (3, 3), "Input matrix must be a 3x3 rotation matrix"
        
        # Convert the rotation matrix to a quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        
        return quaternion

    def quaternion_to_rotation_matrix(self, quaternion):
        # Ensure the input quaternion is a valid quaternion
        assert len(quaternion) == 4, "Input must be a 4-element quaternion"

        # Normalize the quaternion to handle numerical precision issues
        quaternion = quaternion / np.linalg.norm(quaternion)

        # Convert the quaternion to a rotation matrix
        rotation = Rotation.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()

        return rotation_matrix
    
def get_prediction_model(poses):
    pred = np.eye(4)  # Identity matrix for SE(3)
    N = len(poses)
    if N < 2:
        return pred
    inverse_pose_N_minus_2 = np.linalg.inv(poses[N - 2])
    pred = np.dot(inverse_pose_N_minus_2, poses[N - 1])
    return pred    

def find_pose_error(reference_pose, target_pose):
    # Calculate the relative transformation (error)
    relative_pose = np.dot(np.linalg.inv(reference_pose), target_pose)

    # Extract translation and rotation from the relative transformation
    translation_error = relative_pose[:3, 3]
    rotation_error = Rotation.from_matrix(relative_pose[:3, :3]).as_rotvec()

    return translation_error, rotation_error


def main():

    assert os.path.exists(MAP_PATH), f"Map file not found at: {MAP_PATH}"
    assert os.path.exists(SCANS_PATH), f"Scans directory not found at: {SCANS_PATH}"

    scans = sorted([os.path.join(SCANS_PATH, file) for file in os.listdir(SCANS_PATH)])
    loc = Localizer()
    traj = Trajectory()

    map_cloud = loc.load_cloud(MAP_PATH).paint_uniform_color([0, 0.651, 0.929])
    map_cloud, _ = map_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)  # Discard unused indices
    loc.set_target_cloud(map_cloud)
    vis = Visualizer()
    vis.add_geometry(map_cloud)

    guess = np.eye(4)
    first_run = True
    scan_cloud = o3d.geometry.PointCloud()
    poses = [guess]
    for scan in scans:

        cloud = np.load(scan)[:,:3]
        scan_cloud.points = o3d.utility.Vector3dVector(cloud)
        scan_cloud.paint_uniform_color([1, 0.706, 0])

        # prediction = get_prediction_model(poses)
        # predicted_pose = np.dot(poses[-1],prediction)
        # aligned_pose = loc.align(scan_cloud, predicted_pose)
        # scan_cloud.transform(aligned_pose)
        # poses.append(aligned_pose)

        guess = loc.align(scan_cloud, guess)
        scan_cloud.transform(guess)
        if first_run: 
            vis.add_geometry(scan_cloud)
            first_run = False
        vis.render()  # Render once per scan for efficiency
        vis.update_geometry(scan_cloud)
    
        scan_time_stamp = Path(scan).stem
        traj.update(guess, scan_time_stamp)
    
    traj.save_as_tum('traj', SEQUENCE)


if __name__ == "__main__":
    main()

