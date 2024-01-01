
import numpy as np
import open3d as o3d

class Visualizer:
    def __init__(self) -> None:
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

    def add_coordinate_frame(self):
        # Create a coordinate frame for reference
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.vis.add_geometry(coordinate_frame)

    def add_geometry(self, object):
        self.vis.add_geometry(object)

    def remove_geometry(self, object):
        self.vis.remove_geometry(object)  # Specify the geometry to remove

    def update_geometry(self, object):
        self.vis.update_geometry(object)

    def render(self):
        self.vis.poll_events()  # Check for user interactions (e.g., window closure)
        self.vis.update_renderer()  # Re-render the scene

    
if __name__ == "__main__":
    vis = Visualizer()

    # Create a sphere geometry
    sphere1 = o3d.geometry.TriangleMesh.create_sphere()
    vis.add_geometry(sphere1)

    def create_transform_matrix_from_z(z):
        """Creates a 4x4 transformation matrix that translates along the z-axis."""
        result = np.identity(4)
        result[2, 3] = z
        return result

    # Animate the sphere along the z-axis
    prev_tf = None
    for curr_z in np.arange(0.5, 15.0, 0.005):
        # Reset the sphere to its original position if needed
        if prev_tf is not None:
            sphere1.transform(np.linalg.inv(prev_tf))

        # Create a transformation matrix for the current z position
        curr_tf = create_transform_matrix_from_z(curr_z)
        sphere1.transform(curr_tf)
        prev_tf = curr_tf

        # Update the visualization
        vis.update_geometry(sphere1)
        vis.render()  # Check for user interactions (e.g., window closure)


