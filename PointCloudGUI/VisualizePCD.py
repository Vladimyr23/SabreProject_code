import numpy as np
import open3d as o3d
from threading import Thread
import matplotlib.pyplot as plt


class VisualizePCD(Thread):

    def __init__(self, pcd):
        super().__init__()
        self.vis = None
        self.pcd = pcd
        # self.pcd.points = pcd

    def visualize(self):
        # # Create a visualizer object
        # self.vis = o3d.visualization.VisualizerWithEditing()
        # self.vis.create_window()
        # # opt = vis.get_render_option() # This one and the next lines are to place black background
        # # opt.background_color = np.asarray([0, 0, 0])    # Add point cloud to visualizer
        # self.vis.add_geometry(self.pcd)
        # pr=o3d.visualization.PickedPoint.coord
        # print(pr)
        # # This will measure the distance between the last 2 selected points
        # #self.vis.register_selection_changed_callback(self.measure_dist) # working with o3d.visualization.VisualizerWithVertexSelection()
        # # Start visualizer
        # fig = self.vis.run()
        #
        # pts_index = self.vis.get_picked_points()
        # pts = np.asarray(self.pcd.points)[pts_index]
        # if len(pts) > 1:
        #     self.measure_dist(pts)
        # #self.vis_close()

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                self.pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([self.pcd])


    def pick_points(self):
        print("")
        print(
            "1) Please pick at least three correspondences using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(self.pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        return vis.get_picked_points()

    # def on_point_pick(self):
    #     # Get selected point coordinates
    #     p = self.vis.get_picked_points()[0]
    #     # Add point to line list
    #     if len(line.points) == 0:
    #         line.points[0] = p
    #     elif len(line.points) == 1:
    #         line.points[1] = p
    #         self.vis.add_geometry(line)
    #     else:
    #         line.points = []
    #         line.points[0] = p

    def measure_dist(self, pts):

        if len(pts) > 1:
            point_a = pts[-2]
            point_b = pts[-1]
            # Formula for Euclidean Distance
            dist = np.sqrt(
                (point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2 + (point_a[2] - point_b[2]) ** 2)
            print(f"Point_A: {point_a}")
            print(f"Point_B: {point_b}")
            print(f"Distance: {dist}")

        else:
            print("Select at least 2 points to calculate Dist")
        
    def vis_close(self):
        self.vis.remove_geometry([self.pcd])
        self.vis.destroy_window()
        self.vis.close()