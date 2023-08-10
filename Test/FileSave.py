import numpy as np
import open3d as o3d
import laspy
import pye57

class FileSave:

    def __init__(self, file_name, pcd):
        self.file_name = str(file_name)
        self.pcd = pcd
        # self.points = points

    def save_file(self):
        # TODO add code to save point cloud in a different formats
        try:
            if self.pcd is not None:
                print("Saving point cloud to file: ", self.file_name)
                o3d.io.write_point_cloud(self.file_name, self.pcd)
            else:
                print("No point data to save to file: ", self.file_name)
                return
        except Exception as e:
            print("[Exception] Cannot save the file: ", e)

        # print(self.file_name)
        # if self.file_name.endswith(".las") or self.file_name.endswith(".laz"):
        #     print("[INFO] .las (.laz) file loading")
        #     try:
        #         # import lidar .las data and assign to variable
        #         self.pcd = laspy.read(self.file_name)
        #         # examine the available features for the lidar file we have read
        #         # list(las.point_format.dimension_names)
        #         #
        #         # set(list(las.classification))
        #
        #         # Creating, Filtering, and Writing Point Cloud Data
        #         # To create 3D point cloud data, we can stack together with the X, Y, and Z dimensions, using Numpy like this.
        #         point_data = np.stack([self.pcd.X, self.pcd.Y, self.pcd.Z], axis=0).transpose((1, 0))
        #         # self.pcd = o3d.geometry.PointCloud()
        #         # self.points = o3d.utility.Vector3dVector(point_data)
        #         self.points = point_data
        #         if self.pcd is not None:
        #             print("[Info] Successfully read", self.file_name)
        #
        #             # Point cloud
        #             return self.pcd, self.points
        #
        #     except Exception:
        #         print(".las, .laz file load failed")
        #
        # elif self.file_name.endswith(".e57"):
        #     print("[INFO] .e57 file loading")
        #     try:
        #         e57_file = pye57.E57(self.file_name)
        #
        #         # other attributes can be read using:
        #         data = e57_file.read_scan(0)
        #
        #         # 'data' is a dictionary with the point types as keys
        #         # assert isinstance(data["cartesianX"], np.ndarray)
        #         # assert isinstance(data["cartesianY"], np.ndarray)
        #         # assert isinstance(data["cartesianZ"], np.ndarray)
        #
        #         point_xyz = np.stack([data["cartesianX"], data["cartesianY"], data["cartesianZ"]]).transpose((1, 0))
        #         # points_rgb = [data["colorRed"], data["colorGreen"], data["colorBlue"]]
        #         # points_intensity = data["intensity"]
        #
        #         self.pcd = o3d.geometry.PointCloud()
        #         # self.points = o3d.utility.Vector3dVector(point_xyz)
        #         self.points = point_xyz
        #         # self.pcd.colors = o3d.utility.Vector3dVector(points_rgb)
        #         # self.pcd.colors[:, 0] = points_intensity
        #         print("[Info] Successfully read", self.file_name)
        #         return self.pcd, self.points
        #
        #     except Exception:
        #         print(".e57 file load failed")
        #
        # elif self.file_name.endswith(".ply"):
        #     self.pcd = o3d.io.read_point_cloud(self.file_name)
        #     points_xyz = np.asarray(self.pcd.points)
        #     # self.pcd = o3d.geometry.PointCloud()
        #     # self.points = o3d.utility.Vector3dVector(points_xyz)
        #     self.points = points_xyz
        #     if self.pcd is not None:
        #         print("[Info] Successfully read", self.file_name)
        #         # Point cloud
        #         return self.pcd, self.points
        #
        # else:
        #     self.pcd = None
        #     geometry_type = o3d.io.read_file_geometry_type(self.file_name)
        #     print(geometry_type)
        #
        #     mesh = None
        #     if geometry_type & o3d.io.CONTAINS_TRIANGLES:
        #         mesh = o3d.io.read_triangle_model(self.file_name)
        #     if mesh is None:
        #         print("[Info]", self.file_name, "appears to be a point cloud")
        #         cloud = None
        #         try:
        #             if geometry_type == o3d.io.FileGeometry.CONTAINS_POINTS:
        #                 with open(self.file_name, "r") as f:
        #                     points_np = []
        #                     for line in f:
        #                         if len(line.split()) == 4:
        #                             x, y, z, i = [num for num in line.split()]
        #                             points_np.append([float(x), float(y), float(z), int(i)])
        #                         elif len(line.split()) == 3:
        #                             x, y, z = [num for num in line.split()]
        #                             points_np.append([float(x), float(y), float(z)])
        #                         else:
        #                             print("[Info] The file has unregistered format")
        #                 points_arr = np.array(points_np).transpose()
        #                 print(len(points_arr))
        #                 point_xyz = points_arr[:3].transpose()
        #                 # points_intensity = points_arr[3]
        #                 cloud = o3d.geometry.PointCloud()
        #                 cloud.points = o3d.utility.Vector3dVector(point_xyz)
        #             else:
        #                 cloud = o3d.io.read_point_cloud(self.file_name)
        #             # print(type(cloud))
        #         except Exception:
        #             pass
        #         if cloud is not None:
        #             print("[Info] Successfully read", self.file_name)
        #
        #             if not cloud.has_normals():
        #                 cloud.estimate_normals()
        #             cloud.normalize_normals()
        #             self.pcd = cloud
        #             # self.points = cloud.points
        #             self.points = point_xyz
        #         else:
        #             print("[WARNING] Failed to read points", self.file_name)
