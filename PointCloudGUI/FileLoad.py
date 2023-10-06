import os
import numpy as np
import open3d as o3d
import laspy
import pye57
from pykml import parser
import struct
import re
from threading import Thread

class FileLoad(Thread):
    def __init__(self):
        super().__init__()
        #self.file_name = None
        self.pcd = None
        # self.points = None


    def load_file(self, file_name):
        print(file_name)

        if file_name.endswith(".las") or file_name.endswith(".laz"):
            print("[INFO] .las (.laz) file loading")
            try:
                # import lidar .las data and assign to variable
                self.pcd = laspy.read(file_name)
                # examine the available features for the lidar file we have read
                # list(las.point_format.dimension_names)
                #
                # set(list(las.classification))

                # Creating, Filtering, and Writing Point Cloud Data
                # To create 3D point cloud data, we can stack together with the X, Y, and Z dimensions, using Numpy like this.
                point_data = np.stack([self.pcd.X, self.pcd.Y, self.pcd.Z], axis=0).transpose((1, 0))
                self.pcd = o3d.geometry.PointCloud()
                self.pcd.points = o3d.utility.Vector3dVector(point_data)
                # self.points = point_data
                if self.pcd is not None:
                    print("[Info] Successfully read", file_name)

                    # Point cloud
                    return self.pcd

            except Exception:
                print(".las, .laz file load failed")

        elif file_name.endswith(".e57"):
            print("[INFO] .e57 file loading")
            try:
                e57_file = pye57.E57(file_name)

                # other attributes can be read using:
                data = e57_file.read_scan(0)

                # 'data' is a dictionary with the point types as keys
                # assert isinstance(data["cartesianX"], np.ndarray)
                # assert isinstance(data["cartesianY"], np.ndarray)
                # assert isinstance(data["cartesianZ"], np.ndarray)

                point_xyz = np.stack([data["cartesianX"], data["cartesianY"], data["cartesianZ"]]).transpose((1, 0))
                # points_rgb = [data["colorRed"], data["colorGreen"], data["colorBlue"]]
                # points_intensity = data["intensity"]

                self.pcd = o3d.geometry.PointCloud()
                self.pcd.points = o3d.utility.Vector3dVector(point_xyz)
                # self.points = o3d.utility.Vector3dVector(point_xyz)
                # self.points = point_xyz
                # self.pcd.colors = o3d.utility.Vector3dVector(points_rgb)
                # self.pcd.colors[:, 0] = points_intensity
                print("[Info] Successfully read", file_name)
                return self.pcd

            except Exception:
                print(".e57 file load failed")

        elif file_name.endswith(".bin"):
            print("[INFO] .bin file loading")
            try:
                size_float = 4
                list_pcd = []
                with open(file_name, "rb") as f:
                    byte = f.read(size_float * 4)
                    while byte:
                        x, y, z, intensity = struct.unpack("ffff", byte)
                        list_pcd.append([x, y, z])
                        byte = f.read(size_float * 4)
                np_pcd = np.asarray(list_pcd)
                self.pcd = o3d.geometry.PointCloud()
                self.pcd.points = o3d.utility.Vector3dVector(np_pcd)
                print("[Info] Successfully read", file_name)
                return self.pcd

            except Exception:
                print(".bin file load failed")

        elif file_name.endswith(".ply"):
            self.pcd = o3d.io.read_point_cloud(file_name)
            points_xyz = np.asarray(self.pcd.points)
            #self.pcd = o3d.geometry.PointCloud() # No need to do that already a PointCloud
            self.pcd.points = o3d.utility.Vector3dVector(points_xyz)
            # self.points = points_xyz
            if self.pcd is not None:
                print("[Info] Successfully read", file_name)
                # Point cloud
                return self.pcd

        elif file_name.endswith(".pts"):
            try:
                with open(file_name, "r") as f:
                    # Log every 1000000 lines.
                    LOG_EVERY_N = 1000000
                    points_np = []
                    for line in f:
                        if len(line.split()) == 4:
                            x, y, z, i = [num for num in line.split()]
                            points_np.append([float(x), float(y), float(z), float(i)])
                            if (len(points_np) % LOG_EVERY_N) == 0:
                                print('point', len(points_np))
                        elif len(line.split()) == 3:
                            x, y, z = [num for num in line.split()]
                            points_np.append([float(x), float(y), float(z)])
                            if (len(points_np) % LOG_EVERY_N) == 0:
                                print('point', len(points_np))
                        elif len(line.split()) == 5:
                            x, y, z, i, zeroes_v = [num for num in line.split()]
                            points_np.append([float(x), float(y), float(z), float(i)])
                            if (len(points_np) % LOG_EVERY_N) == 0:
                                print('point', len(points_np))
                        elif len(line.split()) == 7:
                            x, y, z, r, g, b, i = [num for num in line.split()]
                            points_np.append([float(x), float(y), float(z),
                                              float(r), float(g), float(b),
                                              float(i)])
                            if (len(points_np) % LOG_EVERY_N) == 0:
                                print('point', len(points_np))
                        else:
                            print("[Info] The file has unregistered format")
                            return
                print('loop end')
                points_arr = np.array(points_np).transpose()
                print(len(points_arr))
                point_xyz = points_arr[:3].transpose()
                print("xyz points shape", point_xyz.shape)
                self.pcd = o3d.geometry.PointCloud()
                self.pcd.points = o3d.utility.Vector3dVector(point_xyz)
                if len(points_arr) == 4:
                    points_intensity = (points_arr[3])/255.0
                    print("intensity points len", points_intensity.shape)
                    points_intensity_rgb = np.vstack((points_intensity,
                                                      points_intensity,
                                                      points_intensity)).T
                    print("intensity_rgb points shape", points_intensity_rgb.shape)
                    self.pcd.colors = o3d.utility.Vector3dVector(points_intensity_rgb)
                elif len(points_arr) == 7:
                    points_red = (points_arr[4]) / 255.0
                    points_green = (points_arr[5]) / 255.0
                    points_blue = (points_arr[6]) / 255.0
                    points_rgb = np.vstack((points_red,
                                            points_green,
                                            points_blue)).T

                    # points_intensity = ((points_arr[3]) / 255.0).T
                    # print("intensity points len", points_intensity.shape)
                    print("rgb points shape", points_rgb.shape)
                    self.pcd.colors = o3d.utility.Vector3dVector(points_rgb)
                    #self.pcd.intensities = o3d.utility.Vector3dVector(points_intensity)
                if self.pcd is not None:
                    filename_ply = str(os.getcwd())+'/pcd_temp/'+str((file_name[:-3].rsplit('/', 1)[-1:])[0])+'ply'
                    print(filename_ply)
                    o3d.io.write_point_cloud(filename_ply, self.pcd)
                    pcd_ply = o3d.io.read_point_cloud(filename_ply)
                    print("[Info] Successfully read", file_name)
                    # Point cloud
                    return pcd_ply

            except Exception:
                print("[Info] Reading .pts file failed", file_name)

        # elif file_name.endswith(".kml"):
        #     try:
        #         with open(file_name, "r") as f:
        #             # Log every 1000000 lines.
        #             LOG_EVERY_N = 1000000
        #             points_np = []
        #             for line in f:
        #                 print(line)
        #                 if len(line.split(",")) == 3 and (line[0].isdigit() or line.startswith("-")):
        #                     y, x, z = [num for num in line.split(",")]
        #                     points_np.append([float(x), float(y), float(z)])
        #                     if (len(points_np) % LOG_EVERY_N) == 0:
        #                         print('point', len(points_np))
        #                 else:
        #                     print("[Info] The file has unregistered format")
        #         print('loop end')
        #         points_arr = np.array(points_np).transpose()
        #         print(len(points_arr))
        #         point_xyz = points_arr[:3].transpose()
        #         # points_intensity = points_arr[3]
        #         self.pcd = o3d.geometry.PointCloud()
        #         self.pcd.points = o3d.utility.Vector3dVector(point_xyz)
        #         if self.pcd is not None:
        #             print("[Info] Successfully read", file_name)
        #             # Point cloud
        #             return self.pcd
        #
        #     except Exception:
        #         print("[Info] Reading .kml file failed", file_name)

        else:
            self.pcd = None
            geometry_type = o3d.io.read_file_geometry_type(file_name)
            print(geometry_type)

            mesh = None
            if geometry_type & o3d.io.CONTAINS_TRIANGLES:
                mesh = o3d.io.read_triangle_model(file_name)
            if mesh is None:
                print("[Info]", file_name, "appears to be a point cloud")
                cloud = None
                try:
                    cloud = o3d.io.read_point_cloud(file_name)
                    # print(type(cloud))
                except Exception:
                    print("[Info] Unknown filename", file_name)
                if cloud is not None:
                    print("[Info] Successfully read", file_name)

                    if not cloud.has_normals():
                        cloud.estimate_normals()
                    cloud.normalize_normals()
                    self.pcd = cloud
                    #self.points = cloud.points
                    self.pcd.points = o3d.utility.Vector3dVector(cloud.points)
                else:
                    print("[WARNING] Failed to read points", file_name)

            if self.pcd is not None or mesh is not None:
                try:
                    if mesh is not None:
                        # Triangle model
                        self._scene.scene.add_model("__model__", mesh)
                    else:
                        # Point cloud
                        return self.pcd

                except Exception as e:
                    print(e)

    def load_kml(self, file_name):
        kml_coordinates = []
        # gpd.io.file.fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
        # gpd.io.file.fiona.drvsupport.supported_drivers['KML'] = 'rw'  # enable KML support which is disabled by default

        try:
            # # Load the KML trajectory
            # # Load KML location data (assuming it contains timestamped GPS coordinates)
            # kml_data = gpd.read_file(file_name)
            #
            # # Extract timestamp and coordinates from KML data
            # timestamps = kml_data['timestamp']
            # latitudes = kml_data['latitude']
            # longitudes = kml_data['longitude']
            # altitudes = kml_data['altitude']
            #
            # kml_coordinates = np.vstack((latitudes, longitudes, altitudes)).T

            with open(file_name) as f:
                pm = parser.parse(f).getroot().Document.Placemark
                # print("got :")
                # print(pm.LineString.coordinates)

            pattern = r'[-+]?\d*\.\d+|[-+]?\d+'  # Regular expression pattern for float numbers
            lines = pm.LineString.coordinates.text.strip().split('\n')  # Split text into lines

            # print(pm.LineString.coordinates.text)
            for line in lines:
                coord = re.findall(pattern, line)
                coord_list = [float(x) for x in coord]
                kml_coordinates.append(coord_list)
                #print(coord_list)
            # for coord in pm.LineString.coordinates:
            #     print(coord)
            #     coord = coord.text.strip()
            #     coord = re.split(',| ', coord)
            #     coord = [float(c) for c in coord]
            #     kml_coordinates.append(coord)
            # print(kml_coordinates)
            return kml_coordinates

        except Exception as e:
            print(e, "[WARNING] Failed to read .kml file:", file_name)
            return

    # def export_image(self, path, width, height):
    #     def on_image(image):
    #         img = image
    #
    #         quality = 9  # png
    #         if path.endswith(".jpg"):
    #             quality = 100
    #         o3d.io.write_image(path, img, quality)
    #
    #     self._scene.scene.scene.render_to_image(on_image)

