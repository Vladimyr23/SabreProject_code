import os
# from stat import ST_CTIME
import numpy as np
import open3d as o3d
from threading import Thread
# from sklearn.cluster import KMeans, DBSCAN
# from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from probreg import cpd
from probreg import callbacks
import copy
import argparse
from datetime import datetime
# from scipy.spatial.transform import Rotation as R

from FileLoad import FileLoad
from FileSave import FileSave

class VisualizePCD(Thread):

    def __init__(self):
        super().__init__()
        self.vis = None
        # self.pcd = pcd
        # self.pcd.points = pcd
        self.real_time_flag = False
        self.real_time_files_savedSet = set()
        self.load = FileLoad()
        self.save = FileSave()

    def pick_points(self, pcd):
        print("")
        print(
            "1) Please pick at least three correspondences using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        # vis.destroy_window()

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

    def visualize(self, pcd):
        # Create a visualizer object
        self.vis = o3d.visualization.VisualizerWithEditing()
        self.vis.create_window()

        # # Hidden point removal
        # diameter = np.linalg.norm(
        #     np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
        # camera = [0, 0, diameter]
        # radius = diameter * 10
        # print("Get all points that are visible from given view point")
        # _, pt_map = pcd.hidden_point_removal(camera, radius)
        # print("Visualize result")
        # pcd = pcd.select_by_index(pt_map)

        # opt = vis.get_render_option() # This one and the next lines are to place black background
        # opt.background_color = np.asarray([0, 0, 0])    # Add point cloud to visualizer
        self.vis.add_geometry(pcd)
        pr = o3d.visualization.PickedPoint.coord
        print(pr)
        # This will measure the distance between the last 2 selected points
        # self.vis.register_selection_changed_callback(self.measure_dist) # working with o3d.visualization.VisualizerWithVertexSelection()
        # Start visualizer
        self.vis.run()

        pts_index = self.vis.get_picked_points()
        pts = np.asarray(pcd.points)[pts_index]
        if len(pts) > 1:
            self.measure_dist(pts)

        # DBSCAN clustering method------
        # with o3d.utility.VerbosityContextManager(
        #         o3d.utility.VerbosityLevel.Debug) as cm:
        #     labels = np.array(
        #         pcd.cluster_dbscan(eps=0.02, min_points=100, print_progress=True))
        #
        # max_label = labels.max()
        # print(f"point cloud has {max_label + 1} clusters")
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([pcd])
        # ---------------DBSCAN clustering method end

    def visualize_mob_strips(self, sour, targ, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        pcd_combined = sour + targ
        self.save.save_pcd_file("combined_point_cloud.ply", pcd_combined)
        sour.paint_uniform_color(source_color)
        targ.paint_uniform_color(target_color)
        o3d.visualization.draw([sour, targ])

    def visualize_pcd_transf(self, pcd, transf_mtrx):
        # Create a visualizer object
        self.vis = o3d.visualization.VisualizerWithEditing()
        self.vis.create_window()
        # opt = vis.get_render_option() # This one and the next lines are to place black background
        # opt.background_color = np.asarray([0, 0, 0])    # Add point cloud to visualizer
        self.vis.add_geometry(pcd.transform(transf_mtrx))
        pr = o3d.visualization.PickedPoint.coord
        print(pr)
        # This will measure the distance between the last 2 selected points
        # self.vis.register_selection_changed_callback(self.measure_dist) # working with o3d.visualization.VisualizerWithVertexSelection()
        # Start visualizer
        self.vis.run()

        pts_index = self.vis.get_picked_points()
        pts = np.asarray(pcd.points)[pts_index]
        if len(pts) > 1:
            self.measure_dist(pts)

    def draw_registration_result(self, source, target, transformation, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color(source_color)
        target_temp.paint_uniform_color(target_color)
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])


    def manual_registration(self, source, target, threshold=0.2, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        print("Visualization of two point clouds before manual alignment")
        self.draw_registration_result(source, target, np.identity(4), source_color, target_color)

        # pick points from two point clouds and builds correspondences
        picked_id_source = self.pick_points(source)
        picked_id_target = self.pick_points(target)
        assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
        assert (len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source), 2))
        corr[:, 0] = picked_id_source
        corr[:, 1] = picked_id_target

        # estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(source, target,
                                                o3d.utility.Vector2iVector(corr))

        # point-to-point ICP for refinement
        print("Perform point-to-point ICP refinement")
        # threshold = 0.03  # 3cm distance threshold
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        self.draw_registration_result(source, target, reg_p2p.transformation, source_color, target_color)
        print("")

    def cpd_affine(self, sour, targ, voxel_size=0.7, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        use_cuda = True
        if use_cuda:
            import cupy as cp
            to_cpu = cp.asnumpy
            cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)
        else:
            cp = np
            to_cpu = lambda x: x

        # getting the current date and time
        start = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nCPD_Affine Started', start_date_time, '\n')

        source_down = sour.voxel_down_sample(voxel_size=voxel_size)
        target_down = targ.voxel_down_sample(voxel_size=voxel_size)
        source_down_pt = cp.asarray(source_down.points, dtype=cp.float32)
        target_down_pt = cp.asarray(target_down.points, dtype=cp.float32)

        acpd = cpd.AffineCPD(source_down_pt, use_cuda=use_cuda)
        tf_param, _, _ = acpd.registration(target_down_pt)

        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('CPD_Affine REGISTRATION Finished', finish_date_time,
              "\nNon-rigid registration took %.3f sec.\n" % (finish - start).total_seconds())
        print("result: ", to_cpu(tf_param.b), to_cpu(tf_param.t))

        print("Performing Non-Rigid transformation on raw data ...")
        sour_result = tf_param.transform(source_down_pt)
        sour_pc = o3d.geometry.PointCloud()
        sour_pc.points = o3d.utility.Vector3dVector(to_cpu(sour_result))

        if not self.real_time_flag:
            sour_pc.paint_uniform_color(source_color)
            target_down.paint_uniform_color(target_color)
            o3d.visualization.draw_geometries([sour_pc, target_down])


    # Function to calculate rotational and translational error
    def registration_error(self, sour, targ, transformation):
        # # Make source and target of the same size
        # minimum_len = min(len(sour), len(targ))
        # source = sour[:minimum_len, :3]
        # target = sour[:minimum_len, :3]
        # # Apply transformation to point cloud
        # source_transformed = np.dot(transformation[:3, :3], source.T).T + transformation[:3, 3]
        # # Compute the difference between the transformed source and target point clouds
        # diff = np.subtract(target, source_transformed)
        # # RMSE of the difference
        # rmse = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
        # # Compute the rotational error using quaternions
        # r = R.from_matrix(transformation)
        # q = r.as_quat()
        # q_target = R.from_matrix(np.identity(3)).as_quat()
        # rot_error = np.arccos(np.abs(np.dot(q, q_target))) * 180 / np.pi
        # # Compute the translational error
        # trans_error = np.linalg.norm(transformation - np.array([0, 0, 0]))
        # return rmse, rot_error, trans_error
        print('Calculating errors...')
        # Calculate the centroid of the source and target points
        source_centroid = np.mean(sour, axis=0)
        target_centroid = np.mean(targ, axis=0)
        print(f'Sour centroid: {source_centroid}')
        print(f'Targ centroid: {target_centroid}')

        # Calculate the covariance matrix of the source and target points
        source_covariance = np.cov(sour.T)
        target_covariance = np.cov(targ.T)

        # Calculate the singular value decomposition of the covariance matrices
        U_source, S_source, Vt_source = np.linalg.svd(source_covariance)
        U_target, S_target, Vt_target = np.linalg.svd(target_covariance)

        # Calculate the rotation matrix
        rot = Vt_target.T @ U_source.T

        # Calculate the translation vector
        transl = target_centroid - rot @ source_centroid
        print(f'Transl vector: {transl}')

        rot_err = rot - np.eye(3)
        # Mean Absolute error for each axis (row in rot_err)
        rot_mae_xyz = np.mean(np.abs(rot_err), axis=1)

        # Calculating translational error
        transl_xyz = np.divide(np.abs(transl), (np.abs(source_centroid)+np.abs(target_centroid)+np.abs(transl))/3)
        transl_xyz_mae = np.divide(transl_xyz, 100)
        # Calculate the mean squared error
        #mse = np.mean(np.sum((targ - (sour @ rot.T + transl)) ** 2, axis=1))

        return rot_mae_xyz, transl_xyz_mae

    # Pre-process data for FGR and RANSAC Registration
    def preprocess_point_cloud(self, pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
                                                 max_nn=30))
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                                 max_nn=100))
        return pcd_down, pcd_fpfh

    # This function is part of the Multiway registration
    def pairwise_registration(self, source, target, max_correspondence_distance_coarse,
                              max_correspondence_distance_fine):
        print("Apply point-to-plane ICP\n", source, target)
        source.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.0006 * 2.0,
                                                 max_nn=30))  # VY voxel_size=0.02
        target.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.0006 * 2.0,
                                                 max_nn=30))  # VY voxel_size=0.02
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_coarse, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())  # VY removed , o3d.pipelines.registration.TransformationEstimationPointToPlane())
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())  # VY removed , o3d.pipelines.registration.TransformationEstimationPointToPlane())
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            transformation_icp)

        fitness = icp_fine.fitness
        print("Fitness:")
        print(fitness)
        print("")

        rmse = icp_fine.inlier_rmse
        print("RMSE of all inlier correspondences:")
        print(rmse)
        print("")

        trans = icp_fine.transformation
        print("The estimated transformation matrix:")
        print(trans)
        print("Saving the transformation matrix in mw_transformation_matrix.txt ...")
        np.savetxt('mw_transformation_matrix.txt', trans)
        print("")

        rot_err, transl_err = self.registration_error(np.asarray(source.points), np.asarray(target.points), trans)
        print(f'Rotational MAE error xyz: {rot_err}, Translational MAE error xyz: {transl_err}')
        print(f'Rotational MAE: {np.mean(rot_err)}, Translational MAE: {np.mean(transl_err)}')
        print("")

        correspondences = icp_fine.correspondence_set
        print("Correspondence Set:")
        print(correspondences)
        # print("")

        return transformation_icp, information_icp, trans

    # This function is part of the Multiway registration
    def full_registration(self, pcds, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)
        print("Pcd's #: ", n_pcds)
        for source_id in range(n_pcds - 1):
            for target_id in range(source_id + 1, n_pcds):
                transformation_icp, information_icp, trans = self.pairwise_registration(
                    pcds[source_id], pcds[target_id],
                    max_correspondence_distance_coarse,
                    max_correspondence_distance_fine)
                print("Build o3d.pipelines.registration.PoseGraph")
                if target_id == source_id + 1:  # odometry case
                    odometry = np.dot(transformation_icp, odometry)
                    pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            np.linalg.inv(odometry)))
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=False))
                else:  # loop closure case
                    pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                                 target_id,
                                                                 transformation_icp,
                                                                 information_icp,
                                                                 uncertain=True))
        return pose_graph, trans

    def visualize_mw(self, sour, targ, voxel_size=0.0006, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        # Multiway REGISTRATION START--------------------
        start = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nMultiway REGISTRATION Started', start_date_time, '\n')
        pcds = []
        pcds.append(sour)
        pcds.append(targ)
        voxel_size = voxel_size  # in pairwise_registration(...) radius=0.0006 * 2.0,max_nn=30
        print("voxel_size =", voxel_size)
        pcds_down = []
        source_down = sour.voxel_down_sample(voxel_size=voxel_size)
        pcds_down.append(source_down)
        print(f'Source Pointcloud down sampled from {len(np.asarray(sour.points))} points '
              f'to {len(np.asarray(source_down.points))} points.')
        target_down = targ.voxel_down_sample(voxel_size=voxel_size)
        pcds_down.append(target_down)
        print(f'Target Pointcloud down sampled from {len(np.asarray(targ.points))} points '
              f'to {len(np.asarray(target_down.points))} points.')
        print("Full registration ...")
        max_correspondence_distance_coarse = voxel_size * 25
        max_correspondence_distance_fine = voxel_size * 2.5
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph, trans = self.full_registration(pcds_down,
                                                       max_correspondence_distance_coarse,
                                                       max_correspondence_distance_fine)

        print("Optimizing PoseGraph ...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0)
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            o3d.pipelines.registration.global_optimization(
                pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option)

        print("Transform down-sampled points ")
        for point_id in range(len(pcds_down)):
            print(pose_graph.nodes[point_id].pose)
            pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

        print("Transform original points and display")
        for point_id in range(len(pcds)):
            print(pose_graph.nodes[point_id].pose)
            pcds[point_id].transform(pose_graph.nodes[point_id].pose)

        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('Multiway REGISTRATION Finished', finish_date_time,
              "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())

        source = pcds[0]
        target = pcds[1]
        pcd_combined = source + target
        self.save.save_pcd_file("mw_combined_point_cloud.ply", pcd_combined)
        # source.paint_uniform_color(source_color)
        # target.paint_uniform_color(target_color)
        # o3d.visualization.draw([source, target])
        if not self.real_time_flag:
            source.paint_uniform_color(source_color)
            target.paint_uniform_color(target_color)
            o3d.visualization.draw([source, target])
        # --------------------Multiway REGISTRATION END

    def visualize_ICP(self, sour, targ, voxel_size=0.02,
                      source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        # # Point-to-plane ICP REGISTRATION START--------------------
        # point to plane ICP
        o3d.visualization.draw_geometries([sour, targ])
        current_transformation = np.identity(4)
        threshold = 20000000
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            #relative_fitness=0.000001,
            #relative_rmse=0.000001,
            max_iteration=2000)

        print('Down sampling inputs')
        src_down, src_fpfh = self.preprocess_point_cloud(sour, voxel_size)
        print(f'Source Point cloud down sampled from {len(np.asarray(sour.points))} '
              f'points to {len(np.asarray(src_down.points))} points.')
        targ_down, targ_fpfh = self.preprocess_point_cloud(targ, voxel_size)
        print(f'Target Point cloud down sampled from {len(np.asarray(targ.points))} points '
              f'to {len(np.asarray(targ_down.points))} points.')

        o3d.visualization.draw_geometries([src_down, targ_down])
        print("Initial alignment")
        evaluation = o3d.pipelines.registration.evaluate_registration(
            src_down, targ_down, threshold, current_transformation)
        print(evaluation, "\n")

        # print("Point-to-Plane ICP applied to down-sampled point-clouds")

        print("Point-to-Plane ICP registration is applied on original point")
        # print("   clouds to refine the alignment. Distance threshold 0.02.")
        result_icp = o3d.pipelines.registration.registration_icp(
            src_down, targ_down, threshold, current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria)
        current_transformation = result_icp.transformation
        print(result_icp)
        print("Transformation is:")
        print(current_transformation)

        evaluation = o3d.pipelines.registration.evaluate_registration(
            src_down, targ_down, threshold, current_transformation)
        print("Evaluation: ", evaluation, "\n")

        # source_temp = copy.deepcopy(sour)
        # target_temp = copy.deepcopy(targ)
        sour.paint_uniform_color(source_color)
        targ.paint_uniform_color(target_color)

        pcd_combined = sour.transform(result_icp.transformation) + targ
        o3d.visualization.draw(pcd_combined)

        # # Not working on mob-stat. Stat-stat places point clouds one on the top of another.
        # threshold = voxel_size
        # trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
        #                          [0.0, 1.0, 0.0, 0.0],
        #                          [0.0, 0.0, 1.0, 0.0],
        #                          [0.0, 0.0, 0.0, 1.0]])
        # sour.estimate_normals(
        #     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
        #                                          max_nn=30))  # VY voxel_size=0.02
        # targ.estimate_normals(
        #     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
        #                                          max_nn=30))  # VY voxel_size=0.02
        # print("Apply point-to-plane ICP")
        # reg_p2l = o3d.pipelines.registration.registration_icp(
        #     sour, targ, threshold, trans_init,
        #     o3d.pipelines.registration.TransformationEstimationPointToPlane())
        # print(reg_p2l)
        # print("Transformation is:")
        # print(reg_p2l.transformation)
        # evaluation = o3d.pipelines.registration.evaluate_registration(
        #     sour, targ, threshold, trans_init)
        # print(evaluation, "\n")
        # #self.draw_registration_result(sour, targ, reg_p2l.transformation, source_color, target_color)
        # o3d.visualization.draw_geometries([sour.transform(reg_p2l.transformation), targ])
        # # Point-to-plane ICP REGISTRATION FINISH--------------------

        # # ROBUST ICP REGISTRATION START--------------------
        # trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
        #                          [0.0, 1.0, 0.0, 0.0],
        #                          [0.0, 0.0, 1.0, 0.0],
        #                          [0.0, 0.0, 0.0, 1.0]])
        # # Mean and standard deviation.
        # mu, sigma = 0, 0.1
        # threshold = 0.02 #VY was 50
        # print("Using the noisy source point cloud to perform robust ICP.\n")
        # print("Robust point-to-plane ICP, threshold={}:".format(threshold))
        # loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
        # print("Using robust loss:", loss)
        # p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        # reg_p2l = o3d.pipelines.registration.registration_icp(
        #     sour, targ, threshold, trans_init, p2l)
        # print(reg_p2l)
        # print("Transformation is:")
        # print(reg_p2l.transformation)
        #
        # source_temp = copy.deepcopy(sour)
        # target_temp = copy.deepcopy(targ)
        # source_temp.paint_uniform_color([1, 0.706, 0])
        # target_temp.paint_uniform_color([0, 0.651, 0.929])
        # source_temp.transform(reg_p2l.transformation)
        # o3d.visualization.draw([source_temp, target_temp])
        # # --------------------ROBUST ICP REGISTRATION END

    def visualize_FGR(self, sour, targ, voxel_size=0.3, source_color=(1, 0.706, 0),
                             target_color=(0, 0.651, 0.929)):
        #FGR REGISTRATION START--------------------
        #Not working on mob-stat. Stat-stat?
        parser = argparse.ArgumentParser(
            'Global point cloud registration example with RANSAC')
        parser.add_argument('src',
                            type=str,
                            default=sour,
                            nargs='?',
                            help='path to src point cloud')
        parser.add_argument('dst',
                            type=str,
                            default=targ,
                            nargs='?',
                            help='path to dst point cloud')
        parser.add_argument('--voxel_size',
                            type=float,
                            default=0.05,
                            help='voxel size in meter used to down sample inputs')
        parser.add_argument(
            '--distance_multiplier',
            type=float,
            default=50, #VY was1.5
            help='multiplier used to compute distance threshold'
                 'between correspondences.'
                 'Threshold is computed by voxel_size * distance_multiplier.')
        parser.add_argument('--max_iterations',
                            type=int,
                            default=512,# VY was 64
                            help='number of max FGR iterations')
        parser.add_argument(
            '--max_tuples',
            type=int,
            default=100000,# VY was 1000
            help='max number of accepted tuples for correspondence filtering')

        args = parser.parse_args()
        trans_init = np.asarray(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        voxel_size = voxel_size
        distance_threshold = args.distance_multiplier * voxel_size
        print("Distance threshold: ", distance_threshold)

        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        src = sour
        dst = targ

        print('Down sampling inputs')
        src_down, src_fpfh = self.preprocess_point_cloud(src, voxel_size)
        dst_down, dst_fpfh = self.preprocess_point_cloud(dst, voxel_size)
        # getting the current date and time
        start = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        print('\n Fast Global Registration Started ', start_date_time, '\n')
        print("voxel_size =", voxel_size)
        print("distance_multiplier =", args.distance_multiplier)
        print("Distance threshold: ", distance_threshold)
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            src_down, dst_down, src_fpfh, dst_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold,
                iteration_number=args.max_iterations,
                maximum_tuple_count=args.max_tuples))
        # getting the current date and time
        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('Fast Global Registration Finished', finish_date_time,
              "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())

        evaluation = o3d.pipelines.registration.evaluate_registration(src, dst, distance_threshold, trans_init)
        print(evaluation)

        fitness = result.fitness
        print("Fitness:")
        print(fitness)
        print("")

        rmse = result.inlier_rmse
        print("RMSE of all inlier correspondences:")
        print(rmse)
        print("")

        trans = result.transformation
        print("The estimated transformation matrix:")
        print(trans)
        print("")

        rot_err, transl_err = self.registration_error(np.asarray(sour.points), np.asarray(targ.points), trans)
        print(f'Rotational MAE error xyz: {rot_err}, Translational MAE error xyz: {transl_err}')
        print(f'Rotational MAE: {np.mean(rot_err)}, Translational MAE: {np.mean(transl_err)}')
        print("")

        correspondences = result.correspondence_set
        print("Correspondence Set:")
        print(correspondences)
        print("")
        src.paint_uniform_color(source_color)
        dst.paint_uniform_color(target_color)
        o3d.visualization.draw([src.transform(result.transformation), dst])
        #--------------------FGR REGISTRATION END

    def visualize_ransac(self, sour, targ, voxel_size=0.3, source_color=(1, 0.706, 0), target_color=(0, 0.651, 0.929)):
        # RANSAC REGISTRATION START--------------------
        parser = argparse.ArgumentParser(
            'Global point cloud registration example with RANSAC')
        parser.add_argument('src',
                            type=str,
                            default=sour,
                            nargs='?',
                            help='path to src point cloud')
        parser.add_argument('dst',
                            type=str,
                            default=targ,
                            nargs='?',
                            help='path to dst point cloud')
        parser.add_argument('--voxel_size',
                            type=float,
                            default=0.3,
                            help='voxel size in meter used to down sample inputs')  # VY changed from 0.05
        parser.add_argument(
            '--distance_multiplier',
            type=float,
            default=2.5,
            help='multiplier used to compute distance threshold'
                 'between correspondences.'
                 'Threshold is computed by voxel_size * distance_multiplier.')  # VY changed from 1.5
        parser.add_argument('--max_iterations',
                            type=int,
                            default=1000000,
                            help='number of max RANSAC iterations')
        parser.add_argument('--confidence',
                            type=float,
                            default=0.999,
                            help='RANSAC confidence')
        parser.add_argument(
            '--mutual_filter',
            action='store_true',
            help='whether to use mutual filter for putative correspondences')

        args = parser.parse_args()

        # getting the current date and time
        start = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nRANSAC Started', start_date_time, '\n')

        voxel_size = voxel_size
        print("voxel_size =", voxel_size)
        print("distance_multiplier =", args.distance_multiplier)
        distance_threshold = args.distance_multiplier * voxel_size
        print("Distance threshold: ", distance_threshold)

        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        # print('Reading inputs')
        # src = source
        # dst = target

        print('Down sampling inputs')
        src_down, src_fpfh = self.preprocess_point_cloud(sour, voxel_size)
        dst_down, dst_fpfh = self.preprocess_point_cloud(targ, voxel_size)

        max_validation = np.min([len(src_down.points), len(dst_down.points)]) // 2
        print("max_validation replaces args.confidence in static-static and mobile-static", max_validation)

        print('Running RANSAC\n')
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down, dst_down, src_fpfh, dst_fpfh,  # True,
            mutual_filter=args.mutual_filter,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.
            TransformationEstimationPointToPoint(True),
            ransac_n=3,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                args.max_iterations, max_validation))  # max_validation replaces args.confidence in mobile-static
        # getting the current date and time
        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('RANSAC Finished', finish_date_time,
              "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())

        fitness = result.fitness
        print("Fitness:")
        print(fitness)
        print("")

        rmse = result.inlier_rmse
        print("RMSE of all inlier correspondences:")
        print(rmse)
        print("")

        trans = result.transformation
        print("The estimated transformation matrix:")
        print(trans)
        print("Saving the transformation matrix in ransac_transformation_matrix.txt ...")
        np.savetxt('ransac_transformation_matrix.txt', trans)
        print("")

        rot_err, transl_err = self.registration_error(np.asarray(sour.points), np.asarray(targ.points), trans)
        print(f'Rotational MAE error xyz: {rot_err}, Translational MAE error xyz: {transl_err}')
        print(f'Rotational MAE: {np.mean(rot_err)}, Translational MAE: {np.mean(transl_err)}')
        print("")

        correspondences = result.correspondence_set
        print("Correspondence Set:")
        print(correspondences)
        print("")

        pcd_combined = sour.transform(result.transformation) + targ
        self.save.save_pcd_file("ransac_combined_point_cloud.ply", pcd_combined)

        # self.cpd_affine(sour.transform(result.transformation), targ, 0.6)

        # o3d.visualization.draw([sour.transform(result.transformation), targ])
        if not self.real_time_flag:
            # sour.paint_uniform_color(source_color)
            # targ.paint_uniform_color(target_color)
            #o3d.visualization.draw([sour.transform(result.transformation), targ])
            o3d.visualization.draw(pcd_combined) # VY Visualization that worked last
            # o3d.visualization.draw_geometries([sour.transform(result.transformation), targ])
            # self.draw_registration_result(sour, targ, trans, source_color, target_color)
        # # result ICP Local refinement
        # distance_threshold = voxel_size * 0.02 # changed from * 0.4
        # print(":: Point-to-plane ICP registration is applied on original point")
        # print("   clouds to refine the alignment. This time we use a strict")
        # print("   distance threshold %.3f." % distance_threshold)
        # result_icp_refined = o3d.pipelines.registration.registration_icp(
        #     sour.transform(result.transformation), targ, distance_threshold, result.transformation)
        # print(result_icp_refined)
        # o3d.visualization.draw([sour.transform(result.transformation).transform(result_icp_refined.transformation), targ])
        # --------------------RANSAC REGISTRATION END

    def real_time_search_load_pcd(self):
        mypath = str(os.getcwd()) + '/real-time_workdir/'
        # print(mypath)

        # Retrieve a set of file paths
        nameSet = set()
        for file in os.listdir(mypath):
            fullpath = os.path.join(mypath, file)
            if os.path.isfile(fullpath):
                nameSet.add(file)

        # Create tuples
        retrieved_set = set()
        for name in nameSet:
            # stat = os.stat(os.path.join(mypath, name))
            # time = ST_CTIME
            # size=stat.ST_SIZE If you add this, you will be able to detect file size changes as well.
            # Also consider using ST_MTIME to detect last time modified
            retrieved_set.add(name)

        # Compare set with saved set to find new files
        new_set = retrieved_set - self.real_time_files_savedSet
        # Compare set with saved set to find removed files
        deletedSet = self.real_time_files_savedSet - retrieved_set

        #Load files with names from new_set
        # print(new_set)
        if bool(new_set):
            filename = mypath + list(new_set)[0]
            print(filename)
            target = self.load.load_file(filename)

            # Update saved set
            self.real_time_files_savedSet.add(list(new_set)[0])
            return target

    def visualize_real_time(self, source):
        self.real_time_flag = True
        # to add new points each dt secs.
        dt = 20
        # getting the current date and time
        start_time = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start_time.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nReal-time visualization Started', start_date_time, '\n')
        # create visualizer and window.
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        # vis = o3d.visualization.draw(source)
        self.vis.add_geometry(source)
        previous_t = start_time
        # run non-blocking visualization.
        # To exit, press 'q' or click the 'x' of the window.
        keep_running = True
        while keep_running:
            if (datetime.now() - previous_t).total_seconds() > dt:
                target = self.real_time_search_load_pcd()
                if target is not None:
                    dists = source.compute_point_cloud_distance(target)
                    dists = np.asarray(dists)
                    ind = np.where(dists > 0.01)[0]
                    # source_filtered = source.select_by_index(ind)
                    target_filtered = target.select_by_index(ind)
                    # Options (uncomment each to try them out):
                    # 1) extend with ndarrays.
                    source.points.extend(np.asarray(target_filtered.points))

                    # 2) extend with Vector3dVector instances.
                    # pcd.points.extend(
                    #     o3d.utility.Vector3dVector(np.random.rand(n_new, 3)))

                    # 3) other iterables, e.g
                    # pcd.points.extend(np.random.rand(n_new, 3).tolist())

                    self.vis.update_geometry(source)
                previous_t = datetime.now()

            keep_running = self.vis.poll_events()
            self.vis.update_renderer()

        self.vis.destroy_window()
        # # Global settings.
        # dt = 3e-2  # to add new points each dt secs.
        # t_total = 100  # total time to run this script.
        # n_new = 10  # number of points that will be added each iteration.
        # self.real_time_flag = True
        #
        # # Create a visualizer object
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        # # opt = vis.get_render_option() # This one and the next lines are to place black background
        # # opt.background_color = np.asarray([0, 0, 0])    # Add point cloud to visualizer
        # self.vis.add_geometry(source)
        # exec_times = []
        #
        # # getting the current date and time
        # start = datetime.now()
        # # getting the date and time from the current date and time in the given format
        # start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        # print('\nReal-time visualization Started', start_date_time, '\n')
        # current_t = start
        # t0 = current_t
        #
        # while (current_t - t0).total_seconds() < t_total:
        #
        #     previous_t = datetime.now()
        #     target = self.real_time_search_load_pcd()
        #     if target is not None:
        #         while (current_t - previous_t).total_seconds() < dt:
        #             s = datetime.now()
        #
        #             # Options (uncomment each to try it out):
        #             # 1) extend with ndarrays.
        #             self.vis.add_geometry(target)
        #             # source.intensity.extend(np.asarray(target.intensity))
        #
        #             # 2) extend with Vector3dVector instances.
        #             # source.points.extend(
        #             #     o3d.utility.Vector3dVector(np.random.rand(n_new, 3)))
        #
        #             # 3) other iterables, e.g
        #             # source.points.extend(np.random.rand(n_new, 3).tolist())
        #
        #             self.vis.update_geometry(source)
        #
        #             # getting the current date and time
        #             # finish = datetime.now()
        #             # getting the date and time from the current date and time in the given format
        #             # finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        #             # print('RANSAC Finished', finish_date_time,
        #             #       "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())
        #             current_t = datetime.now()
        #             exec_times.append((datetime.now() - s).total_seconds())
        #
        #     self.vis.poll_events()
        #     self.vis.update_renderer()
        #
        # print(f"Using extend\t\t\t# Points: {len(source.points)},\n"
        #       f"\t\t\t\t\t\tMean execution time:{np.mean(exec_times)}")
        # self.real_time_flag = False
        # self.vis.destroy_window()

    def vis_segm_DBSCAN(self, pcd):
        # getting the current date and time
        start_time = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start_time.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nDBSCAN Segmentation process started', start_date_time, '\n')
        # # Get points and transform it to a numpy array:
        # points = np.asarray(pcd.points).copy()
        # # Normalisation:
        # print("Normalization ...")
        # scaled_points = StandardScaler().fit_transform(points)
        # # Clustering:
        # print("Clustering ...")
        # model = DBSCAN(eps=0.15, min_samples=100)
        # model.fit(scaled_points)
        #
        # # Get labels:
        # print("Getting cluster labels ...")
        # labels = model.labels_
        # # Get the number of colors:
        #
        # n_clusters = len(set(labels))
        # # Mapping the labels classes to a color map:
        # colors = plt.get_cmap("tab20")(labels / (n_clusters if n_clusters > 0 else 1))
        # # Attribute to noise the black color:
        # colors[labels < 0] = 0
        # # Update points colors:
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        print('Down sampling input')
        voxel_size = 0.2
        print("voxel_size =", voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(
            f'Pointcloud down sampled from {len(np.asarray(pcd.points))} points to {len(np.asarray(pcd_down.points))} points.')

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd_down.cluster_dbscan(eps=1.2, min_points=100, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd_down.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # getting the current date and time
        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('DBSCAN Segmentation process Finished', finish_date_time,
              "\nDBSCAN Segmentation process execution time %.3f sec.\n" % (finish - start_time).total_seconds())
        # current_t = datetime.now()
        # exec_times.append((datetime.now() - s).total_seconds())

        # Display:
        o3d.visualization.draw([pcd_down])


    def vis_close(self):
        # self.vis.remove_geometry(pcd)
        self.vis.destroy_window()
        self.vis.close()
