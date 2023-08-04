import numpy as np
import open3d as o3d
from threading import Thread
import matplotlib.pyplot as plt
import copy
import argparse
from datetime import datetime
from scipy.spatial.transform import Rotation as R


class VisualizePCD(Thread):

    def __init__(self):
        super().__init__()
        self.vis = None
        # self.pcd = pcd
        # self.pcd.points = pcd

    def visualize(self, pcd):
        # Create a visualizer object
        self.vis = o3d.visualization.VisualizerWithEditing()
        self.vis.create_window()
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
        self.vis.close()

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

        # Calculate the centroid of the source and target points
        source_centroid = np.mean(sour, axis=0)
        target_centroid = np.mean(targ, axis=0)

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

        rot_err = rot - np.eye(3)
        transl_err = transl
        # Calculate the mean squared error
        #mse = np.mean(np.sum((targ - (sour @ rot.T + transl)) ** 2, axis=1))

        return rot_err, transl_err

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
        print("")

        rot_err, transl_err = self.registration_error(np.asarray(source.points), np.asarray(target.points), trans)
        print(f'Rotational error: {rot_err}, Translational error: {transl_err}')
        print("")

        correspondences = icp_fine.correspondence_set
        print("Correspondence Set:")
        print(correspondences)
        # print("")

        return transformation_icp, information_icp

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
                transformation_icp, information_icp = self.pairwise_registration(
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
        return pose_graph

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp = source_temp.transform(transformation)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw([source_temp, target_temp])

    def visualize_mw(self, sour, targ, voxel_size=0.0006):
        # Multiway REGISTRATION START--------------------
        start = datetime.now()
        # getting the date and time from the current date and time in the given format
        start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        print('\nMultiway REGISTRATION Started', start_date_time, '\n')

        voxel_size = voxel_size  # in pairwise_registration(...) radius=0.0006 * 2.0,max_nn=30
        print("voxel_size =", voxel_size)
        pcds_down = []
        source_down = sour.voxel_down_sample(voxel_size=voxel_size)
        pcds_down.append(source_down)
        print(
            f'Source Pointcloud down sampled from {len(np.asarray(sour.points))} points to {len(np.asarray(source_down.points))} points.')
        target_down = targ.voxel_down_sample(voxel_size=voxel_size)
        pcds_down.append(target_down)
        print(
            f'Target Pointcloud down sampled from {len(np.asarray(targ.points))} points to {len(np.asarray(target_down.points))} points.')
        print("Full registration ...")
        max_correspondence_distance_coarse = voxel_size * 25
        max_correspondence_distance_fine = voxel_size * 2.5
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph = self.full_registration(pcds_down,
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

        print("Transform points and display")
        for point_id in range(len(pcds_down)):
            print(pose_graph.nodes[point_id].pose)
            pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

        finish = datetime.now()
        # getting the date and time from the current date and time in the given format
        finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        print('Multiway REGISTRATION Finished', finish_date_time,
              "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())

        source = pcds_down[0]
        target = pcds_down[1]
        source.paint_uniform_color([1, 0.706, 0])
        target.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw([source, target])
        # --------------------Multiway REGISTRATION END

    def visualize_mob_strips(self, sour, targ):
        sour.paint_uniform_color([1, 0.706, 0])
        targ.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw([sour, targ])

    def visualize_ransac(self, sour, targ, voxel_size=0.3):
        # # Point-to-plane ICP REGISTRATION START--------------------
        # # Not working on mob-stat. Stat-stat places point clouds one on the top of another.
        # threshold = voxel_size
        # trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
        #                          [0.0, 1.0, 0.0, 0.0],
        #                          [0.0, 0.0, 1.0, 0.0],
        #                          [0.0, 0.0, 0.0, 1.0]])
        # sour.estimate_normals(
        #     o3d.geometry.KDTreeSearchParamHybrid(radius=0.0006 * 2.0,
        #                                          max_nn=30))  # VY voxel_size=0.02
        # targ.estimate_normals(
        #     o3d.geometry.KDTreeSearchParamHybrid(radius=0.0006 * 2.0,
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
        # self.draw_registration_result(sour, targ, reg_p2l.transformation)
        # # Point-to-plane ICP REGISTRATION FINISH--------------------

        # # ROBUST ICP REGISTRATION START--------------------
        # trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
        #                          [0.0, 1.0, 0.0, 0.0],
        #                          [0.0, 0.0, 1.0, 0.0],
        #                          [0.0, 0.0, 0.0, 1.0]])
        # # Mean and standard deviation.
        # mu, sigma = 0, 0.1
        # threshold = 50.0
        # print("Using the noisy source point cloud to perform robust ICP.\n")
        # print("Robust point-to-plane ICP, threshold={}:".format(threshold))
        # loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
        # print("Using robust loss:", loss)
        # p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        # reg_p2l = o3d.pipelines.registration.registration_icp(
        #     sour, targ, threshold, init=trans_init, p2l)
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

        # #FGR REGISTRATION START--------------------
        # #Not working on mob-stat. Stat-stat?
        # parser = argparse.ArgumentParser(
        #     'Global point cloud registration example with RANSAC')
        # parser.add_argument('src',
        #                     type=str,
        #                     default=sour,
        #                     nargs='?',
        #                     help='path to src point cloud')
        # parser.add_argument('dst',
        #                     type=str,
        #                     default=targ,
        #                     nargs='?',
        #                     help='path to dst point cloud')
        # parser.add_argument('--voxel_size',
        #                     type=float,
        #                     default=0.05,
        #                     help='voxel size in meter used to down sample inputs')
        # parser.add_argument(
        #     '--distance_multiplier',
        #     type=float,
        #     default=1.5,
        #     help='multiplier used to compute distance threshold'
        #          'between correspondences.'
        #          'Threshold is computed by voxel_size * distance_multiplier.')
        # parser.add_argument('--max_iterations',
        #                     type=int,
        #                     default=1000000,# VY was 64
        #                     help='number of max FGR iterations')
        # parser.add_argument(
        #     '--max_tuples',
        #     type=int,
        #     default=1000000,# VY was 1000
        #     help='max number of accepted tuples for correspondence filtering')
        #
        # args = parser.parse_args()
        # trans_init = np.asarray(
        #     [
        #         [1.0, 0.0, 0.0, 0.0],
        #         [0.0, 1.0, 0.0, 0.0],
        #         [0.0, 0.0, 1.0, 0.0],
        #         [0.0, 0.0, 0.0, 1.0],
        #     ]
        # )
        # voxel_size = voxel_size
        # distance_threshold = args.distance_multiplier * voxel_size
        # print("Distance threshold: ", distance_threshold)
        #
        # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        # src = sour
        # dst = targ
        #
        # print('Down sampling inputs')
        # src_down, src_fpfh = self.preprocess_point_cloud(src, voxel_size)
        # dst_down, dst_fpfh = self.preprocess_point_cloud(dst, voxel_size)
        # # getting the current date and time
        # start = datetime.now()
        # # getting the date and time from the current date and time in the given format
        # start_date_time = start.strftime("%m/%d/%Y, %H:%M:%S")
        # print('\n Fast Global Registration Started ', start_date_time, '\n')
        # print("voxel_size =", voxel_size)
        # print("distance_multiplier =", args.distance_multiplier)
        # print("Distance threshold: ", distance_threshold)
        # result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        #     src_down, dst_down, src_fpfh, dst_fpfh,
        #     o3d.pipelines.registration.FastGlobalRegistrationOption(
        #         maximum_correspondence_distance=distance_threshold,
        #         iteration_number=args.max_iterations,
        #         maximum_tuple_count=args.max_tuples))
        # # getting the current date and time
        # finish = datetime.now()
        # # getting the date and time from the current date and time in the given format
        # finish_date_time = finish.strftime("%m/%d/%Y, %H:%M:%S")
        # print('Fast Global Registration Finished', finish_date_time,
        #       "\nGlobal registration took %.3f sec.\n" % (finish - start).total_seconds())
        #
        # evaluation = o3d.pipelines.registration.evaluate_registration(src, dst, distance_threshold, trans_init)
        # print(evaluation)
        #
        # fitness = result.fitness
        # print("Fitness:")
        # print(fitness)
        # print("")
        #
        # rmse = result.inlier_rmse
        # print("RMSE of all inlier correspondences:")
        # print(rmse)
        # print("")
        #
        # trans = result.transformation
        # print("The estimated transformation matrix:")
        # print(trans)
        # print("")
        #
        # correspondences = result.correspondence_set
        # print("Correspondence Set:")
        # print(correspondences)
        # print("")
        # src.paint_uniform_color([1, 0.706, 0])
        # dst.paint_uniform_color([0, 0.651, 0.929])
        # o3d.visualization.draw([src.transform(result.transformation), dst])
        # #--------------------FGR REGISTRATION END

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
        print("")

        rot_err, transl_err = self.registration_error(np.asarray(sour.points), np.asarray(targ.points), trans)
        print(f"Rotational error: {rot_err}, Translational error: {transl_err}")
        print("")

        correspondences = result.correspondence_set
        print("Correspondence Set:")
        print(correspondences)
        print("")

        # sour.paint_uniform_color([1, 0.706, 0])
        # targ.paint_uniform_color([0, 0.651, 0.929])
        # o3d.visualization.draw([sour.transform(result.transformation), targ])
        self.draw_registration_result(src_down, dst_down, result.transformation)

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

    def vis_close(self):
        # self.vis.remove_geometry(pcd)
        self.vis.destroy_window()
        self.vis.close()
