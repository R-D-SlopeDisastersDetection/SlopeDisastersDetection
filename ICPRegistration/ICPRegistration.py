import copy
import math

import open3d as o3d
import numpy as np

class ICPRegistration:
    def __init__(self, source, target, threshold, console_output=False):
        self.source = source
        self.target = target
        self.source_fpfh = None
        self.target_fpfh = None
        self.reg_source = None
        self.reg_result = None
        self.threshold = threshold
        self.raw_trans = None
        self.console_output = console_output

    def draw_registration_result(self, transformation):
        """
        Used to show two cloud points, this function will draw the blue and yellow to cloud point
        , and move and rotate the source cloud point according to the transformation matrix
        :param transformation:`4 x 4` float64 numpy array: The estimated transformation matrix
        :return:
        """
        source_temp = copy.deepcopy(self.source)
        target_temp = copy.deepcopy(self.target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def preprocess_dataset(self):
        """
        Initial the Cloud Point Data and extract the feature of the Cloud Point
        :return:
        """
        print(":: Load two point clouds and disturb initial pose.")

        # Feature Extraction
        self.source_fpfh = self.fpfh_feature_extract(self.source, self.threshold)
        self.target_fpfh = self.fpfh_feature_extract(self.target, self.threshold)

    def execute_global_registration(self):
        """
        Global Registration with RANSAC
        """
        distance_threshold = self.threshold * 5
        print(":: RANSAC registration on downsampled point clouds.")
        print("   we use a liberal distance threshold %.3f." % distance_threshold)
        self.reg_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            self.source, self.target, self.source_fpfh, self.target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(10000, 0.99))
        self.raw_trans = copy.deepcopy(self.reg_result.transformation)

    def execute_icp_registration(self, iter_method, iter_threshold, dis_threshold=0.05):
        """
        Accurate Registration with ICP
        :param iter_method: string, use "rmse", "fitness" or "max_iteration" to modify the iteration method
        :param iter_threshold: the threshold of iteration
        :param dis_threshold: float, distance threshold, radius of K-NN in ICP
        :return:
        """
        trans_init = self.reg_result.transformation
        '''
        fitness，用于测量重叠面积（内点对应数/目标点数）。 值越高越好。
        inlier_rmse，它测量所有内点对应的 RMSE。越低越好。
        '''
        print("Initial alignment")
        reg_p2p = o3d.pipelines.registration.evaluate_registration(
            self.source, self.target, dis_threshold, trans_init)
        print(reg_p2p)

        print("Apply point-to-point ICP")
        if iter_method == "rmse":
            self.reg_result = o3d.pipelines.registration.registration_icp(
                self.source, self.target, dis_threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(relative_rmse=iter_threshold))
        elif iter_method == "fitness":
            self.reg_result = o3d.pipelines.registration.registration_icp(
                self.source, self.target, dis_threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=iter_threshold))
        elif iter_method == "max_iteration":
            self.reg_result = o3d.pipelines.registration.registration_icp(
                self.source, self.target, dis_threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iter_threshold))
        else:
            pass
        print(self.reg_result)
        print("Transformation is:")
        print(self.reg_result.transformation)

    def evaluation_matrix(self, reg_p2p):
        """
        Compare the difference of pcd1 and pcd2, includes Hausdorff Distance , Mean Distance, Standard Deviation
        , Fitness, RMSE and Correspondence Set
        :return:
        """
        hausdorff_distance = self.hausdorff_distance(self.reg_source, self.target)
        mean, std = self.point2point_mean_and_std_deviation(self.reg_source, self.target)
        print("The Hausdorff Distance is ", hausdorff_distance)
        print("The mean is ", mean, ", the std is ", std)
        print("The inliner RMSE is ", reg_p2p.inlier_rmse, ", the fitness is ", reg_p2p.fitness)
        self.draw_registration_result(reg_p2p.transformation)

    def cloudpoint_registration(self, icp_method="max_iteration", iter_threshold=10000, dis_threshold=0.05):
        """
        :param icp_method:
        :param iter_threshold:
        :param dis_threshold:
        :return:
        """
        o3d.visualization.draw_geometries([self.source, self.target])
        self.preprocess_dataset()
        self.execute_global_registration()
        print(self.reg_result)

        mean, std = self.point2point_mean_and_std_deviation(self.source, self.target)
        print("RANSAC Finish!\nThe mean is ", mean, ", the std is ", std)
        self.draw_registration_result(np.identity(4))

        self.execute_icp_registration(icp_method, iter_threshold, dis_threshold)
        self.reg_source = copy.deepcopy(self.source)
        self.reg_source.transform(self.reg_result.transformation)
        self.evaluation_matrix(self.reg_result)

    def fpfh_feature_extract(self, pcd, size):
        """
        Use Fast Point Feature Histogram(FPFH) Feature Extraction to get the feature of Cloud Point
        :param pcd: open3d.geometry.PointCloud, origin CloudPoint Object
        :param size: int, the resolution of process, the resolution of Estimate Normals, FPFH is 2*size, 5*size
        :return: pcd_fpfh: open3d.geometry.PointCloud, The FPFH feature extract output of original pcd
        """
        # 法线计算
        radius_normal = size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # FPFH特征提取
        radius_feature = size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_fpfh

    def hausdorff_distance(self, pcd1, pcd2):
        """
        Hausdorff 距离衡量两个点集之间的最大最小距离，是评估点云相似度的一种严格标准。
        :param pcd1: 点云1路径
        :param pcd2:  点云2路径
        :return: hausdorff distance
        """

        # 计算 Hausdorff 距离
        distances = pcd1.compute_point_cloud_distance(pcd2)
        hausdorff = np.max(distances)
        print(f"Hausdorff distance: {self.hausdorff_distance}")
        return hausdorff

    def point2point_mean_and_std_deviation(self, pcd1, pcd2, console_output=False, output_zero=True):
        """
        返回两个点云的平均值差和方差
        :param pcd1:
        :param pcd2:
        :param console_output:
        :param output_zero: 是否将nan替换为0值
        :return: mean_distance: 平均值距离
        :return: std_distance: 标准差距离
        """
        # 计算点到点距离
        distances = pcd1.compute_point_cloud_distance(pcd2)
        mean_distance = 0 if math.isnan(np.mean(distances)) and output_zero else np.mean(distances)
        std_distance = 0 if math.isnan(np.std(distances)) and output_zero else np.std(distances)

        if console_output:
            print(f"Mean distance: {mean_distance}")
            print(f"Standard deviation of distances: {std_distance}")
        return mean_distance, std_distance

    def compute_repeated_part(self, pcd1, pcd2):
        """
        使用 ICP（Iterative Closest Point）算法对齐两个点云，然后计算重叠的点数或比例。
        :param pcd2:
        :param pcd1:
        :return: overlap_ratio: 重叠率
        """

        # 使用 ICP 进行点云配准
        threshold = 0.02
        trans_init = np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd1, pcd2, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        print(f"Transformation matrix:\n{reg_p2p.transformation}")

        # 计算重叠部分
        pcd1.transform(reg_p2p.transformation)
        distances = pcd1.compute_point_cloud_distance(pcd2)
        overlap_ratio = np.sum(np.asarray(distances) < threshold) / len(distances)
        print(f"Overlap ratio: {overlap_ratio}")
        return overlap_ratio

    def voxel_similarity(self, pcd1, pcd2):
        """
        将点云转换为体素网格，然后计算体素网格的重叠率。
        :param pcd1: 点云1路径
        :param pcd2:  点云2路径
        :return: voxel_similarity: 体素相似度
        """

        # 设置体素大小
        voxel_size = 0.02

        # 将点云转换为体素网格
        voxel_grid1 = o3d.geometry.VoxelGrid.create_from_point_cloud(input = pcd1, voxel_size = voxel_size)
        voxel_grid2 = o3d.geometry.VoxelGrid.create_from_point_cloud(input = pcd2, voxel_size = voxel_size)

        # 计算体素网格相似度
        intersection = voxel_grid1 + voxel_grid2
        union = voxel_grid1 | voxel_grid2
        voxel_similarity = len(intersection.voxels) / len(union.voxels)
        print(f"Voxel similarity: {voxel_similarity}")
        return voxel_similarity

    def compute_z_axis(self, pcd_a, pcd_b):
        distances, indices = pcd_a.compute_point_cloud_distance(pcd_b)
        closest_points_a = pcd_a.points[np.asarray(indices)]
        closest_points_b = pcd_b.points
        print(closest_points_b[1])




