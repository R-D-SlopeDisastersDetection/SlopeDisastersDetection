import math
import string

import numpy as np
import open3d as o3d

class CloudPointGridCutting:
    def __init__(self, x_num : float, y_num : float, point_cloud: o3d.geometry.PointCloud, output_path : string = None,
                 output_file_type : string = ".ply", cut_type :string = 'block_num'):
        """
        This class is used to cut the point cloud into x_block * y_block blocks
        :param x_num: number of blocks in x direction or block size in x direction
        :param y_num: number of blocks in y direction or block size in y direction
        :param point_cloud: input point cloud
        :param output_path: output path （default is None）
        :param output_file_type: output file type （default is ".ply"）
        :param cut_type: cut type, 'block_num' or 'block_size' （default is 'block_num'）
        """
        self.x_num = x_num
        self.y_num = y_num
        self.point_cloud = point_cloud
        self.output_path = output_path
        self.output_file_type = output_file_type
        self.output = []
        self.cut_type = cut_type

    def grid_cutting(self):
        if self.cut_type == 'block_num':
            return self.grid_cutting_by_block_num()

        if self.cut_type == 'block_size':
            return self.grid_cutting_by_block_size()

    def grid_cutting_by_block_num(self):
        """
        Cut the point cloud into x_block * y_block blocks
        :return: x_block * y_block blocks, y_blocks_num, x_blocks_num
        """
        self.output = [[] for i in range(self.y_num)]

        xyz = np.array(self.point_cloud.points)
        colors = np.array(self.point_cloud.colors)

        x_min = xyz[:, 0].min()
        x_max = xyz[:, 0].max()
        y_min = xyz[:, 1].min()
        y_max = xyz[:, 1].max()

        lis = [[[] for i in range(self.x_num)] for j in range(self.y_num)]
        x_tmp = (x_max - x_min) / self.x_num
        y_tmp = (y_max - y_min) / self.y_num

        for i in range(len(xyz)):
            x = int((xyz[i][0] - x_min) / x_tmp)
            y = int((xyz[i][1] - y_min) / y_tmp)
            if x == self.x_num:
                x -= 1
            if y == self.y_num:
                y -= 1
            lis[y][x].append(i)

        for i in range(self.y_num):
            for j in range(self.x_num):
                pc_tmp = o3d.geometry.PointCloud()
                pc_tmp.points = o3d.utility.Vector3dVector(xyz[lis[i][j]])
                pc_tmp.colors = o3d.utility.Vector3dVector(colors[lis[i][j]])
                self.output[i].append(pc_tmp)

        return self.output, self.y_num, self.x_num

    def grid_cutting_by_block_size(self):
        """
        Cut the point cloud into blocks with the same size
        :return: x_block * y_block blocks, y_blocks_num, x_blocks_num
        """
        xyz = np.array(self.point_cloud.points)
        colors = np.array(self.point_cloud.colors)

        x_min = xyz[:, 0].min()
        x_max = xyz[:, 0].max()
        y_min = xyz[:, 1].min()
        y_max = xyz[:, 1].max()

        x_blocks_num = math.ceil((x_max - x_min) / self.x_num)
        y_blocks_num = math.ceil((y_max - y_min) / self.y_num)

        lis = [[[] for i in range(x_blocks_num)] for j in range(y_blocks_num)]
        self.output = [[] for i in range(y_blocks_num)]

        for i in range(len(xyz)):
            x = int((xyz[i][0] - x_min) / x_blocks_num)
            y = int((xyz[i][1] - y_min) / y_blocks_num)
            if x == x_blocks_num:
                x -= 1
            if y == y_blocks_num:
                y -= 1
            lis[y][x].append(i)

        for i in range(y_blocks_num):
            for j in range(x_blocks_num):
                pc_tmp = o3d.geometry.PointCloud()
                pc_tmp.points = o3d.utility.Vector3dVector(xyz[lis[i][j]])
                pc_tmp.colors = o3d.utility.Vector3dVector(colors[lis[i][j]])
                self.output[i].append(pc_tmp)

        return self.output, y_blocks_num, x_blocks_num


    def output_files(self):
        """
        Output the point cloud blocks to the output path
        :return: if you did not set the output path, it will print "Output path is not set"
        """
        if self.output_path is None:
            print("Output path is not set")
            return
        for i in range (self.y_num):
            for j in range (self.x_num):
                if len(np.array(self.output[i][j].points)) == 0:
                    print("Waring: Block " + str(i) + " " + str(j) + " is empty")
                    continue
                o3d.io.write_point_cloud(self.output_path + "/output_" + str(j) + "_" + str(i) + self.output_file_type, self.output[i][j])

    def get_x_block(self):
        return self.x_num

    def set_x_block(self, x_block : int):
        self.x_num = x_block

    def get_y_block(self):
        return self.y_num

    def set_y_block(self, y_block : int):
        self.y_num = y_block

    def get_point_cloud(self):
        return self.point_cloud

    def set_point_cloud(self, point_cloud: o3d.geometry.PointCloud):
        self.point_cloud = point_cloud

    def get_output_path(self):
        return self.output_path

    def set_output_path(self, output_path : string):
        self.output_path = output_path

    def get_output(self):
        return self.output

    def set_cut_type(self, cut_type : string):
        self.cut_type = cut_type

    def get_cut_type(self):
        return self.cut_type


