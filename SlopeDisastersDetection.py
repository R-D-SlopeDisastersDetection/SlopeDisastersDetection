import math
import string

import numpy as np

from BlockAnalyze.BlockAnalyzeByChange import BlockAnalyzeByChange
from CSF2.CSF2 import CSF2
from cross_intersection.cross_intersection import cross_intersection


class SlopeDisastersDetection:
    def __init__(self, points_cloud_path_list : list[string], x_size:float = 0.5, y_size:float = 0.5 ):
        self.intersection = None
        self.points_cloud_path_list = points_cloud_path_list
        self.pcd_ground_list = []
        self.x_size = x_size
        self.y_size = y_size

    def process(self):
        for points_cloud_path in self.points_cloud_path_list:
            csf = CSF2(points_cloud_path, 'ply')
            csf.process()
            self.pcd_ground_list.append(csf.outfile)
        ##TODO: add the Registration there

        ##
        self.intersection = cross_intersection(self.pcd_ground_list)

        #切块大小预处理
        xyz = np.array(self.intersection[0].points)
        x_min = xyz[:, 0].min()
        x_max = xyz[:, 0].max()
        y_min = xyz[:, 1].min()
        y_max = xyz[:, 1].max()

        x_blocks_num = math.ceil((x_max - x_min) / self.x_size)
        y_blocks_num = math.ceil((y_max - y_min) / self.y_size)
        #分块分析
        block_analyze = BlockAnalyzeByChange(self.intersection[0], self.intersection[1], self.intersection[2],
                                             x_blocks_num, y_blocks_num)
        block_analyze.block_analyze()
        block_analyze.block_analyze(0.1)
        # 使用统计学找异常点
        block_analyze.statistics_analyze(0, 3)
        # 查看统计异常点云
        block_analyze.visualize_color_pcd(block_analyze.exception_pcd, True)
        # 将结果保存为csv
        block_analyze.save_result_as_csv()
        # 将着色点云保存为pcd文件
        block_analyze.save_color_blocks()
