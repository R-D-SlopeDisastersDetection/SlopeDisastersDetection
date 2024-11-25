import math
import open3d as o3d
import numpy as np

def cross_intersection(cloud_list : list[o3d.geometry.PointCloud], precision : float = 0.2):
    """
    This function is used to get the intersection of all clouds
    :param cloud_list: the list of clouds which need to get the intersection
    :param precision: the precision of the intersection, if the value is smaller, the intersection will be more accurate
    :return: the intersection of all clouds
    """
    #get the min x of all clouds and the max x of all clouds
    clouds_list_x_min = np.array(cloud_list[0].points)[:, 0].min()
    clouds_list_x_max = np.asarray(cloud_list[0].points)[:, 0].max()
    for cloud in cloud_list:
        x_min = np.array(cloud.points)[:, 0].min()
        x_max = np.array(cloud.points)[:, 0].max()
        if x_min < clouds_list_x_min:
            clouds_list_x_min = x_min
        if x_max > clouds_list_x_max:
            clouds_list_x_max = x_max

    #get the number of blocks
    blocks_num = math.ceil((clouds_list_x_max - clouds_list_x_min) / precision)
    # tmp[i][j] means the jth block of the ith cloud [y_min, y_max]
    tmp = [ [ [] for j in range(blocks_num)  ] for i in range (len(cloud_list))]

    #get the y_location of each point in the cloud, which i mean the ith cloud and the j mean the jth block
    for i in range(len(cloud_list)):
        xyz = np.array(cloud_list[i].points)
        for j in range(1, len(cloud_list[i].points)):
            x_tmp = int((xyz[j][0] - clouds_list_x_min) / precision)
            if x_tmp == blocks_num:
                x_tmp -= 1
            tmp[i][x_tmp].append(xyz[j][1])

    #get the y_min and y_max of each block in every cloud
    tmp_y_min_and_y_max =  [ [ [] for j in range(blocks_num)  ] for i in range (len(cloud_list))]

    for i in range(len(cloud_list)):
        for j in range(blocks_num):
            if len(tmp[i][j]) != 0:
                tmp_y_min_and_y_max[i][j] = [min(tmp[i][j]), max(tmp[i][j])]

    #get the y_min and y_max of each block in all clouds
    y_min_max = [[]for i in range(blocks_num)]
    for j in range(blocks_num):
        y_min = 0
        y_max = 0
        for i in range(len(cloud_list)):
            if len(tmp_y_min_and_y_max[i][j]) == 0:
                y_min_max[j] = []
                continue
            if len(tmp_y_min_and_y_max[i][j]) == 0:
                y_min_max[j] = []
                continue
            if i>0 and len(y_min_max[j])==0:
                continue
            if len(y_min_max[j]) == 0 and len(tmp_y_min_and_y_max[i][j]) != 0:
                y_min = tmp_y_min_and_y_max[i][j][0]
                y_max = tmp_y_min_and_y_max[i][j][1]
                continue
            if tmp_y_min_and_y_max[i][j][0] >y_min:
                y_min = tmp_y_min_and_y_max[i][j][0]
            if tmp_y_min_and_y_max[i][j][1] < y_max:
                y_max = tmp_y_min_and_y_max[i][j][1]
        # y_min = min(tmp_y_min_and_y_max[i][j][0] for i in range(len(cloud_list)))
        # y_max = max(tmp_y_min_and_y_max[i][j][1] for i in range(len(cloud_list)))
        y_min_max[j] = [y_min, y_max]

    #get the points location in the block
    output_points_location = [ [ ] for i in range(len(cloud_list)) ]

    for i in range(len(cloud_list)):
        xyz = np.array(cloud_list[i].points)
        for j in range(len(cloud_list[i].points)):
            x_tmp = int((xyz[j][0] - clouds_list_x_min) / precision)
            if x_tmp == blocks_num:
                x_tmp -= 1

            if y_min_max[x_tmp][0] <= xyz[j][1] <= y_min_max[x_tmp][1]:
                output_points_location[i].append(j)

    #get the output cloud
    output = []
    for i in range (len(cloud_list)):
        cloud_tmp = o3d.geometry.PointCloud()
        cloud_tmp.points = o3d.utility.Vector3dVector(np.array(cloud_list[i].points)[output_points_location[i]])
        cloud_tmp.colors = o3d.utility.Vector3dVector(np.array(cloud_list[i].colors)[output_points_location[i]])
        output.append(cloud_tmp)

    return output