from CloudPointGridCutting import CloudPointGridCutting
import open3d as o3d

if __name__ == '__main__':
    point_cloud = o3d.io.read_point_cloud("../dataset_reg/simtest/5331ori.ply")
    x_block = 5
    y_block = 5
    cloud_point_grid_cutting = CloudPointGridCutting(x_block, y_block, point_cloud, 'output', cut_type='block_size')
    blocks = cloud_point_grid_cutting.grid_cutting()
    cloud_point_grid_cutting.output_files()
