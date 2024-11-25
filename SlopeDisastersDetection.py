import string

from CSF2.CSF2 import CSF2

class SlopeDisastersDetection:
    def __init__(self, points_cloud_path_list : list[string], x_size:float = 0.5, y_size:float = 0.5 ):
        self.points_cloud_path_list = points_cloud_path_list
        self.pcd_ground_list = []
        self.x_size = x_size
        self.y_size = y_size

    def process(self):
        for points_cloud_path in self.points_cloud_path_list:
            csf = CSF2(points_cloud_path, 'ply')
            csf.process()
            self.pcd_ground_list.append(csf.outfile)
        self.pcd_ground =