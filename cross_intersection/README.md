# cross_intersection
本项目只包含了一个简单的函数，主要用于计算多个点云的交集部分，并输出每个点云的交集部分。 

This project only includes a simple function, mainly used to calculate 
the intersection part of multiple point clouds and output the intersection part of each point cloud.

请注意，本项目为了提高运行效率的原因，，只适用于凸包形状的点云，对于非凸包形状的点云，可能会出现错误。本项目主要结合微积分的思想构建而成。

Please note that in order to improve the running efficiency, this project is only suitable for point clouds with convex 
hull shapes. For point clouds with non-convex hull shapes, errors may occur. This project is mainly constructed based on
the idea of calculus.
## 1. 环境配置（仅做参考） 
Environment configuration (for reference only)

```
python = 3.9
numpy = 1.22.4
open3d = 0.18.0
```

## 2. 参数说明
Parameter Description
```
该函数主要用于获取所有点云的交集
    :param cloud_list: 需要获取交集的点云列表
    :param precision: 交集的精度，值越小，交集越精确
    :return: 所有点云的交集
```


```
This function is used to get the intersection of all clouds
    :param cloud_list: the list of clouds which need to get the intersection
    :param precision: the precision of the intersection, if the value is smaller, the intersection will be more accurate
    :return: the intersection of all clouds
```

## 3. 使用方法
usage method
```
if __name__ == '__main__':
    cloud_list = []
    cloud_list.append(o3d.io.read_point_cloud("./data/129-scnu-40-40-10cm - Cloud.ply"))
    cloud_list.append(o3d.io.read_point_cloud("./data/130-scnu-40-40-10cm.ply"))
    cloud_list.append(o3d.io.read_point_cloud("./data/133-scnu-40-40-15cm-and-20-20-5cm.ply"))

    result = cross_intersection(cloud_list, 0.5)
    o3d.visualization.draw_geometries(result, window_name='cross_intersection', width=800, height=600)
```