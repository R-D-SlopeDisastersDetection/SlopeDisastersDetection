# ICPRegistration
本模块主要用于ICP配准的相关操作，包括点云配准、多轮次配准测试、网格化分析等。

## 环境参考

~~~
python 3.9
open3d 0.18.0
numpy 1.22.4
~~~



## 代码使用示例（由于项目合并后续将做调整）
~~~
# if __name__ == "__main__":
#     pcd1 = o3d.io.read_point_cloud("dataset_reg/simtest/format/5331.pcd")
#     pcd2 = o3d.io.read_point_cloud("dataset_reg/simtest/format/5332.pcd")
#     reg = Registration(pcd1, pcd2, 0.05)
#     reg.cloudpoint_registration()
#     output = reg.reg_source
#     print("内部结果")
#     o3d.visualization.draw_geometries([output, pcd2])
#     o3d.visualization.draw_geometries([pcd1.transform(reg.raw_trans), pcd2])

~~~