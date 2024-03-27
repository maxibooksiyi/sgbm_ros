# sgbm功能包
基于D435i的双目灰度图做立体匹配输出深度图
## 部署
下载到工作空间再catkin_make即可。  

## 使用

### sgbm_node.cpp
是把 https://gitee.com/maxibooksiyi/awesome-sgbm/tree/maxitest/ 这个代码移植到ros1功能包，结合D435i的双目灰度图话题来跑。  
订阅双目灰度话题 /camera/infra1/image_rect_raw 和 /camera/infra2/image_rect_raw ，发出16位深度图话题 /sgbm_depth_image ，深度值单位是毫米。  

```
rosrun offboard_pkg sgbm_node
```

sgbm_node.cpp是基于李迎松的SGBM代码(https://gitee.com/maxibooksiyi/awesome-sgbm )改的，自己加上了视差图转深度图，出来的深度图效果还可以，就是计算耗时太长，得8-9秒才算出一张深度图，才能发出一个深度图话题。  

### opencv_sgbm_node.cpp
所以我又基于opencv的sgbm函数弄了一个opencv_sgbm_node.cpp。这个可以在普通笔记本上基于CPU实时运行。  
订阅双目灰度话题 /camera/infra1/image_rect_raw 和 /camera/infra2/image_rect_raw ，发出16位深度图话题 /sgbm_depth_image ，深度值单位是毫米。  

```
rosrun offboard_pkg opencv_sgbm_node
```


