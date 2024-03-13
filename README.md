# sgbm功能包

是把 https://gitee.com/maxibooksiyi/awesome-sgbm/tree/maxitest/ 这个代码移植到ros1功能包，结合D435i的双目灰度图话题来跑。
订阅双目灰度话题 /camera/infra1/image_rect_raw 和 /camera/infra2/image_rect_raw ，发出16位深度图话题 /image_topic ，深度值单位是毫米

```
rosrun offboard_pkg sgbm_node
```
