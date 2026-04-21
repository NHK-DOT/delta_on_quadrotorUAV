# AprilTag 论文整理

本目录包含 3 篇核心资料：

1. `pdf/olson2011tags.pdf`
   `markdown/olson2011tags.md`
   原始 AprilTag 论文，解释了为什么它比普通二维码更适合机器人定位。

2. `pdf/wang2016iros.pdf`
   `markdown/wang2016iros.md`
   AprilTag 2 论文，重点是更快、更稳、更适合小标签与实时检测。

3. `pdf/krogius2019iros.pdf`
   `markdown/krogius2019iros.md`
   讨论更灵活的标签布局，比如嵌套和非标准形状。

本项目的实现选择：

- 标签家族：`tag36h11`
- 检测后端：`OpenCV cv2.aruco`

原因：

- Python 调用最直接
- OpenCV 内建 AprilTag 字典，环境更简单
- 足够满足比赛中的二维定位与距离估计
