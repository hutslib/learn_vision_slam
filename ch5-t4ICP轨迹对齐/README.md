# 真实轨迹与估计轨迹误差计算与可视化
<img src="result_img/ICP.png" alt="Tent-PCA">
蓝色为真实轨迹 红色为估计轨迹
由于 ground-truth 轨迹与相机轨迹很可能不在一个参考系中,它们得到的轨迹并不能直接比较。这时,我们可以用 ICP 来计算两条轨迹之间的相对旋转与平移,从而估计出两个参考系之间的差异。
