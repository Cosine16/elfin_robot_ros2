# STATION 分支说明（elfin5_station 工位维护分支）

本仓库是 [huayan-robotics/elfin_robot_ros2](https://github.com/huayan-robotics/elfin_robot_ros2) 的工位 fork，
由 [Cosine16/elfin5_ros2-foxy](https://github.com/Cosine16/elfin5_ros2-foxy)（elfin5_station 项目）以
git submodule 方式引入并维护。

## 分支结构

| 分支 | 基线 | 用途 |
|---|---|---|
| `humble-station` | humble_ethercat@1381e3f | **日常使用**：Humble 基线 + 工位补丁（主仓库 submodule 指向此分支） |
| `foxy-station` | foxy_ethercat@fddb7d0 | Foxy 线存档：同基线 + 同套工位补丁（升级回退参照） |
| upstream `humble_ethercat` / `foxy_ethercat` / `foxy_485` | — | 官方分支（`git fetch upstream` 同步） |

上游基线说明：humble_ethercat 在内容上是 foxy_ethercat 的超集
（cartPathGoalCB 修复两分支 patch-id 一致），切换到 humble 基线不丢失 foxy 线修复。

## 工位补丁清单（相对上游基线）

1. `fix(soem)` — oshw.c strncpy→memcpy：GCC 11+ `-Werror=stringop-overflow` 构建失败
2. `fix(launch)` — MoveIt2 kinematics 改 `robot_description_kinematics` 命名空间传入；
   移除冗余独立 `ros2_control_node`（gazebo_ros2_control 下 pluginlib 基类不匹配 SIGABRT）
3. `feat(bringup)` — `elfin_drivers.local.yaml` 深合并逐机覆盖机制（网卡名等差异不进库）
4. `fix(driver)` — error_log 格式串拼接错误修复
5. `feat(station)` — E05-1195_Pro 工位适配：EtherCAT 从站拓扑（驱动 {2,3,4} + IO 5）、
   count_zeros 出厂标定、Elfin5 前两轴 axis_torque_factors

## 与上游同步流程（上游更新频率约年 1–2 次）

```sh
git fetch upstream
git checkout humble-station
git merge upstream/humble_ethercat   # 冲突在本仓库内隔离解决
git push origin humble-station

# 然后在主仓库前移 submodule 指针：
cd elfin_ws/src/elfin_robot && git pull
cd ../../.. && git add elfin_ws/src/elfin_robot && git commit -m "chore: 前移 elfin_robot submodule"
```

## 注意事项

- 机器标定（count_zeros、扭矩系数等）提交在分支内；网卡名等逐机差异走
  `elfin_robot_bringup/config/elfin_drivers.local.yaml`（本仓库 .gitignore 已忽略）。
- humble 基线的 `update_rate: 250`（controller_manager）为上游默认，工位 Foxy 时期为 50，
  **实机验证后如周期不稳可回调**。
- `elfin_ros_control` 的 CMakeLists 含 `../../../install|build` 相对路径 hack，
  必须与 `elfin_ethercat_driver` 同工作区构建，勿单独挪动。
