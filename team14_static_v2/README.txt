Team 14 · MEAM 5200 Final Project · 运行说明

当前红队仿真已夹起 cube4_static。保持 Gazebo 和 final.launch 运行。
在虚拟机终端执行下面四行，程序提示 START 后按 Enter。

source ~/meam520_ws/devel_isolated/setup.bash
bash /mnt/hgfs/Team14Final/install.sh
cd ~/meam520_ws/src/meam520_labs/labs/final/team14_static_v2
python3 final.py match-held

如果 /mnt/hgfs/Team14Final 不存在，可在新的终端建立当前用户可读的挂载点。

mkdir -p "$HOME/team14_vm_share"
vmhgfs-fuse .host:/Team14Final "$HOME/team14_vm_share" -o ro
bash "$HOME/team14_vm_share/install.sh"

也可把 U 盘中的 Team14_Final_Project.zip 拖进 Ubuntu 的 Downloads，然后执行：

stage=$(mktemp -d)
unzip "$HOME/Downloads/Team14_Final_Project.zip" -d "$stage"
bash "$stage/team14_static_v2/install.sh"

正式比赛入口
从解压出的 team14_static_v2 目录执行 bash install.sh --competition。
这会同时安装项目文件夹和课程 labs/final/final.py 入口，并备份已有入口。
启动课程环境、进入 labs/final 后运行 python3 final.py，按 Enter 开始一场新的比赛。
压缩包最外层的 final.py 是同一个课程入口，需要与 team14_static_v2 同级。

新开一场 Gazebo 仿真
终端一：source ~/meam520_ws/devel_isolated/setup.bash
        roslaunch meam520_labs final.launch team:=red
也可以指定 team:=blue。终端二 source 同一个 setup.bash，进入项目目录后执行：

python3 final.py match

match 从空夹爪开始。match-held 用于本次已夹起 cube4_static 的特殊场景。
只想测试当前方块能否放好，可以运行 python3 final.py place-held；这会改变场景。
其他单项入口包括 observe、pregrasp、pick、observe-goal、cycle、stack、stack-held 和 dynamic。
运行 observe 和 observe-goal 时机械臂会移动到相应观察位。

程序如何运行
默认时限为 180 ROS 秒。比赛先从己方静态台抓取上层方块、暴露的长方块和余下方块。
相机在释放后重新检查方块的位置，检查通过后再累计放置次数。
到达第一列的规划高度上限后，使用目标平台上相邻的一列。
动态阶段优先抓取转盘上六个正方块。它根据多帧相机位置估计转速、旋转方向和到达时刻，
在近侧拦截点再次观察并合拢夹爪。移动长方块没有列入本版动态策略。
实际抓放次数取决于场景、腕部相机视野、动作时间和夹持情况。
--seconds 允许设置 30 到 300 ROS 秒。正式轮次默认 180 秒，延长赛时长由现场裁判决定。
程序会在发送动作前检查剩余时间，已经送入课程接口的阻塞动作由控制器管理。

测试与真机
目前已有红队 Gazebo 上层方块抓取、抬升观测。全流程使用合成相机与夹爪模型做了离线检查。
安排一次完整 Gazebo 测试，重点看长方块是否沿短边夹紧、塔顶能否重新看到、动态闭爪是否及时。
真机按项目讲义先在实验室电脑演示仿真，由 TA 确认，并按要求至少两位队员在场。
使用课程 vision_pipeline.launch、franka.sh master 和 interface_final.launch team:=red/blue 流程。
在项目目录手动启动真机完整流程时执行 python3 final.py match --hardware。
课程入口会根据已启动的环境自动识别队色和设备。根据裁判信号与课程 Software Stop 结束比赛。

运行记录
终端会显示本次日志路径：~/meam520_project_runs/<本次运行编号>/。
events.jsonl 保存动作、实测关节、夹爪反馈、相机观测和 ROS/实际经过时间。
result.json 保存运行状态与结果。placed 是程序用视觉确认的放置数，比赛得分由裁判确定。
