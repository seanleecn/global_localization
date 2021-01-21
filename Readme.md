# 基于STDR仿真器的全局定位研究

# amcl定位实验

1. 打开顶点巡逻
roslaunch src/stdr_navigation/launch/diy_map_patrol_nav.launch

2. 录制数据
rosbag record -O pose /amcl_pose /tf

3. 保存成txt
rostopic echo -b pose.bag -p /amcl_pose > amcl.txt
rostopic echo -b pose.bag -p /tf > tf.txt

4. MATLAB代码
```
clc;clear;
fout=fopen('tf_new.txt','wt');      %新建一个txt文件
fin = ['tf.txt'];                 %要读取的文档所在的路径
fpn = fopen (fin, 'rt');           %打开文档
while feof(fpn) ~= 1                %用于判断文件指针p在其所指的文件中的位置，如果到文件末，函数返回1，否则返回0
    line = fgetl(fpn);            
    if contains(line,'map_static,robot0')
        new_str=line;                 
        fprintf(fout,'%s\n',new_str);%新的字符串写入当新建的txt文档中
    end
    
end
fclose(fout);
```

# 基础命令

1. 启动仿真环境:
   
   roslaunch stdr_diymap diy_map_robot_gui.launch 
2. 发布/amcl/map话题给map_server:
   
   roslaunch stdr_diymap load_diy_map.launch
3. AMCL定位:
   
   roslaunch stdr_amcl diy_map_amcl.launch
4. move_base导航:
   
   roslaunch stdr_move_base stdr_move_base.launch
   
   注意，这里加载的全局地图是map_server中的/amcl/map，在launch中制定
   
5. 全局定位:
   
   roslaunch gki_global_localization global_localize_once.launch