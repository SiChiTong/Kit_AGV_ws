sudo ip link set can0 up type can bitrate 125000
cd ~/khongminh/Kit_agv_ws

sheet1 - dieu khien chuong trinh chinh
source devel/setup.bash   // dang ky 
roslaunch agv_main agvRun.launch 

sheet 2 - dieu khien chuong trinh dieu khien bang ban phim
source devel/setup.bash   // dang ky 
roslaunch kit_agv_teleop kit_agv_teleop_key.launch

sheet 3 - dieu khien chuong trinh chay thuat toan do do bam duong
$source devel/setup.bash  
$rqt_plot

1. Open new sheet
2. Diretory: $cd ~/khongminh/Kit_agv_ws
3. Source: $source devel/setup.bash
4. Diretory: $cd ~/khongminh/Kit_agv_ws/src/agv_main/scripts
5. Run: $python exportCsv.py

