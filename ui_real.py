import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import os
import subprocess
import itertools

DEBUG = False

root = tk.Tk()
root.title("导航")
root.geometry("400x450") 

try:
    options = sorted(list({f.split(".")[0] for f in os.listdir("install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/reality/")}))
except FileNotFoundError:
    options = ["default_map"]

selected_option = tk.StringVar(value=options[0])

run_button = tk.Button(root, text="运行", width=8)
mapping_button = tk.Button(root, text="建图", width=8)
save_button = tk.Button(root, text="保存", width=8)
filename_input = tk.Entry(root, width=10)

navigate_button = tk.Button(root, text="导航", width=8)
navigate_pos_input = tk.Entry(root, width=10)

hp_button = tk.Button(root, text="发送血量", width=8)
hp_input = tk.Entry(root, width=10)
# hp_input.insert(0, "100")


world = ttk.Combobox(root, textvariable=selected_option, values=options, state="readonly", width=10)

run_button.grid(row=0, column=0, padx=5, pady=10)
world.grid(row=0, column=1, padx=5, pady=10)

mapping_button.grid(row=1, column=0, padx=5, pady=10)

save_button.grid(row=2, column=0, padx=5, pady=10)
filename_input.grid(row=2, column=1, padx=5, pady=10)

navigate_button.grid(row=3, column=0, padx=5, pady=10)
navigate_pos_input.grid(row=3, column=1, padx=5, pady=10)

hp_button.grid(row=4, column=0, padx=5, pady=10)
hp_input.grid(row=4, column=1, padx=5, pady=10)


def on_navigate():
    def split(str):
        return list(itertools.chain(*[i.split(",") for i in str.split(" ")]))

    x, y = split(navigate_pos_input.get().strip())
    os.system('ros2 topic pub --once /red_standard_robot1/goal_pose_raw geometry_msgs/msg/PoseStamped "{header: {frame_id: \'map\'}, pose: {position: {x: ' + str(x) + ', y: ' + str(y) + ', z: 0.0}, orientation: {w: 1.0}}}"')
def on_run():
    selected_world = f"{selected_option.get()}"
    os.system(f"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:={selected_world} \
slam:=False \
use_robot_state_pub:=True")

def on_mapping():
    os.system("ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True &")

def on_save():
    filename = filename_input.get().strip()
    if not filename:
        messagebox.showerror("错误", "请输入文件名")
        return
    if filename in options:
        messagebox.showerror("错误", "文件已存在，请更换文件名")
        return
    os.system(f"ros2 run nav2_map_server map_saver_cli -f {filename} --ros-args -r __ns:=/red_standard_robot1")
    os.system(f"mv {filename}.pgm src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/{filename}.pgm")
    os.system(f"mv {filename}.yaml src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/{filename}.yaml")
    os.system(f"cp src/pb2025_sentry_nav/point_lio/PCD/scans.pcd src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/{filename}.pcd")
    messagebox.showinfo("成功", "地图保存成功")

def on_publish_hp():
    hp_value = hp_input.get().strip()
    
    if not hp_value.isdigit() and not (hp_value.startswith('-') and hp_value[1:].isdigit()):
        messagebox.showerror("错误", "请输入有效的数字血量！")
        return

    topic_name = "/robot_hp" 
    cmd = f'ros2 topic pub --once {topic_name} std_msgs/msg/Int32 "{{data: {hp_value}}}"'
    
    os.system(cmd)


run_button.config(command=on_run)
mapping_button.config(command=on_mapping)
save_button.config(command=on_save)
navigate_button.config(command=on_navigate)
hp_button.config(command=on_publish_hp)

root.mainloop()