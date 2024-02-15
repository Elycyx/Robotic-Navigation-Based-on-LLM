from openai import OpenAI
import rclpy
import math
import os
import json
from tf_reader import get_current_position
from trajectory_length_calculator import start_trajectory_length_calculator, stop_trajectory_length_calculator
import time
import csv
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from pos_pub import Nav2Client
import subprocess
import shlex


def next_available_filename(base_filename, extension, directory='.'):
    '''生成可用的结果文件路径'''
    i = 1
    # 构造完整的文件路径
    while os.path.exists(os.path.join(directory, f'{base_filename}{i}{extension}')):
        i += 1
    # 返回不存在的文件名
    return os.path.join(directory, f'{base_filename}{i}{extension}')

def calculate_navigation_error(current_position, target_position):
    """
    Calculate the Euclidean distance between the current position and the target position.
    
    Parameters:
    - current_position: A tuple (x, y, z) representing the current coordinates.
    - target_position: A tuple (x, y, z) representing the target coordinates.
    
    Returns:
    - The Euclidean distance as a float.
    """
    dx = target_position[0] - current_position[0]
    dy = target_position[1] - current_position[1]
    return math.sqrt(dx**2 + dy**2)

def data_reader(tasks_path, prompt_path):
    '''读取相关数据'''
    with open(tasks_path, 'r') as file:
        # 读取json文件
        tasks_info = json.load(file)

    with open(prompt_path, 'r') as f:
        # 读取prompt文件
        pr = str(f.read())
    return tasks_info, pr

def llm_query(task_description):
    '''执行LLM查询并返回相关信息'''
    response = client.chat.completions.create(
            # model="gpt-4-turbo-preview",
            model='gpt-3.5-turbo',
            # model = 'ft:gpt-3.5-turbo-1106:melx::8k5nE521',
            messages=[
                {"role": "system", "content": pr},
                {"role": "user", "content": str(task_description)},
            ],
            # response_format={"type": "json_object"}
        )
    result = json.loads(response.choices[0].message.content)
    print(result)
    return result

def generate_codes(position):
    '''生成导航代码'''
    try:
        code = 'ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: ''{header: {frame_id: \'map\'}, pose: {position: {x: ' + str(position[0]) +', y: ' + str(position[1]) + ', z: 0.0}, orientation: {w: 1.0}}}}"'
    except Exception as e:
        print(f'error:{e}')
        code = ''
    return code

def output2csv(results, headers, result_path):
    '''将导航信息输出到csv文件'''
    with open(result_path, 'w', newline='') as file:
    # 保存结果，输出到csv文件
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(results)

def run_nav2(code):
    '''执行导航代码，等待机器人完成导航'''
    '''process_output = os.popen(code) # 执行导航代码
    command_output = process_output.read() # 读取导航代码执行结果
    print(command_output) '''
    try:
        args = shlex.split(code)
        # 使用subprocess.Popen而不是subprocess.run来手动控制进程
        with subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            try:
                # 等待进程完成或超时
                stdout, stderr = process.communicate(timeout=420)
                print(stdout)
                if process.returncode != 0:
                    print(f"Command failed with error: {stderr}")
            except subprocess.TimeoutExpired:
                print("Command timed out")
                process.kill()  # 杀死子进程
                stdout, stderr = process.communicate()  # 获取进程输出
                print(stderr)
                return  # 退出函数
    except Exception as e:
        print(f"An error occurred: {e}")
        return  # 退出函数

def navigate(task_description, target_position):
    '''执行导航任务并返回相关信息'''
    print(task_description)
    # rclpy.init() # 初始化ros2
    success = 1
    total_distance = 0 
    nav_error = 0
    time_spent = 0 
    start_time = time.time() # 记录开始时间
    nav_start_time = start_time
    try:
        result = llm_query(task_description) # 执行LLM查询
        num = len(result['positions']) # 获取导航点个数
        if num != len(target_position):
            # rclpy.shutdown() # 关闭ros2
            return {'success': 0,'total_distance': 0,'navigation_error': 0,'time': 0,'nav_time': 0}
        # start_trajectory_length_calculator() # 开始计算轨迹长度
        nav_start_time = time.time() # 记录导航开始时间
        for i in range(num):
            # code = generate_codes(result['positions'][i]) # 生成导航代码
            target = target_position[i] # 获取目标位置
            # print(f'target: {target}')
            # run_nav2(code) # 执行导航代码
            nav2client.nav2(result['positions'][i])
            x_final, y_final = get_current_position() # 获取当前位置
            current_position = [x_final, y_final] # 记录当前位置
            # print(f'current_position: {current_position}')
            error = calculate_navigation_error(current_position, target) # 计算导航误差
            # print(f'error: {error}')
            nav_error += error
            if error > 3:
                # 如果导航误差大于3，认为导航失败
                success = 0
            time.sleep(1) # 等待1s
        end_time = time.time() # 记录结束时间
        # print(f'end_time: {end_time}')
        # total_distance = stop_trajectory_length_calculator() # 停止计算轨迹长度
        # print(f'total_distance: {total_distance}')
        nav_error /= num # 计算平均导航误差
    except Exception as e:
        print(f'error:{e}')
        end_time = time.time()
        # total_distance = stop_trajectory_length_calculator() # 停止计算轨迹长度

    time_spent = end_time - start_time # 计算总时间
    nav_time = end_time - nav_start_time # 计算导航时间   
    # print(f'final_position: {final_position}')
    # rclpy.shutdown() # 关闭ros2
    return {
        'success': success,
        # 'total_distance': total_distance,
        'navigation_error': nav_error,
        'time': time_spent,
        'nav_time': nav_time
    }



# 定义初始化参数

rclpy.init() # 初始化ros2
client = OpenAI(api_key='') # openai api key
global success, nav2client
nav2client = Nav2Client()
success = 0 # 全局成功次数
prompt_path = 'prompt_en.txt' # prompt文件路径
tasks_path = 'tasks.json' # 任务文件路径·
result_path = next_available_filename('result', '.csv', 'results')
results = []
total_length = 0 # 总距离
total_error = 0 # 总误差
total_time = 0 # 总时间
total_MTR = 0 # 总移动用时比
headers = ['number of the task', 'NE', 'SR','time', 'MTR'] # csv文件表头


tasks_info, pr = data_reader(tasks_path, prompt_path) # 读取任务和prompt文件
for i, task in enumerate(tasks_info["tasks"], start=1):
    # 遍历所有任务，执行导航任务并返回相关信息
    navigation_info = navigate(task["description"], task["goal"])
    success += navigation_info['success']
    print(navigation_info) # 打印导航信息
    if navigation_info['time'] != 0:
        results.append([i, navigation_info['navigation_error'], navigation_info['success'], navigation_info['time'], navigation_info['nav_time'] / navigation_info['time']]) # 存储导航信息
    else:
        results.append([i, navigation_info['navigation_error'], navigation_info['success'], navigation_info['time'], 0])
    if navigation_info['success'] == 1:
        # 如果导航成功，计算总体结果
        # total_length += navigation_info['total_distance']
        total_error += navigation_info['navigation_error']
        total_time += navigation_info['time']
        total_MTR += navigation_info['nav_time'] / navigation_info['time']
    time.sleep(5) # 等待5s
nav2client.destroy_node()
rclpy.shutdown() # 关闭ros2

results.append(['overall', total_error / success, success / tasks_info["num"], total_time / success, total_MTR / success]) # 计算并存储总体结果
output2csv(results, headers, result_path) # 将导航信息输出到csv文件

  

