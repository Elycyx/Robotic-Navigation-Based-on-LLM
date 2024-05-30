from langchain_community.chat_models import ChatOpenAI
from generate_prompt import generate_prompt, prompter, gpt_message, individual_prompt
from reader_files import read
import json
import time
import csv
import os
import math
import rclpy
from tf_reader import get_current_position
import time
import csv
from pos_pub import Nav2Client
import argparse

def save_to_file(output, i):
    '''保存结果到文件'''
    with open('memory.txt', 'a') as f:
        f.write(f'task{i}: '+ output + '\n')

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


def memory(output, character):
    '''保存结果到文件'''
    with open('memory.txt', 'a') as f:
        f.write(f'{character}: '+ output + '\n')

def output2csv(results, headers, result_path):
    '''将导航信息输出到csv文件'''
    with open(result_path, 'w', newline='') as file:
    # 保存结果，输出到csv文件
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(results)

def llm_query(task_description, i):
    '''执行fastchat查询并返回相关信息'''
    save_to_file(task_description, i)
    print(f"Task {i}: {task_description}")
    feedback = read(i)
    messages = individual_prompt(task_description, feedback)
    output1 = local_llm.invoke(messages).content
    print(output1)
    result = json.loads(output1)
    return result

def navigate(task_description, target_position, i):
    '''执行导航任务并返回相关信息'''
    print(task_description)
    # rclpy.init() # 初始化ros2
    success = 1
    total_distance = 0 
    nav_error = 0
    time_spent = 0 
    start_time = time.time() # 记录开始时间
    nav_start_time = start_time
    save_to_file(task_description, i)
    try:
        result = llm_query(task_description, i) # 执行crewai查询
        # print(result)
        num = len(result['positions']) # 获取导航点个数
        if num != len(target_position):
            # stop_trajectory_length_calculator()
            feedback_list.append({'result': result, 'success': 0, 'error_cause': 'The number of goal points is incorrect.'})
            save_to_file('Failed! The number of goal points is incorrect.', i)
            return {'success': 0,'navigation_error': 0,'time': 0,'nav_time': 0}
        nav_start_time = time.time() # 记录导航开始时间
        for j in range(num):
            target = target_position[j] # 获取目标位置
            nav2client.nav2(result['positions'][j])
            x_final, y_final = get_current_position() # 获取当前位置
            current_position = [x_final, y_final] # 记录当前位置
            error = calculate_navigation_error(current_position, target) # 计算导航误差
            nav_error += error
            if error > 3:
                # 如果导航误差大于3，认为导航失败
                success = 0
            time.sleep(1) # 等待1s
        end_time = time.time() # 记录结束时间
        nav_error /= num # 计算平均导航误差
    except Exception as e:
        result = 'Failed! Unknown error.'
        print(f'error:{e}')
        end_time = time.time()
        feedback_list.append({'result': result, 'success': 0, 'error_cause': 'Unknown.'})
        save_to_file('Failed! Unknown error.', i)
        return {'success': 0,'navigation_error': 0,'time': 0,'nav_time': 0}
    
    time_spent = end_time - start_time # 计算总时间
    nav_time = end_time - nav_start_time # 计算导航时间   

    if success == 1:
        feedback_list.append({'result': result, 'success': success, 'error_cause': 'This task was successful.'})
        save_to_file('Success!', i)
    else:
        feedback_list.append({'result': result, 'success': success, 'error_cause': 'Some goal point locations are incorrect.'})
        save_to_file('Failed! Some goal point locations are incorrect.', i)
    # print(feedback_list)
    return {
        'success': success,
        'navigation_error': nav_error,
        'time': time_spent,
        'nav_time': nav_time
    }

parser = argparse.ArgumentParser(description="Run the local LLM model with specified parameters.")
parser.add_argument('--model', type=str, required=True, help='Model name to use with FastChat.')

# 解析命令行参数
args = parser.parse_args()


gpt4 = ChatOpenAI(model='gpt-4-turbo', base_url='https://api.openai-proxy.com/v1', openai_api_key='', temperature=0.1) # Loading GPT-4
local_llm = ChatOpenAI(model=args.model, base_url='http://localhost:8000/v1', openai_api_key='EMPTY', temperature=0.1, max_tokens=200) # Loading fastchat



# 定义初始化参数

global feedback_list, nav2client
rclpy.init() # 初始化ros2
nav2client = Nav2Client()
feedback_list = []
prompt_path = 'prompt_en.txt' # prompt文件路径
tasks_path = 'tasks30.json' # 任务文件路径·
result_path = next_available_filename('result', '.csv', 'results')


def main():
    results = []
    total_error = 0 # 总误差
    total_time = 0 # 总时间
    total_MTR = 0 # 总移动用时比
    headers = ['number of the task', 'NE', 'SR','time', 'MTR'] # csv文件表头
    S = 0 # 全局成功次数
    tasks_info, pr = data_reader(tasks_path, prompt_path) # 读取任务和prompt文件
    for i, task in enumerate(tasks_info["tasks"], start=1):
        # 遍历所有任务，执行导航任务并返回相关信息
        navigation_info = navigate(task["description"], task["goal"], i)
        S += navigation_info['success']
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
        output2csv(results, headers, result_path) # 将导航信息输出到csv文件
        time.sleep(3) # 等待5s

    rclpy.shutdown() # 关闭ros2
    nav2client.destroy_node() # 关闭节点
    '''with open('feedback/fb.json', 'w') as file:
        json.dump(feedback_list, file)'''

    results.append(['overall', total_error / S, S / tasks_info["num"], total_time / S, total_MTR / S]) # 计算并存储总体结果
    output2csv(results, headers, result_path) # 将导航信息输出到csv文件


if __name__ == "__main__":
    main()