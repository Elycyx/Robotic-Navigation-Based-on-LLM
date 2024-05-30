from langchain_community.chat_models import ChatOpenAI
from generate_prompt import generate_prompt, prompter
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
    '''Save results to file'''
    with open('memory.txt', 'a') as f:
        f.write(f'task{i}: '+ output + '\n')

def next_available_filename(base_filename, extension, directory='.'):
    '''Generate usable result file paths'''
    i = 1
    # Construct the full file path
    while os.path.exists(os.path.join(directory, f'{base_filename}{i}{extension}')):
        i += 1
    # Returns the name of a non-existent file
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
    '''Read the relevant data'''
    with open(tasks_path, 'r') as file:
        tasks_info = json.load(file)

    with open(prompt_path, 'r') as f:
        pr = str(f.read())
    return tasks_info, pr


def memory(output, character):
    '''Save results to file'''
    with open('memory.txt', 'a') as f:
        f.write(f'{character}: '+ output + '\n')

def output2csv(results, headers, result_path):
    '''Exporting navigation information to a csv file'''
    with open(result_path, 'w', newline='') as file:
    # Save the results and output to a csv file
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(results)

def llm_query(task_description, i):
    '''Execute a LLM query and return relevant information'''
    save_to_file(task_description, i)
    print(f"Task {i}: {task_description}")
    feedback = read(i)
    messages = prompter(task_description, feedback)
    output1 = gpt4.invoke(messages).content
    memory(output1, 'Prompt Engineer')
    print('\033[32m' + output1 + '\033[0m')
    input = generate_prompt(task_description, output1)
    output2 = local_llm.invoke(input).content
    memory(output2, 'Robot Motion Controller')
    print('\033[33m' + output2 + '\033[0m')
    result = json.loads(output2)
    return result

def navigate(task_description, target_position, i):
    '''Perform navigation tasks and return relevant information'''
    print(task_description)
    success = 1
    total_distance = 0 
    nav_error = 0
    time_spent = 0 
    start_time = time.time() # Record start time
    nav_start_time = start_time
    save_to_file(task_description, i)
    try:
        result = llm_query(task_description, i) # Execute LLM query
        print(result)
        num = len(result['positions']) # Get the number of navigation points
        if num != len(target_position):
            feedback_list.append({'result': result, 'success': 0, 'error_cause': 'The number of goal points is incorrect.'})
            save_to_file('Failed! The number of goal points is incorrect.', i)
            return {'success': 0,'navigation_error': 0,'time': 0,'nav_time': 0}
        nav_start_time = time.time() # Record navigation start time
        for j in range(num):
            target = target_position[j] # Get target location
            nav2client.nav2(result['positions'][j])
            x_final, y_final = get_current_position() # Get current position
            current_position = [x_final, y_final] # Record current position
            error = calculate_navigation_error(current_position, target) # Calculating navigation errors
            nav_error += error
            if error > 3:
                # If the navigation error is greater than 3, the navigation is considered to have failed
                success = 0
            time.sleep(1) # Wait 1s
        end_time = time.time() # Record end time
        nav_error /= num # Calculate the average navigation error
    except Exception as e:
        result = 'Failed! Unknown error.'
        print(f'error:{e}')
        end_time = time.time()
        feedback_list.append({'result': result, 'success': 0, 'error_cause': 'Unknown.'})
        save_to_file('Failed! Unknown error.', i)
        return {'success': 0,'navigation_error': 0,'time': 0,'nav_time': 0}
    
    time_spent = end_time - start_time # Calculate total time
    nav_time = end_time - nav_start_time # Calculate navigation time   

    if success == 1:
        feedback_list.append({'result': result, 'success': success, 'error_cause': 'This task was successful.'})
        save_to_file('Success!', i)
    else:
        feedback_list.append({'result': result, 'success': success, 'error_cause': 'Some goal point locations are incorrect.'})
        save_to_file('Failed! Some goal point locations are incorrect.', i)
    return {
        'success': success,
        'navigation_error': nav_error,
        'time': time_spent,
        'nav_time': nav_time
    }

parser = argparse.ArgumentParser(description="Run the local LLM model with specified parameters.")
parser.add_argument('--model', type=str, required=True, help='Model name to use with FastChat.')

# Parsing command line arguments
args = parser.parse_args()


gpt4 = ChatOpenAI(model='gpt-4-turbo', base_url='https://api.openai-proxy.com/v1', openai_api_key='', temperature=0.1) # Loading GPT-4
local_llm = ChatOpenAI(model=args.model, base_url='http://localhost:8000/v1', openai_api_key='EMPTY', temperature=0.1, max_tokens=200) # Loading fastchat


# Defining Initialization Parameters

global feedback_list, nav2client
rclpy.init() # Initializing ros2
nav2client = Nav2Client()
feedback_list = []
prompt_path = 'prompt_en.txt' # Path to the prompt file
tasks_path = 'tasks30.json' # Task file path
result_path = next_available_filename('result', '.csv', 'results')


def main():
    results = []
    total_error = 0 
    total_time = 0
    total_MTR = 0 
    headers = ['number of the task', 'NE', 'SR','time', 'MTR'] # csv file table header
    S = 0 # Global successes
    tasks_info, pr = data_reader(tasks_path, prompt_path) # Reading tasks and prompt files
    for i, task in enumerate(tasks_info["tasks"], start=1):
        # Iterate over all tasks, perform navigation tasks and return relevant information
        navigation_info = navigate(task["description"], task["goal"], i)
        S += navigation_info['success']
        print(navigation_info) # Printing navigation information
        if navigation_info['time'] != 0:
            results.append([i, navigation_info['navigation_error'], navigation_info['success'], navigation_info['time'], navigation_info['nav_time'] / navigation_info['time']]) # 存储导航信息
        else:
            results.append([i, navigation_info['navigation_error'], navigation_info['success'], navigation_info['time'], 0])
        if navigation_info['success'] == 1:
            # If the navigation is successful, calculate the overall result
            total_error += navigation_info['navigation_error']
            total_time += navigation_info['time']
            total_MTR += navigation_info['nav_time'] / navigation_info['time']
        output2csv(results, headers, result_path) # Exporting navigation information to a csv file
        time.sleep(3) # Wait 5s

    rclpy.shutdown() # Close ros2
    nav2client.destroy_node() # Close ros2 node
    with open('feedback/fb.json', 'w') as file:
        json.dump(feedback_list, file)

    results.append(['overall', total_error / S, S / tasks_info["num"], total_time / S, total_MTR / S]) # Calculate and store overall results
    output2csv(results, headers, result_path) # Exporting navigation information to a csv file


if __name__ == "__main__":
    main()