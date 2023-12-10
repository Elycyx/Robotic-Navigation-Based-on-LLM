from openai import OpenAI
import os
import json

client = OpenAI(api_key='')

with open('prompt1.txt', 'r') as f:
    pr = str(f.read())
    f.close()

response = client.chat.completions.create(
  model="gpt-4-1106-preview",
  messages=[
    {"role": "system", "content": pr},
    {"role": "user", "content": "把客厅里的脏盘子送去洗碗池，然后把门口的毛巾给正在洗澡的姐姐送去"},
  ],
  # response_format={"type": "json_object"}
)
result = str(response.choices[0].message.content)
print(result)

a = os.popen(result)
# a = os.popen("ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: " + str(x[0]) + ", y: " + str(y[0]) + ", z: 0.0}, orientation: {w: 1.0}}}'") # 使用a接收返回值

print(a.read()) # 读取输出内容
