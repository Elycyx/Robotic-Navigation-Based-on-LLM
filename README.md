

# Trajectron++: Dynamically-Feasible Trajectory Forecasting With Heterogeneous Data
This repository contains the code for []() by Yuxuan Chen\*, Yixin Han\*, and Li Xiao (\* denotes equal contribution).

## Installation

### Cloning Method

```
git clone https://github.com/Elycyx/Robotic-Navigation-Based-on-LLM.git
```
### Ros2 Installation
Follow the [tutorial](https://docs.ros.org/en/rolling/index.html) to install Ros2

### Nav2 Installation
Our robot navigation controller is based on [Nav2](https://docs.nav2.org/getting_started/index.html). Before using method, we need to install Nav2.
```
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
sudo apt install ros-<ros2-distro>-turtlebot3-gazebo
```
### FastChat Installation
Install [FastChat](https://github.com/lm-sys/FastChat).
```
pip3 install "fschat[model_worker,webui]"
```

### Environment Setup

#### OpenAI API Key
Before we run `simulation.py`,`simulation_slm.py` and `simulation_single.py`, we should set OpenAI API Key to use OpenAI API.
```
gpt4 = ChatOpenAI(model='gpt-4-turbo', base_url='https://api.openai-proxy.com/v1', openai_api_key=<your_openai_api_key>, temperature=0.1)
```

#### Conda Environment
We'll create a conda environment to hold the dependencies.
```
conda create -n method python=3.10
conda activate method
pip install -r requirements.txt
```

#### Simulation Environment
Our simulation environment employs [AWS RoboMaker Hospital World ROS package](https://github.com/aws-robotics/aws-robomaker-hospital-world) through Gazebo.

We'll first donwload `models` from [here](https://github.com/aws-robotics/aws-robomaker-hospital-world) and put it under `hospital` folder.

## Running
### Running Simulator
To run our simulator based on Gazebo and Nav2, according to the [tutorial](https://docs.nav2.org/getting_started/index.html), we should set key environment variables first.
```
source /opt/ros/<ros2-distro>/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models:hospital/models
# The path to the models we just downloaded.
```

Then, in the same terminal, run:
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False map:=/hospital/hospital.yaml world:=/hospital/hospital_with_waffle.world
```
This launch file will launch Nav2 with the AMCL localizer in the simulation environment. It will also launch the robot state publisher to provide transforms, a Gazebo instance with the Turtlebot3 URDF, and RVIZ.

### Running FastChat
FastChat is used to provide LLM inference interfaces through API. We need 3 terminals to run these commands:
```
python3 -m fastchat.serve.controller  # launch the controller
python3 -m fastchat.serve.model_worker --model-path <model's path on Huggingface>  # launch the model worker(s)
python3 -m fastchat.serve.openai_api_server --host localhost --port 8000  # launch the RESTful API server
```
Now, the local LLM can execute queries through OpenAI API with variables set as below:
```
openai.api_key = "EMPTY"
openai.base_url = "http://localhost:8000/v1/"
```

### Running Method
#### Method with Teacher-student Iteration
```
python simulation.py --model <model_name>
```
The <model_name> here is different from that when running FastChat. For example, in FastChat, it is `google/gemma-2b`, while here, it is `gemma-2b`, just the model name without usenames.
#### Method with Memory
```
python simulation_slm.py --model <model_name>
```
This runs SLMs with the memory during iterations, so we need to generate a feedback file first through `simulation.py`.
#### Running with Single Models
```
python simulation_single.py --model <model_name>
```
This runs a single model like GPT-4 to finish the navigation tasks.

## Fine-tuning


## Citation

If you find our work useful, please cite:

```bibtex
@inproceedings{
} 
```