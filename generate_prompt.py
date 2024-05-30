from textwrap import dedent

def generate_prompt(task, prompt):
    output = dedent(f"""
Here is the very useful prompt provided by the expert to help you better complete this task. Please study it carefully and accept the prompt.
{prompt}
<|im_start|>system
I now need your assistance in controlling the movement of a robot, serving as my robotic assistant. I will provide you with my instructions, and then you will respond with the corresponding output in JSON format. The format should be:

{{
    "explanation": "A step-by-step logical analysis of the task, from which other content is generated",
    "positions": [[x1,y1],[x2,y2]]
}}
<|im_end|>
<|im_start|>user
{task}<|im_end|>
<|im_start|>assistant
""")
    return output

def individual_prompt(task,feedback):
    output = dedent(f"""
These are the feedback information from last cycle of tasks by you. I hope you can learn from it and improve your performance. Try to correct the mistakes and provide the correct output for guiding the robot's movements.
{feedback}
<|im_start|>system
I now need your assistance in controlling the movement of a robot, serving as my robotic assistant. I will provide you with my instructions, and then you will respond with the corresponding output in JSON format. The format should be:

{{
    "explanation": "A step-by-step logical analysis of the task, from which other content is generated",
    "positions": [[x1,y1],[x2,y2]]
}}
<|im_end|>
<|im_start|>user
{task}<|im_end|>
<|im_start|>assistant
""")
    return output

def prompter(task,feedback):
    description=dedent(f"""You are a prompt engineer responsible for a robot's intelligent navigation project. 
				You are highly skilled in robot indoor navigation tasks and prompt engineering for LLM. 
				With your utmost dedication, you aim to read the feedback and design a perfect prompt 
				that enables LLM on the robot to provide the correct output for guiding the robot's movements.You will create a prompt for llm to generate the correct output for guiding the robot's movements.
			Please help LLM understand the requirements of the input task correctly and break down the task clearly, so that the llm knows the positions to head for.
			The llm only needs to provide the target points to visit in sequential order and doesn't need to worry about how to get there.
            Help the language model analyze the task quantitatively and specifically so that it can generate the correct output.
			Here is the spatial information of the environment:
				The starting point is the current location of the robot. There is no need to consider where to start from.
				------------
				Pantry 1: (-8.08, 14.2)  Storage for various medicines in the hospital, you need to come here to pick up medications.
				Pantry 2: (8.43, 14.8)  Stores various medical supplies, such as sterilized scalpels, disposable gloves, disposable masks, etc.
				Front desk: (1.92, 7.66)  The front desk is where service personnel are located. You can inquire about hospital-related matters here. Additionally, the robot's charging station is also located here. The robot can come here to recharge when its battery is low.
				Staircase 1: (-3.77, 17.3)
				Staircase 2: (3.97, 17.5)  The stairwells and elevator of Building 1 and Building 2 are the places for going up and down stairs, and they are the necessary passage for people to enter this floor. Food delivery and packages are usually delivered to this location.
				Elevator: (0.139, 14.2)
				Beverage vending machine: (-7.03, 10.2)  You can purchase beverages at the beverage vending machine location.
				Food vending machine: (7.35, 10.2)  You can purchase food at the food vending machine location.
				Room 1: (-9.6, 10.7)  Room 1 is the pediatric examination room, where children usually go for medical treatment.
				Room 2: (-9.68, 6.02)  Room 2 is the internal medicine examination room, where patients usually go for internal medicine-related illnesses, such as heart and appendix-related diseases.
				Room 3: (10, 5.7)  Room 3 is the surgical examination room, where patients usually go for surgical-related illnesses such as fractures and sprains.
				Room 4: (9.77, 10.9)  Room 4 is the traditional Chinese medicine department, where treatments related to traditional Chinese medicine are usually conducted. Acupuncture and herbal medicine are available here.
				Lounge: (-8.83, -26.1)  The lounge is where doctors take breaks, and it is also where entertainment facilities such as sofas and televisions are located.
				Operating Room 1: (-2.94, -16.4)
				Operating Room 2: (1.59, -17.3)  Surgery Room 1 and Surgery Room 2 are where doctors perform surgeries. Currently, Surgery Room 1 is in operation.
				Ward 1: (-8.2, -6.68)  In Ward 1, there are beds numbered 1, 2, and 3.
				Ward 2: (-8.62, -18.9)  Ward 2 is an isolation ward primarily used for isolating patients infected with the novel coronavirus. The ward has beds numbered 4, 5, and 6.
				Ward 3: (2.67, -26.7)  In Ward 3, there are beds numbered 7, 8, and 9.
				Ward 4: (7.28, -17.6)  In Ward 4, there are beds numbered 10, 11, 12, and 13.
				Ward 5: (7.36, -6.98)  In Ward 5, there are beds numbered 14, 15, and 16.
				Bed 1: (-8.83, 0.924)  There is an elderly grandmother who just had heart surgery lying on bed number 1.
				Bed 2: (-9.13, -3.03)  There is a Black individual who just had their appendix removed lying on bed number 2.
				Bed 3: (-8.32, -6.22)
				Bed 4: (-9.23, -12.4)  There is an elderly person infected with the novel coronavirus lying on bed number 4.
				Bed 5: (-9.56, -15.6)  There is a Black individual infected with the novel coronavirus lying on bed number 5.
				Bed 6: (-9.35, -19.5)
				Bed 7: (0.429, -27.2)
				Bed 8: (4.37, -27.2)
				Bed 9: (7.54, -27.5)
				Bed 10: (8.03, -26.4)
				Bed 11: (9, -21.7)
				Bed 12: (8.93, -17.6)
				Bed 13: (8.9, -14)
				Bed 14: (9.14, -7.06)  There is a man with a leg fracture lying on bed number 14. All other unmentioned beds are empty.
				Bed 15: (8, -4.02)
				Bed 16: (8.26, -1.1)
				Men's restroom: (-1.66, -11)
				Women's restroom: (1.52, -11)
				Ward 1 restroom: (-6.99, 0.906)
				Ward 2 restroom: (-7.45, -12.4)
				Ward 4 restroom: (6.59, -13)
				Ward 5 restroom: (6.61, 0.432)
				Dean's office: (1.16, -4.2)  The hospital director's office is where you can find the hospital director.
				Vice Dean's office: (-1.13, -3.71)  The vice director's office is where you can find the vice director of the hospital.
				------------
			Standard output format:
				{{
					"explanation": "Logical analysis step by step for the task, based on which other content is generated.",
					"positions": "Coordinates of the target points to be visited in order, stored as a list.",
				}}
								 
			Here is the task you need to create a prompt for:
			------------
    		{task}
			------------
			LLM is already aware of the basic information mentioned above, so you may primarily focus on helping it analyze the task.
			Here is the output provided by the language model during the last iteration and the flag indicating whether it was successful.
            {feedback}
            Success=1 means the task was successful, Success=0 means the task failed.If there is no information, it means this is the first iteration and no previous iterations have been done.
			Adjust your prompt based on this feedback. 
			For those failed tasks, analyse the mistakes, help the LLM avoid mistakes and improve its performance.
			!!!For those successful tasks, you can directly tell the LLM the correct positions(RIGHT ANSWER).
			Your Final answer must be the prompt for the llm, only the prompt and nothing else.
			""")
    return description


def gpt_message(task):
    description=dedent(f"""user: Now I need your help to control the movement of a robot as my robot assistant. I will provide you with instructions, and you will respond with the corresponding JSON-formatted output. The format is as follows:
			{{
				"explanation": "Logical analysis step by step for the task, based on which other content is generated.",
				"positions": "Coordinates of the target points to be visited in order, stored as a list."
			}}
			For example:
			{{
				"explanation": "Go to the specified room.",
				"positions": [[x1,y1], [x2,y2]]
			}}
			Now, based on the known positions and information of some areas on this map (indicated by triple quotes):
			'''
			Pantry 1: (-8.08, 14.2)  Storage for various medicines in the hospital, you need to come here to pick up medications.
			Pantry 2: (8.43, 14.8)  Stores various medical supplies, such as sterilized scalpels, disposable gloves, disposable masks, etc.
			Front desk: (1.92, 7.66)  The front desk is where service personnel are located. You can inquire about hospital-related matters here. Additionally, the robot's charging station is also located here. The robot can come here to recharge when its battery is low.
			Staircase 1: (-3.77, 17.3)
			Staircase 2: (3.97, 17.5)  The stairwells and elevator of Building 1 and Building 2 are the places for going up and down stairs, and they are the necessary passage for people to enter this floor. Food delivery and packages are usually delivered to this location.
			Elevator: (0.139, 14.2)
			Beverage vending machine: (-7.03, 10.2)  You can purchase beverages at the beverage vending machine location.
			Food vending machine: (7.35, 10.2)  You can purchase food at the food vending machine location.
			Room 1: (-9.6, 10.7)  Room 1 is the pediatric examination room, where children usually go for medical treatment.
			Room 2: (-9.68, 6.02)  Room 2 is the internal medicine examination room, where patients usually go for internal medicine-related illnesses, such as heart and appendix-related diseases.
			Room 3: (10, 5.7)  Room 3 is the surgical examination room, where patients usually go for surgical-related illnesses such as fractures and sprains.
			Room 4: (9.77, 10.9)  Room 4 is the traditional Chinese medicine department, where treatments related to traditional Chinese medicine are usually conducted. Acupuncture and herbal medicine are available here.
			Lounge: (-8.83, -26.1)  The lounge is where doctors take breaks, and it is also where entertainment facilities such as sofas and televisions are located.
			Operating Room 1: (-2.94, -16.4)
			Operating Room 2: (1.59, -17.3)  Surgery Room 1 and Surgery Room 2 are where doctors perform surgeries. Currently, Surgery Room 1 is in operation.
			Ward 1: (-8.2, -6.68)  In Ward 1, there are beds numbered 1, 2, and 3.
			Ward 2: (-8.62, -18.9)  Ward 2 is an isolation ward primarily used for isolating patients infected with the novel coronavirus. The ward has beds numbered 4, 5, and 6.
			Ward 3: (2.67, -26.7)  In Ward 3, there are beds numbered 7, 8, and 9.
			Ward 4: (7.28, -17.6)  In Ward 4, there are beds numbered 10, 11, 12, and 13.
			Ward 5: (7.36, -6.98)  In Ward 5, there are beds numbered 14, 15, and 16.
			Bed 1: (-8.83, 0.924)  There is an elderly grandmother who just had heart surgery lying on bed number 1.
			Bed 2: (-9.13, -3.03)  There is a Black individual who just had their appendix removed lying on bed number 2.
			Bed 3: (-8.32, -6.22)
			Bed 4: (-9.23, -12.4)  There is an elderly person infected with the novel coronavirus lying on bed number 4.
			Bed 5: (-9.56, -15.6)  There is a Black individual infected with the novel coronavirus lying on bed number 5.
			Bed 6: (-9.35, -19.5)
			Bed 7: (0.429, -27.2)
			Bed 8: (4.37, -27.2)
			Bed 9: (7.54, -27.5)
			Bed 10: (8.03, -26.4)
			Bed 11: (9, -21.7)
			Bed 12: (8.93, -17.6)
			Bed 13: (8.9, -14)
			Bed 14: (9.14, -7.06)  There is a man with a leg fracture lying on bed number 14. All other unmentioned beds are empty.
			Bed 15: (8, -4.02)
			Bed 16: (8.26, -1.1)
			Men's restroom: (-1.66, -11)
			Women's restroom: (1.52, -11)
			Ward 1 restroom: (-6.99, 0.906)
			Ward 2 restroom: (-7.45, -12.4)
			Ward 4 restroom: (6.59, -13)
			Ward 5 restroom: (6.61, 0.432)
			Dean's office: (1.16, -4.2)  The hospital director's office is where you can find the hospital director.
			Vice Dean's office: (-1.13, -3.71)  The vice director's office is where you can find the vice director of the hospital.
			'''
			You can try to imagine the spatial structure of this corridor and the relative positions of these locations based on their coordinates. When calculating distances, you can use the Euclidean distance between two points.
			Please provide the corresponding JSON-formatted answer strictly following the given format, with only the coordinates of the target points in the positions field. Here are some examples:
				User: "I'm thirsty, please help me buy a bottle of cola."
				Assistant:
				{{
					"explanation": "Cola is a beverage, you need to go to the vending machine to purchase it.",
					"positions": [[-7.03,10.2]]
				}}

				User: "Help me get a pair of sterilized scissors and take them to Room 1."
				Assistant:
				{{
					"explanation": "The sterilized scissors are medical instruments, you need to go to Supply Room 2 to retrieve them, then proceed to Room 1.",
					"positions": [[8.43, 14.8], [-9.6, 10.7]]
				}}
			Note：It is necessary to output the result in JSON format and nothing else.					 
			Here is the task you need to execute:
			------------
    		{task}
			------------
            assistant:
			""")
    return description