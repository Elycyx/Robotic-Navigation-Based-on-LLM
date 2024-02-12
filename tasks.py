from textwrap import dedent
from crewai import Task

class Tasks():
	def prompt_task(self, agent, task, i):
		return Task(description=dedent(f"""You will create a prompt for llm to generate the correct output for guiding the robot's movements.
			Please help LLM understand the requirements of the input task correctly and break down the task clearly, so that the llm knows the positions to head for.
			The llm only needs to provide the target points to visit in sequential order and doesn't need to worry about how to get there.
			Here is the spatial information of the environment:
				The starting point is the current location of the robot. There is no need to consider where to depart from.
				------------
				Staircase: (5.48, 2.91) The essential location for accessing this floor.
				200: (6.89, -4.18)
				201: (-2.32, -4.77) This is my workspace, I am normally there.
				202: (0.68, -3.25) This is the drone lab.
				203: (-7.84, -4.80) There is a printer here.
				204: (-6.39, -3.00) This is where my senior colleague is.
				205: (-15.69, -4.52) This is the soft robot lab.
				206: (-13.32, -2.92)
				207: (-21.00, -4.71)
				Rest area: (-21.16, 0.65) You can recharge here.
				210: (-30.26, -3.45)
				211: (-33.30, -4.78) There is a large tool area here.
				212: (-33.83, -3.17)
				Kitchen: (-50.63, -2.67)
				Toilets (two of them): (-44.31, -0.70) and (8.64, 3.30)
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
			Use your "read" tool before starting the work to receive the output results and the success or failure 
			of the {i-1}th task as given by LLM during the previous round of work. Success=1 means the task was successful, Success=0 means the task failed.
			Adjust your prompt based on this feedback. 
			For those failed tasks, analyse the mistakes, help the LLM avoid mistakes and improve its performance.
			!!!For those successful tasks, you can directly tell the LLM the successful positions(RIGHT ANSWER) it gave.Like this: RIGHT ANSWER:{{[[x1,y1],[x2,y2]]}}
			Of course, if you find the feedback content to be empty, it means it's the first round of attempts. Just do your best to perform at your highest capability.
			Your Final answer must be the prompt for the llm, only the prompt and nothing else.
			"""),
			agent=agent
		)

	def control_task(self, agent, task):
		return Task(description=dedent(f"""\
			Here is some information you might need:
			Information
			------------
			spatial:
				Staircase: (5.48, 2.91) The essential location for accessing this floor.
				200: (6.89, -4.18)
				201: (-2.32, -4.77) This is my workspace, I am normally there.
				202: (0.68, -3.25) This is the drone lab.
				203: (-7.84, -4.80) There is a printer here.
				204: (-6.39, -3.00) This is where my senior colleague is.
				205: (-15.69, -4.52) This is the soft robot lab.
				206: (-13.32, -2.92)
				207: (-21.00, -4.71)
				Rest area: (-21.16, 0.65) You can recharge here.
				210: (-30.26, -3.45)
				211: (-33.30, -4.78) There is a large tool area here.
				212: (-33.83, -3.17)
				Kitchen: (-50.63, -2.67)
				Toilets (two of them): (-44.31, -0.70) and (8.64, 3.30)
			Standard output format:
				{{
					"explanation": "Logical analysis step by step for the task, based on which other content is generated.",
					"positions": "Coordinates of the target points to be visited in order, stored as a list.",
				}}
			examples:
				User: "I'm thirsty. I remember there are some drinks in the refrigerator in the kitchen."
				Assistant:
				{{
					"explanation": "Need to go to the refrigerator to get drinks. The refrigerator is located in the kitchen, so go to the kitchen.",
					"positions": [[-50.63, -2.67]]
				}}

				User: "I'm giving you a document. Please make a copy and deliver it to room 201."
				Assistant:
				{{
					"explanation": "Need to make a copy of the document and deliver it to room 201. There is a printer in room 203, so go there first to make the copy, and then go to room 201.",
					"positions": [[-7.84, -4.80], [-2.32, -4.77]]
				}}
			------------					 
			Here is the task you need to complete:

			task
			------------
			{task}
			------------

			You need to provide a perfect JSON-formatted output based on the received prompts and navigation tasks.
			The starting point is the current location of the robot. There is no need to consider where to depart from.
			Do not use line breaks in the explanation. Only use periods and commas for punctuation.
			The "positions" should only contain a list of coordinates and should not include anything else.
			Your Final answer must be the JSON-formatted output, only the JSON-formatted output and nothing else.
			"""),
			agent=agent
		)