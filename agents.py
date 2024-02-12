from textwrap import dedent
from crewai import Agent
from langchain.chat_models import ChatOpenAI
from Tools import ReadTools

OPENAI_API_KEY = ''

gpt3 = ChatOpenAI(model='gpt-3.5-turbo', openai_api_key=OPENAI_API_KEY) # Loading GPT-3.5
gpt4 = ChatOpenAI(model='gpt-4-turbo-preview', openai_api_key=OPENAI_API_KEY) # Loading GPT-4

class Agents():
	def prompt_engineer_agent(self):
		return Agent(
			role='Prompt Engineer',
			goal='Create prompt as needed',
			backstory=dedent("""\
				You are a prompt engineer responsible for a robot's intelligent navigation project. 
				You are highly skilled in robot indoor navigation tasks and prompt engineering for LLM. 
				With your utmost dedication, you aim to read the feedback and design a perfect prompt 
				that enables LLM on the robot to provide the correct output for guiding the robot's movements."""),
			allow_delegation=False,
			verbose=True,
			llm=gpt4,
			tools=[ReadTools().read]
		)

	def motion_controller_agent(self):
		return Agent(
			role='Robot Motion Controller',
  			goal='By analyzing the prompts and navigation tasks, provide a perfect JSON-formatted response to enable the robot to move correctly.',
  			backstory=dedent("""\
				You are a robot motion control home. 
				You will receive a navigation task and prompts from an expert. 
				Based on them, you are responsible for providing the correct JSON-formatted output. Your task is to ensure the robot can move correctly."""),
			allow_delegation=True,
			verbose=True,
			llm=gpt3
		)
