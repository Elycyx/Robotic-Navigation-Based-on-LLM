import json
import requests

from crewai import Agent
from langchain.tools import tool

class ReadTools():
  @tool("Read the content of a file")
  def read(i):
        """Useful to read a file content
        Args:
            i (int): The index of the data to read. Please avoid using any symbols, including line breaks, etc. 
        """
        file_name='feedback/fb.json'
        i = int(i.strip())
        try:
            # 尝试从 JSON 文件读取数据
            with open(file_name, 'r') as file:
                data_loaded = json.load(file)
            
            # 检查请求的索引是否在数据范围内
            if i < len(data_loaded):
                result_n = data_loaded[i].get('result', '')  # 使用 get 防止 KeyError
                success_n = data_loaded[i].get('success', '')
                error_cause_n = data_loaded[i].get('error_cause', '')
                return f"Result: {result_n}, Success: {success_n}, Error Cause: {error_cause_n}"
            else:
                # 请求的索引超出了数据范围
                return ''
        except (json.JSONDecodeError, FileNotFoundError):
            # 文件为空或不是有效的 JSON，或文件不存在
            return ''


