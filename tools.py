from ollama import ChatResponse, chat # type: ignore


def add_two_numbers(a: int, b: int) -> int:
  """
  Add two numbers

  Args:
    a (int): The first number
    b (int): The second number

  Returns:
    int: The sum of the two numbers
  """
  return a + b


def subtract_two_numbers(a: int, b: int) -> int:
  """
  Subtract two numbers
  """
  return a - b




messages = [{'role': 'user', 'content': 'What is three plus one?'}]
print('Prompt:', messages[0]['content'])

available_functions = {
  'add_two_numbers': add_two_numbers,
  'subtract_two_numbers': subtract_two_numbers,
}

response: ChatResponse = chat(
  'llama3.2:latest',
  messages=messages,
  tools=[add_two_numbers, subtract_two_numbers],
)

if response.message.tool_calls:
  # There may be multiple tool calls in the response
  for tool in response.message.tool_calls:
    # Ensure the function is available, and then call it
    if function_to_call := available_functions.get(tool.function.name):
      print('Calling function:', tool.function.name)
      print('Arguments:', tool.function.arguments)
      output = function_to_call(**tool.function.arguments)
      print('Function output:', output)
    else:
      print('Function', tool.function.name, 'not found')

# Only needed to chat with the model using the tool call results
if response.message.tool_calls:
  # Add the function response to messages for the model to use
  messages.append(response.message)
  messages.append({'role': 'tool', 'content': str(output), 'name': tool.function.name})
  print('Messages:', messages)
  # Get final response from model with function outputs
  final_response = chat('llama3.2:latest', messages=messages)
  print('Final response:', final_response.message.content)

else:
  print('No tool calls returned from model')
