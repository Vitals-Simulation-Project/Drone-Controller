from ollama import ChatResponse, chat  # type: ignore

def add_two_numbers(a: int, b: int) -> int:
    """
    Add two numbers

    Args:
        a (int): The first number
        b (int): The second number

    Returns:
        int: The sum of the two numbers
    """
    return int(a) + int(b)

def subtract_two_numbers(a: int, b: int) -> int:
    """
    Subtract two numbers

    Args:
        a (int): The first number
        b (int): The second number

    Returns:
        int: The difference of the two numbers
    """
    return int(a) - int(b)

# Initial message from the user
messages = [{'role': 'user', 'content': 'Add the numbers 1 and 2. Then subtract 3 from the result.'}]
print('Prompt:', messages[0]['content'])

# Available functions
available_functions = {
    'add_two_numbers': add_two_numbers,
    'subtract_two_numbers': subtract_two_numbers,
}

# Call the model for the initial response
response: ChatResponse = chat(
    'llama3.2:latest',
    messages=messages,
    tools=[add_two_numbers, subtract_two_numbers],
)

# Process tool calls in sequence
for _ in range(5):  # Assume a maximum of 5 operations for demonstration
    if response.message.tool_calls:
        for tool in response.message.tool_calls:
            # Ensure the function is available, and then call it
            if function_to_call := available_functions.get(tool.function.name):
                print('Calling function:', tool.function.name)
                print('Arguments:', tool.function.arguments)
                output = function_to_call(**tool.function.arguments)
                print('Function output:', output)

                # Add the tool's result back to messages
                messages.append({
                    'role': 'tool',
                    'content': str(output),
                    'name': tool.function.name
                })

                # Add the tool's result as input for the next operation
                messages.append({'role': 'user', 'content': f'Continue with result {output}'})
            else:
                print('Function', tool.function.name, 'not found')
        
        # Get the next response using updated messages
        response = chat('llama3.2:latest', messages=messages)
    else:
        print('No tool calls returned from model')
        break

# Final output from the model
if not response.message.tool_calls:
    print('Final response:', response.message.content)
