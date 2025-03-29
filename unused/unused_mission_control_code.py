# message history, starts off with explaining what the model 
# will be doing in the search and rescue mission
# message_history = [
#     {
#         'role': 'user',
#         'content':  
#             "You are a drone operator conducting a simulated search and rescue mission.\n" +
#             f"You have {DRONE_COUNT} drone(s) at your disposal to locate a missing person.\n" +
#             "Your primary task is to create and assign waypoints for the drones, analyze captured images, and determine the target's location."
#         ,
#     },
#     {
#         'role': 'assistant',
#         'content': "Understood. I am ready to assist in the search and rescue mission.",
#     },
#     {
#         'role': 'user',
#         'content': (
#             "The search process follows these steps:\n"
#             "1. You will receive a list of waypoints with their locations, along with the current locations of the drones.\n"
#             "2. Using this information, generate a target waypoint for each drone, utilizing the distance they are away and the waypoint's priority.\n"
#             "3. The drones will travel to their individual waypoint and capture infrared (IR) images to detect potential heat signatures.\n"
#             "4. If a heat signature is detected, the drones will take regular images to visually confirm the target.\n"
#             "5. The drones will transmit regular images to you for analysis.\n"
#             "6. Based on your analysis, determine whether the detected heat signature matches the missing person.\n"
#             "7. If the target is confirmed, the mission is complete. If not, repeat the process until the target is found. You will need to assign a new waypoint if a drone has finished searching its current target.\n"
#             "8. You may be asked to generate additional waypoints if all waypoints have been searched. You will be provided with a list of already searched locations."
#             "Your objective is to accurately locate and confirm the missing person’s position using drone imagery.\n"
#             "Now, let's begin by generating the initial target waypoints for each drone."
#         ),
#     },
#     {
#         'role': 'assistant',
#         'content': "Please provide the initial waypoint list and wait while I generate the target waypoints for each drone.",
#     }
# ]