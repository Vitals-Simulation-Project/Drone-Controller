import asyncio
import websockets
from test import sendCoordinates
import json

# Define the WebSocket server handler
async def handler(websocket, path=8765):
    # Send a welcome message to the client
    message = str(sendCoordinates())
    await websocket.send(message)
    # Receive messages from the client
    async for message in websocket:
        print(f"Received message: {message}")
        # Send the message back to the client
        await websocket.send(f"Echo: {message}")

# Start the server
async def start_server():
    server = await websockets.serve(handler, "localhost", 8765)
    print("Server started on ws://localhost:8765")
    await server.wait_closed()

# Run the server
asyncio.run(start_server())
