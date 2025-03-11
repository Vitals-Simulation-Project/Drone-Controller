import asyncio
import websockets
import json

connected_clients = []

# Define the WebSocket server handler
async def handler(websocket, path=8765):

    connected_clients.append(websocket)
    print(f"Client {websocket} connected.")

    async for message in websocket:
        
        message_data = json.loads(message)

        for client in connected_clients:
            if client != websocket:
                await client.send(message)


# Start the server
async def start_server():
    server = await websockets.serve(handler, "localhost", 8765)
    print("Server started on ws://localhost:8765")
    await server.wait_closed()

# Run the server
asyncio.run(start_server())
