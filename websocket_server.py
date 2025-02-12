import asyncio
import websockets
import json

# Define the WebSocket server handler
async def handler(websocket, path=8765):

    # Send a welcome message to the client
    AddWaypointMessage = {
        "MessageType": "AddWaypoint",
        "X": 5000,
        "Y": -100000,
        "Z": 5000
    }

    DeleteWaypointMessage = {
        "MessageType": "DeleteWaypoint",
        "ID": 1
    }

    MoveWaypointMessage = {
        "MessageType": "MoveWaypoint",
        "ID": 1,
        "X": 10000,
        "Y": -10000,
        "Z": 5000
    }

    await asyncio.sleep(5)
    await websocket.send(json.dumps(AddWaypointMessage))
    await asyncio.sleep(5)
    await websocket.send(json.dumps(MoveWaypointMessage))
    await asyncio.sleep(5)
    await websocket.send(json.dumps(DeleteWaypointMessage))

# Start the server
async def start_server():
    server = await websockets.serve(handler, "localhost", 8765)
    print("Server started on ws://localhost:8765")
    await server.wait_closed()

# Run the server
asyncio.run(start_server())
