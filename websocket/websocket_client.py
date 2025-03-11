import asyncio
import websockets
import json

URI = "ws://localhost:8765"

async def test_client():
    '''Test connection by sending messages to server'''
    
    async with websockets.connect(URI) as websocket:

        AddWaypointMessage = {
            "MessageType": "AddWaypoint",
            "X": 3000,
            "Y": 0,
            "Z": 5000
        }

        DeleteWaypointMessage = {
            "MessageType": "DeleteWaypoint",
            "ID": 1
        }

        UpdateStateMessage = {
            "MessageType": "UpdateDroneState",
            "DroneID": 0,
            "State": "SEARCHING"
        }

        UpdateTargetMessage = {
            "MessageType": "UpdateDroneTarget",
            "DroneID": 1,
            "WaypointID": 3
        }

        UpdateStateMessage2 = {
            "MessageType": "UpdateDroneState",
            "DroneID": 2,
            "State": "SEARCHING"
        }

        UpdateTargetMessage2 = {
            "MessageType": "UpdateDroneTarget",
            "DroneID": 3,
            "WaypointID": 3
        }

        TIME_BETWEEN_MESSAGES = 2
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(AddWaypointMessage))
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(DeleteWaypointMessage))
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(UpdateStateMessage))
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(UpdateTargetMessage))
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(UpdateStateMessage2))
        await asyncio.sleep(TIME_BETWEEN_MESSAGES)
        await websocket.send(json.dumps(UpdateTargetMessage2))

asyncio.run(test_client())