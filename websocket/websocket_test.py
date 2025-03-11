import asyncio
import websockets
import json

async def test_client():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
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

        sleep_time = 2
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(AddWaypointMessage))
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(DeleteWaypointMessage))
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(UpdateStateMessage))
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(UpdateTargetMessage))
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(UpdateStateMessage2))
        await asyncio.sleep(sleep_time)
        await websocket.send(json.dumps(UpdateTargetMessage2))

asyncio.run(test_client())