import asyncio
import websockets

async def test_client():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        print(await websocket.recv())  # Receive the initial message
        await websocket.send("Hello, Server!")
        print(await websocket.recv())  # Receive echo

asyncio.run(test_client())