import asyncio
import websockets

HOST = "localhost"
PORT_NUMBER = 8765

CLIENTS = set()


async def send(client, message):
    '''Send a message to the client asynchronously'''

    await client.send(message)


async def handler(websocket):
    '''Handles client connection to websocket server'''

    # Add new connected client to list
    CLIENTS.add(websocket)
    print(f"Client {id(websocket)} connected.")

    try:
        # For every message sent to server, send the message to all other clients, excluding sender
        async for message in websocket:
            for client in CLIENTS:
                if client != websocket:
                    # Create a task to concurrently send messages to clients
                    asyncio.create_task(send(client, message))

    except websockets.ConnectionClosed:
        print(f"Client {id(websocket)} disconnected.")

    finally:
        CLIENTS.remove(websocket)


async def start():
    '''Starts the websocket server on \"ws://localhost:8765\"'''

    server = await websockets.serve(handler, HOST, PORT_NUMBER)
    print(f"Server started on ws://{HOST}:{PORT_NUMBER}")
    await server.wait_closed()


def start_websocket_server():
    '''Starts the websocket server in an asyncio event loop'''

    try:
        server_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(server_loop)
        server_loop.run_until_complete(start())
        
    except KeyboardInterrupt:
        ...