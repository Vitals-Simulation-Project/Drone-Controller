import asyncio
import websockets

HOST = "localhost"
PORT_NUMBER = 8765

connected_clients = []

async def handler(websocket):
    '''Handles client connection to websocket server'''

    # Add new connected client to list
    connected_clients.append(id(websocket))
    print(f"Client {id(websocket)} connected.")

    try:
        # For every message sent to server, send the message to all other clients, excluding sender
        async for message in websocket:
            for client in connected_clients:
                if client != websocket:
                    await client.send(message)

    except websockets.exceptions.ConnectionClosed:
        print(f"Client {id(websocket)} disconnected.")
    finally:
        connected_clients.remove(id(websocket))

async def start_websocket_server():
    '''Starts the websocket server on \"ws://localhost:8765\"'''

    server = await websockets.serve(handler, HOST, PORT_NUMBER)
    print(f"Server started on ws://{HOST}:{PORT_NUMBER}")
    await server.wait_closed()

def start():
    '''Starts the websocket server in an asyncio event loop'''
    
    server_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(server_loop)
    server_loop.run_until_complete(start_websocket_server())