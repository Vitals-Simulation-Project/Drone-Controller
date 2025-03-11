import multiprocessing as mp
import time
import websocket_server

server_process = None

def start_server():
    '''Start the WebSocket server as a background process'''

    global server_process
    if server_process is None or not server_process.is_alive():
        print("Starting WebSocket server...")
        server_process = mp.Process(target=websocket_server.start, daemon=True)
        server_process.start()
    else:
        print("Server is already running.")

def stop_server():
    '''Stop the server background process'''

    global server_process
    if (server_process is not None and server_process.is_alive()):
        print("Stopping WebSocket Server...")
        server_process.terminate()
        server_process.join()
        server_process = None
        print("Stopped WebSocket Server.")
    else:
        print("Server is not running.")

if __name__ == "__main__":
    start_server()
    time.sleep(5)
    stop_server()