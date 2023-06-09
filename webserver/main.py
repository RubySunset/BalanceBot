from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import time
import json

app = FastAPI()

origins = ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ConnectionManager:
    def __init__(self):
        self.active_connection = None
        self.logs = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connection = websocket
    
    def disconnect(self):
        self.active_connection = None

    # Give dictionary argument when calling function
    async def send_data(self, data: dict):
        if self.active_connection:
            await self.active_connection.send_json(data)

    async def send_log(self, message:str):
        t = time.localtime()
        cur_time = time.strftime("%H:%M:%S", t)
        if self.active_connection:
            log = {"type": "log", "time": cur_time, "message": message}
            self.logs.append(log)
            await self.active_connection.send_json(log)

frontend_manager = ConnectionManager()
rover_manager = ConnectionManager()

@app.websocket("/ws/frontend")
async def websocket_frontend(websocket: WebSocket):
    await frontend_manager.connect(websocket)
    await frontend_manager.send_log("Frontend connected")
    try:
        while True:
            data = await websocket.receive_text()
            command = json.loads(data)
            if command['action'] == 'start':
                print("Start request from front end received")
                pass
            elif command['action'] == 'stop':
                print("Stop request from front end received")
                pass
    except WebSocketDisconnect:
        print("Frontend disconnected")

@app.websocket("/ws/rover")
async def websocket_rover(websocket: WebSocket):
    await rover_manager.connect(websocket)
    await frontend_manager.send_log("Rover connected")
    await frontend_manager.send_data({"type": "connection_status", "connected": True})
    try:
        while True:
            data = await websocket.receive_text()
            rover_data = json.loads(data)
            if rover_data['type'] == 'LDR':
                pass
            elif rover_data['type'] == 'Beacons':
                pass
            elif rover_data['type'] == 'log':
                await frontend_manager.send_log(rover_data['message'])
    except WebSocketDisconnect:
        rover_manager.disconnect()
        await frontend_manager.send_log("Rover disconnected")
        await frontend_manager.send_data({"type": "connection_status", "connected": False})
    
@app.get("/")
def index():
  return {"message": "Hello World!"}

@app.get("/logs")
def get_logs():
    print(frontend_manager.logs)
    return {'logs': frontend_manager.logs}

@app.post("/logs")
def reset_logs():
    print("Logs reset")
    frontend_manager.logs = []

@app.get("/connection_status")
def get_connection_status():
    is_connected = rover_manager.active_connection is not None
    print(f"Connection status: {is_connected}")
    return {"connected": is_connected}

