import subprocess
import sys
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path
from models import AgentLog, AgentPromptRequest, SysIDStatus
from routers import ai, connection, control, telemetry

app = FastAPI()
UI_PATH = Path(__file__).parent / "index.html"

# Serve static files for CSS and JS
app.mount("/static", StaticFiles(directory="static"), name="static")

# Include Routers
app.include_router(ai.router)
app.include_router(connection.router)
app.include_router(control.router)
app.include_router(telemetry.router)

@app.get("/")
def get_ui():
    return HTMLResponse(UI_PATH.read_text(encoding="utf-8"))

# ----- Agent Log WebSocket -----
agent_log_clients: list = []

@app.websocket("/ws/agent_logs")
async def ws_agent_logs(websocket: WebSocket):
    await websocket.accept()
    agent_log_clients.append(websocket)
    try:
        while True:
            await websocket.receive_text()
    except Exception:
        pass
    finally:
        if websocket in agent_log_clients:
            agent_log_clients.remove(websocket)

@app.post("/post_agent_log")
async def post_agent_log(req: AgentLog):
    for client in list(agent_log_clients):
        try:
            await client.send_text(req.message)
        except WebSocketDisconnect:
            if client in agent_log_clients:
                agent_log_clients.remove(client)
    return {"status": "success"}

# ----- Agent Prompt -----
active_agent_prompt = ""
agent_tuner_process = None

def kill_all_tuners():
    try:
        if sys.platform == "win32":
            subprocess.run(["wmic", "process", "where", "CommandLine like '%genai_agent_tuner.py%'", "call", "terminate"], capture_output=True)
        else:
            subprocess.run(["pkill", "-f", "genai_agent_tuner.py"], capture_output=True)
    except Exception:
        pass

# Clean up any orphaned processes on startup
kill_all_tuners()

@app.post("/api/agent_prompt")
async def post_agent_prompt(req: AgentPromptRequest):
    global active_agent_prompt
    active_agent_prompt = req.prompt
    return {"status": "ok", "prompt": active_agent_prompt}

@app.get("/api/agent_prompt")
def get_agent_prompt():
    return {"prompt": active_agent_prompt}

@app.post("/api/agent_prompt_clear")
def clear_agent_prompt():
    global active_agent_prompt
    active_agent_prompt = ""
    return {"status": "ok"}

@app.post("/api/start_tuner")
def start_tuner():
    global agent_tuner_process
    if agent_tuner_process is not None and agent_tuner_process.poll() is None:
        agent_tuner_process.terminate()
        agent_tuner_process.wait()
    kill_all_tuners()
    agent_tuner_process = subprocess.Popen([sys.executable, "genai_agent_tuner.py"])
    return {"status": "started", "pid": agent_tuner_process.pid}

@app.post("/api/stop_tuner")
def stop_tuner():
    global agent_tuner_process
    if agent_tuner_process is not None and agent_tuner_process.poll() is None:
        agent_tuner_process.terminate()
        agent_tuner_process.wait()
        agent_tuner_process = None
    kill_all_tuners()
    return {"status": "stopped"}

# ----- SysID Status -----
_sysid_status: dict = {"phase": "idle", "test": "", "progress": 0.0, "running": False}

@app.get("/sysid_status")
def get_sysid_status():
    return _sysid_status

@app.post("/post_sysid_status")
async def post_sysid_status(req: SysIDStatus):
    global _sysid_status
    _sysid_status = req.dict()
    return {"status": "ok"}

# ----- (legacy monolithic code removed — now lives in routers/) -----
