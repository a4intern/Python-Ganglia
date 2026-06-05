import asyncio
import struct
import time
import threading
import queue
import os
import json
import google.generativeai as genai

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
from pydantic import BaseModel
from pathlib import Path


app = FastAPI()
UI_PATH = Path(__file__).parent / "index.html"

# ---------------------------------------------------------
# Global State & Configurations
# ---------------------------------------------------------
modbus_client = None
DEVICE_ID = 48

modbus_lock = threading.Lock()

active_ws_queues: list[queue.Queue] = []
active_ws_queues_lock = threading.Lock()

ADDR_POS_PID   = 0
ADDR_POS_TARGET = 10
ADDR_VEL_PID   = 16
ADDR_VEL_TARGET = 26
ADDR_CUR_PID   = 32
ADDR_CUR_TARGET = 42
ADDR_PWM_VAL   = 80
ADDR_OP_MODE   = 128
ADDR_MOTOR_STAT = 0

# ---------------------------------------------------------
# High-Speed Polling Worker Thread  (~1 ms period)
# ---------------------------------------------------------
def modbus_polling_worker():
    while True:
        with modbus_lock:
            is_connected = modbus_client and modbus_client.connected

        if is_connected:
            try:
                with modbus_lock:
                    result = modbus_client.read_input_registers(
                        address=ADDR_MOTOR_STAT, count=22, device_id=DEVICE_ID
                    )

                if not hasattr(result, "isError") or not result.isError():
                    regs = result.registers

                    raw_velocity = struct.unpack("<i", struct.pack("<HH", regs[2], regs[3]))[0]
                    raw_current  = struct.unpack("<i", struct.pack("<HH", regs[4], regs[5]))[0]
                    raw_target   = struct.unpack("<i", struct.pack("<HH", regs[18], regs[19]))[0]

                    # Scale according to main.h
                    ENC_TO_RPM = 91.5527344
                    ADC_TO_MA = 4.698555425  # Assuming STM32F3 board

                    pt = {
                        "timestamp": time.time(),
                        "velocity": raw_velocity * ENC_TO_RPM,
                        "current": raw_current * ADC_TO_MA,
                        "target_velocity": raw_target * ENC_TO_RPM,
                    }

                    with active_ws_queues_lock:
                        for q in active_ws_queues:
                            try:
                                q.put_nowait(pt)
                            except queue.Full:
                                pass

            except Exception as e:
                print(f"Polling error: {e}")

        time.sleep(0.001)


threading.Thread(target=modbus_polling_worker, daemon=True).start()

# ---------------------------------------------------------
# Pydantic Models
# ---------------------------------------------------------
class ConnectRequest(BaseModel):
    port: str
    device_id: int

class PIDRequest(BaseModel):
    mode: str
    p: float
    i: float
    d: float
    gain_output: float = 1.0

class TargetRequest(BaseModel):
    mode: str
    value: int
    min_limit: int
    max_limit: int

class OpModeRequest(BaseModel):
    mode: int

class PWMRequest(BaseModel):
    value: int

class InvertRequest(BaseModel):
    invert: bool

class SysIDRequest(BaseModel):
    waveform_type: int
    amplitude: int
    frequency: int
    offset: int
    sine_enable: bool

class ChatRequest(BaseModel):
    message: str
    context: dict

# ---------------------------------------------------------
# REST API Endpoints
# ---------------------------------------------------------
@app.get("/")
def get_ui():
    return HTMLResponse(UI_PATH.read_text(encoding="utf-8"))

@app.post("/chat")
def chat_with_ai(req: ChatRequest):
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        return {"response": "Error: GEMINI_API_KEY environment variable is not set on the server."}
        
    try:
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-3.1-flash-lite')
        
        system_prompt = f"""You are an AI Tutor for a DC Motor Control Lab Experiment.
The student is using a web app to tune a DC motor using PID control.
Here is the current state of their UI:
{json.dumps(req.context, indent=2)}

Use this context to answer their questions accurately. Be encouraging, educational, and avoid giving direct answers without explanation."""
        
        prompt = f"{system_prompt}\n\nStudent asks: {req.message}"
        response = model.generate_content(prompt)
        return {"response": response.text}
    except Exception as e:
        return {"response": f"AI Error: {str(e)}"}

@app.get("/ports")
def list_ports():
    ports = serial.tools.list_ports.comports()
    return {"ports": [port.device for port in ports]}

@app.post("/connect")
def connect(req: ConnectRequest):
    global modbus_client, DEVICE_ID
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()
        DEVICE_ID = req.device_id
        modbus_client = ModbusSerialClient(
            port=req.port,
            framer=FramerType.ASCII,
            baudrate=2000000,
            timeout=0.2,
        )
        if not modbus_client.connect():
            return {"status": "failed", "message": "Cannot open COM port"}
    return {"status": "connected", "message": f"Port Open (ID: {DEVICE_ID})"}

@app.post("/disconnect")
def disconnect_port():
    global modbus_client
    with modbus_lock:
        if modbus_client and modbus_client.connected:
            modbus_client.close()
    return {"status": "disconnected", "message": "Port Disconnected"}

@app.post("/invert_encoder")
def invert_encoder(req: InvertRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(19, req.invert, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_op_mode")
def set_op_mode(req: OpModeRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  False, device_id=DEVICE_ID)
        modbus_client.write_coil(4,  req.mode == -1, device_id=DEVICE_ID)
        modbus_client.write_coil(5,  req.mode == -2, device_id=DEVICE_ID)
        modbus_client.write_coil(6,  req.mode == -3, device_id=DEVICE_ID)
        mode_val = struct.unpack("<H", struct.pack("<h", req.mode))[0]
        modbus_client.write_register(ADDR_OP_MODE, mode_val, device_id=DEVICE_ID)

        if req.mode != 7:
            modbus_client.write_coil(25, False, device_id=DEVICE_ID)
            restore_val_56 = struct.unpack("<2H", struct.pack("<I", 30000))
            modbus_client.write_registers(56, list(restore_val_56), device_id=DEVICE_ID)
            restore_val_58 = struct.unpack("<2H", struct.pack("<I", 0))
            modbus_client.write_registers(58, list(restore_val_58), device_id=DEVICE_ID)

    return {"status": "success"}

@app.post("/set_pid")
def set_pid(req: PIDRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        addresses = {
            "position": ADDR_POS_PID,
            "velocity": ADDR_VEL_PID,
            "current":  ADDR_CUR_PID,
        }
        addr = addresses.get(req.mode)
        packed_bytes = struct.pack("<ffffhH", req.p, req.i, req.d, req.gain_output, 30000, 0)
        regs = struct.unpack("<10H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_target")
def set_target(req: TargetRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        addresses = {
            "position": ADDR_POS_TARGET,
            "velocity": ADDR_VEL_TARGET,
            "current":  ADDR_CUR_TARGET,
        }
        addr = addresses.get(req.mode)
        packed_bytes = struct.pack("<iii", req.value, req.min_limit, req.max_limit)
        regs = struct.unpack("<6H", packed_bytes)
        modbus_client.write_registers(address=addr, values=list(regs), device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/start")
def start_drive():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  True, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_pwm")
def set_pwm(req: PWMRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, True, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  True, device_id=DEVICE_ID)
        val = struct.unpack("<H", struct.pack("<h", req.value))[0]
        modbus_client.write_register(ADDR_PWM_VAL, val, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/stop")
def stop_drive():
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        modbus_client.write_coil(13, False, device_id=DEVICE_ID)
        modbus_client.write_coil(3,  False, device_id=DEVICE_ID)
    return {"status": "success"}

@app.post("/set_sysid")
def set_sysid(req: SysIDRequest):
    with modbus_lock:
        if not modbus_client or not modbus_client.connected:
            return {"error": "Not connected"}
        
        modbus_client.write_coil(25, req.sine_enable, device_id=DEVICE_ID)
        freq_val = struct.unpack("<H", struct.pack("<h", req.frequency))[0]
        modbus_client.write_register(70, freq_val, device_id=DEVICE_ID)
        off_val = struct.unpack("<H", struct.pack("<h", req.offset))[0]
        modbus_client.write_register(71, off_val, device_id=DEVICE_ID)
        
        amp_bytes = struct.pack("<I", req.amplitude)
        amp_regs = struct.unpack("<2H", amp_bytes)
        modbus_client.write_registers(56, list(amp_regs), device_id=DEVICE_ID)
        
        wv_bytes = struct.pack("<I", req.waveform_type)
        wv_regs = struct.unpack("<2H", wv_bytes)
        modbus_client.write_registers(58, list(wv_regs), device_id=DEVICE_ID)
        
    return {"status": "success"}

# ---------------------------------------------------------
# WebSocket Endpoint
# ---------------------------------------------------------
@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    await websocket.accept()
    start_time = time.time()

    q: queue.Queue = queue.Queue(maxsize=5000)
    with active_ws_queues_lock:
        active_ws_queues.append(q)

    try:
        while True:
            pts = []
            while True:
                try:
                    pt = q.get_nowait()
                    pts.append({
                        "time":     pt["timestamp"] - start_time,
                        "velocity": pt["velocity"],
                        "current":  pt["current"],
                        "target_velocity": pt.get("target_velocity", 0),
                    })
                except queue.Empty:
                    break

            if pts:
                await websocket.send_json(pts)

            await asyncio.sleep(0.005)

    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"Telemetry WebSocket error: {e}")
    finally:
        with active_ws_queues_lock:
            if q in active_ws_queues:
                active_ws_queues.remove(q)