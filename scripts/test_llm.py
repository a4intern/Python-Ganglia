"""Quick LLM connection test. Run from project root: python3 scripts/test_llm.py"""
import sys, os
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent / "agents"))
from dotenv import load_dotenv
load_dotenv()

print("=== LLM Connection Test ===")
print(f"Provider : {os.environ.get('LLM_PROVIDER', 'gemini')}")
print(f"Base URL : {os.environ.get('LLM_BASE_URL', '(gemini native)')}")
print(f"Model    : {os.environ.get('LLM_MODEL', 'default')}")
print(f"Key      : {(os.environ.get('LLM_API_KEY') or os.environ.get('GEMINI_API_KEY') or '')[:12]}...")
print()

try:
    from llm_backends import create_backend, TuningResult
    backend = create_backend()
    print(f"Backend  : {backend.__class__.__name__} ✓")
except Exception as e:
    print(f"Backend init FAILED: {e}")
    sys.exit(1)

print("Sending test prompt...")
try:
    result = backend.complete(
        "You are a control systems engineer.",
        'Return JSON with: phase="STEP_POS", wc=5.0, b0=10.0, ramp_time=0.0, target_velocity=5.0, reasoning="connection test ok"',
    )
    print()
    print("=== Response ===")
    print(f"phase           : {result.phase}")
    print(f"wc              : {result.wc}")
    print(f"b0              : {result.b0}")
    print(f"target_velocity : {result.target_velocity}")
    print(f"reasoning       : {result.reasoning}")
    print()
    print("Connection OK ✓")
except Exception as e:
    print(f"\nRequest FAILED: {e}")
    sys.exit(1)
