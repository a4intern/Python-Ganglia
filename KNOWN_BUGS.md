# Known Bugs and UI/UX Feedback

This document tracks known issues, feature requests, and feedback regarding the motor simulation and tuning user interface.

## UI Control Flow & Button Separation
1. **Separate Motor vs. Controller Engagement:**
   - **Start Motor:** Clicking "Start Motor" should start the motor in open-loop control mode.
   - **Start Control:** There should be a separate "Start Control" button to engage the active feedback controller (ADRC, PID, etc.).

## Tab Switching & Mode Control
2. **Prevent Accidental Mode Switching on Tab Switch:**
   - Switching tabs (e.g., from `Velocity` to `Velocity Agent`) currently switches the active control mode immediately.
   - Mode switching should be blocked while a controller is actively running. The user must click "Stop Control" first before they can switch tabs and engage a different control mode.
3. **[RESOLVED] Tab Redirection on Connection/Start Actions:**
   - **Issue:** When viewing the `Velocity Agent` tab, clicking "Connect" or "Start Motor" redirects the UI back to the default `Velocity` tab, making it hard to see what is currently controlling the motor.
   - **Fix:** Removed the hardcoded `switchTab('vel', -2)` and `setTarget` calls from the `connectPort` function in `templates/sim_index.html`. The UI now correctly remains on the active tab when the connection is established.

## Agent State Retention
4. **[RESOLVED] Agent State Persistence/Stale Command Issue:**
   - **Issue:** Starting the program or tuner without any active command might immediately run the motor at a previously used target (like 50 RPM or 1000 RPM).
   - **Fix:** 
     - Modified `/api/start_tuner` and `/api/stop_tuner` in `main.py` to reset the global `active_agent_prompt` state. This prevents stale user prompt overrides from a previous run from taking effect.
     - Added an explicit reset of the motor's target velocity to `0 RPM` on startup in `genai_agent_tuner.py` to prevent the motor from launching to stale speeds when starting the agent.
