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
3. **Tab Redirection on Connection/Start Actions:**
   - When viewing the `Velocity Agent` tab, clicking "Connect" or "Start Motor" redirects the UI back to the default `Velocity` tab, making it hard to see what is currently controlling the motor. The UI should stay on the active tab.

## Agent State Retention
4. **Agent State Persistence/Stale Command Issue:**
   - The tuning agent seems to retain or restore stale target states upon start. For example, starting the program or tuner without any active command might immediately run the motor at a previously used target (like 50 RPM or 1000 RPM).
