# Internship & Thesis Journal
**Project**: Vision-Based Control for small UAVs

This journal is used to track daily progress, challenges, and solutions to help with the final internship/thesis report.

---

## Week 1 (July 6 - July 12, 2026)

### 🎯 Weekly Goals
- [ ] Define control logic improvements
- [ ] 

### 📝 Daily Log

#### Monday, July 6
- Restructured the codebase into an Object-Oriented Programming (OOP) architecture.
- Created core class modules (`camera`, `detector`, `tracker`, `flight_controller`, `mission_controller`, etc.) in a new `classes/` folder.
- Migrated hard-coded scripts into the new OOP framework.

#### Tuesday, July 7
- **Target Selection Logic**: Removed the old Euclidean distance-based threshold (`max_click_distance_px`). Now, the system simply checks if the operator's click coordinates fall inside a detected bounding box (`x1 <= click_x <= x2` and `y1 <= click_y <= y2`). If multiple boxes overlap, it selects the one with the smallest area.
- **Live Calibration Preview**: Overhauled the calibration process in `distance_estimator.py` to use a multithreaded live preview. The camera now streams continuously on a background thread while the main thread handles user input (`ENTER` to start/stop recording), saving per-frame area samples instead of just an average.

#### Wednesday, July 8
- *No direct code commits. (Feel free to add any research, reading, or meetings here)*

#### Thursday, July 9
- *Started journal to track progress*
- **Calibration Safety Interlocks**: Added robust safety checks to prevent calibrating while the drone is flying. Created a new network command (`CMD_CALIBRATE_CHECK`, `0x06`) so the Ground Station can ask the Jetson if calibration is allowed. If the drone is `ARMED`, it rejects the request and displays a massive red "WARNING: DRONE ARMED! DISARM TO CALIBRATE" overlay on the video feed.
- **Target Lock Requirement**: The calibration process now waits for a target to be locked (`self._locked_id`) before recording samples. It also correctly uses the tracked target's specific area rather than just blindly picking the largest detection in the frame.
- **Config Fixes**: Fixed a naming mismatch in the configuration (`fallback_optical_constant` was changed to `optical_constant` in `config.yaml` and `config.py`).
- 

#### Friday, July 10
- 

### 💡 Notes for Final Report
*Jot down architectural decisions, key learnings, or figures you might want to include in the LaTeX report later.*
- 

---

## 📅 Template for New Weeks
*Copy and paste this section when starting a new week.*

## Week X (Month DD - Month DD)

### 🎯 Weekly Goals
- [ ] 

### 📝 Daily Log

#### Monday
- 

#### Tuesday
- 

#### Wednesday
- 

#### Thursday
- 

#### Friday
- 

### 💡 Notes for Final Report
- 
