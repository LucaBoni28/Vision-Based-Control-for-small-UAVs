# Deadzone History Note

## Runs 001 through 010
These runs used **velocity-based deadzones** for yaw and vz.
This means the deadzone was applied to the controller's output (e.g. `abs(omega_z) < deadzone`), which meant the physical size of the deadzone varied depending on the PID gains used.

## Runs 011 and onwards
These runs use **error-based deadzones**. 
The deadzone is applied directly to the normalized image error (e.g. `abs(e_x) < deadzone`), meaning the deadzone is a fixed physical size regardless of PID gains. 

The `metadata.txt` for runs 011+ will explicitly list `deadzone_type: error-based`.
