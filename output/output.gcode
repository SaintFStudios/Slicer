; 4-Axis XYZB Gcode
; Generated from C:/Users/Jerem/Downloads/Prints/4-Axis/Slicer/tube.stl
; Duet3 RepRapFirmware

; Start sequence
G28  ; Home all axes
G29  ; Auto leveling (if configured)
G1 Z5 F300  ; Lift nozzle

; Heating
M104 S200  ; Set extruder temp
M109 S200  ; Wait for extruder
M140 S60   ; Set bed temp
M190 S60   ; Wait for bed

; End sequence
M104 S0     ; Turn off extruder
M140 S0     ; Turn off bed
G28 X0 Y0   ; Home X and Y
G1 Z11.5 F300
M84         ; Disable motors