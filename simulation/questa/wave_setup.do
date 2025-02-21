onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /SafetyBoard_tb/sif/clk
add wave -noupdate /SafetyBoard_tb/sif/rst_n
add wave -noupdate -expand -group Input /SafetyBoard_tb/sif/shutdown_commands
add wave -noupdate -expand -group Input /SafetyBoard_tb/sif/router_feedback
add wave -noupdate -expand -group Input /SafetyBoard_tb/sif/requests
add wave -noupdate -expand -group Status /SafetyBoard_tb/sif/validate_state
add wave -noupdate -expand -group Status /SafetyBoard_tb/sif/feedback_timeout_error
add wave -noupdate -expand -group Status /SafetyBoard_tb/sif/invalid_request
add wave -noupdate -expand -group Status /SafetyBoard_tb/sif/current_state
add wave -noupdate -expand -group Output /SafetyBoard_tb/sif/router_cmd
add wave -noupdate -expand -group Output /SafetyBoard_tb/sif/pg_shutdown
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 285
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ns} {6144 ns}
