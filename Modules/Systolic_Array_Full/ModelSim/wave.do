onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label Clock -radix binary /testbench/clk
add wave -noupdate -label Reset -radix binary /testbench/rst

add wave -noupdate -divider "Systolic Array"

add wave -noupdate -label enable -radix hexadecimal /testbench/en
add wave -noupdate -label in_left -radix hexadecimal /testbench/in_left
add wave -noupdate -label in_top -radix hexadecimal /testbench/in_top
add wave -noupdate -label acc_out -radix hexadecimal /testbench/acc_out

TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {10000 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 80
configure wave -valuecolwidth 40
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
WaveRestoreZoom {0 ps} {500 ns}
