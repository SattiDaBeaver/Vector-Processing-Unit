onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label CLOCK_50 -radix binary /testbench/CLOCK_50
add wave -noupdate -label Reset -radix binary /testbench/rst

add wave -noupdate -divider UART

add wave -noupdate -label DataIn -radix hexadecimal /testbench/U1/dataIn
add wave -noupdate -label TX -radix hexadecimal /testbench/U1/TXout
add wave -noupdate -label busy -radix hexadecimal /testbench/U1/busy
add wave -noupdate -label State -radix hexadecimal /testbench/U1/state
add wave -noupdate -label Packet -radix hexadecimal /testbench/U1/packet
add wave -noupdate -label Index -radix hexadecimal /testbench/U1/index


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
WaveRestoreZoom {0 ps} {200 ns}
