transcript on
if {[file exists gate_work]} {
	vdel -lib gate_work -all
}
vlib gate_work
vmap work gate_work

vlog -vlog01compat -work work +incdir+. {SafetyBoard.vo}

vlog -sv -work work +incdir+C:/Users/liley/Documents/SafetyBoard {C:/Users/liley/Documents/SafetyBoard/SafetyBoard.sv}

vsim -t 1ps -L altera_ver -L altera_lnsim_ver -L fiftyfivenm_ver -L gate_work -L work -voptargs="+acc"  SafetyBoarTestBench

add wave *
view structure
view signals
run -all
