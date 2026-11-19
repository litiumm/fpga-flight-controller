RTL    := rtl/pid_core.v rtl/mixer.v rtl/pwm_gen.v
IVFLAGS := -g2012 -Wall -I sim

.PHONY: all gen test sim check hil-sim synth flash clean

all: test sim check hil-sim

gen:
	python3 model/fc_model.py gen

build results:
	mkdir -p $@

# Unit test: 20k random/edge-case vectors vs bit-exact golden model
test: gen | build results
	iverilog $(IVFLAGS) -o build/tb_vectors sim/tb_pid_vectors.v rtl/pid_core.v
	vvp -n build/tb_vectors

# Closed-loop simulation against the plant model
sim: gen | build results
	iverilog $(IVFLAGS) -o build/tb_closed sim/tb_closed_loop.v rtl/pid_core.v rtl/mixer.v
	vvp -n build/tb_closed

# Compare RTL run against golden + float references, make plots
check:
	python3 model/fc_model.py check

# End-to-end: FPGA top level driven over UART by a PC model (~1 min)
hil-sim: gen | build
	iverilog $(IVFLAGS) -o build/tb_hil sim/tb_hil_top.v rtl/hil_top.v rtl/uart_rx.v rtl/uart_tx.v $(RTL)
	vvp -n build/tb_hil

# Tang Nano 9K bitstream (open-source Gowin flow: yosys + nextpnr + apicula)
DEVICE := GW1NR-LV9QN88PC6/I5
FAMILY := GW1N-9C
# -nodsp: apicula (current oss-cad-suite) cannot pack DSP input registers yet,
# so the multiplier is built from LUTs. `make synth SYNTH_OPTS=` to try the DSP.
SYNTH_OPTS ?= -nodsp
HIL_RTL := rtl/hil_top.v rtl/uart_rx.v rtl/uart_tx.v $(RTL)

synth: | build results
	yosys -q -l build/yosys.log -p "read_verilog $(HIL_RTL); synth_gowin -top hil_top $(SYNTH_OPTS) -json build/hil.json; tee -o results/utilization.txt stat"
	nextpnr-himbaechel --json build/hil.json --write build/hil_pnr.json \
	    --device $(DEVICE) --vopt family=$(FAMILY) --vopt cst=fpga/tangnano9k.cst \
	    --freq 27 --report results/pnr_report.json 2>&1 | tee build/nextpnr.log | grep -E "Max frequency|ERROR|logic LUTs|DFF|MULT|IOB" || true
	gowin_pack -d $(FAMILY) -o build/hil.fs build/hil_pnr.json

flash: synth
	openFPGALoader -b tangnano9k build/hil.fs

clean:
	rm -rf build results/*.csv results/*.vcd
