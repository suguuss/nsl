vhdl-sources += serdes.pkg.vhd

deps += nsl_data.text

ifeq ($(hwdep),xilinx)
ifneq ($(filter xc7%,$(target_part)),)
vhdl-sources += serdes_ddr10_output_series7.vhd
vhdl-sources += serdes_ddr10_input_series7.vhd
endif
ifneq ($(filter xc6%,$(target_part)),)
vhdl-sources += serdes_sdr10_input_series6.vhd
vhdl-sources += serdes_sdr10_output_series6.vhd
endif
deps += unisim.vcomponents
endif

ifeq ($(hwdep),gowin)
vhdl-sources += serdes_ddr10_output_gowin.vhd
ifneq ($(filter GW5%,$(target_part)),)
vhdl-sources += serdes_ddr10_input_gowin_gw5.vhd
else
vhdl-sources += serdes_ddr10_input_gowin.vhd
endif
endif

ifeq ($(hwdep),simulation)
vhdl-sources += serdes_ddr10_output_simulation.vhd
vhdl-sources += serdes_ddr10_input_simulation.vhd
vhdl-sources += serdes_sdr10_output_simulation.vhd
vhdl-sources += serdes_sdr10_input_simulation.vhd
endif
