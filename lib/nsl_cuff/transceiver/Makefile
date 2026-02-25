vhdl-sources += transceiver.pkg.vhd
vhdl-sources += cuff_diff_receiver.vhd
vhdl-sources += cuff_diff_transmitter.vhd
vhdl-sources += cuff_transmitter.vhd

# Spartan-6 specific receivers
ifneq ($(filter xilinx_xc6%,$(hwdep)_$(target_part)),)
vhdl-sources += cuff_receiver_xc6.vhd
# deps += unisim.vcomponents
# deps += nsl_hwdep.xc6_config
else
vhdl-sources += cuff_receiver.vhd
endif

deps += nsl_cuff.protocol
deps += nsl_line_coding.ibm_8b10b
deps += nsl_data.bytestream
deps += nsl_io.diff
deps += nsl_io.pad
deps += nsl_io.serdes
deps += nsl_io.delay
