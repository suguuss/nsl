vhdl-sources += transceiver.pkg.vhd
vhdl-sources += cuff_diff_receiver.vhd
vhdl-sources += cuff_diff_transmitter.vhd
# vhdl-sources += cuff_receiver.vhd
vhdl-sources += cuff_transmitter.vhd

# Spartan-6 specific receivers based on XAPP495
ifneq ($(filter xc6%,$(target_part)),)
# Version with nsl blocks
vhdl-sources += input_delay_variable_xc6.vhd
vhdl-sources += cuff_receiver_xc6_nsl.vhd

# Version based on xapp495
# vhdl-sources += receiver_xc6/phase_detector.vhd
# vhdl-sources += receiver_xc6/serdes_1_to_n_data_s8_se.vhd
# vhdl-sources += receiver_xc6/cuff_receiver_xc6.vhd

deps += unisim.vcomponents
deps += nsl_hwdep.xc6_config
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
