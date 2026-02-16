library ieee;
use ieee.std_logic_1164.all;

library nsl_cuff, nsl_line_coding, nsl_io;
use nsl_cuff.protocol.all;

entity cuff_receiver is
    generic (
        lane_count_c           : natural;
        input_fixed_delay_ps_c : natural := 0;
        has_input_alignment_c  : boolean := true;
        sdr_ddr_c              : string  := "ddr"
    );
    port (
        clock_i         : in std_ulogic;
        gearbox_clock_i : in std_ulogic := '0';
        bit_clock_i     : in std_ulogic;
        reset_n_i       : in std_ulogic;

        serdes_strobe_i : in std_ulogic := '0';

        pad_i  : in  std_ulogic_vector(0 to lane_count_c - 1);
        lane_o : out cuff_code_vector(0 to lane_count_c - 1);

        align_restart_i : in  std_ulogic;
        align_valid_i   : in  std_ulogic_vector(0 to lane_count_c - 1);
        align_ready_o   : out std_ulogic_vector(0 to lane_count_c - 1)
    );
end entity;

architecture beh of cuff_receiver is
    component serdes_1_to_n_data_s8_se is
        generic (
            S      : integer := 8;    -- Parameter to set the serdes factor 1..8
            D      : integer := 16;   -- Set the number of inputs and outputs
            USE_PD : boolean := FALSE -- Enable the use of the phase detector - will enable use of two iserdes2 even if serdes factor is 4 or less
        );
        port (
            use_phase_detector : in  std_logic;                               -- Set generation of phase detector logic
            datain             : in  std_logic_vector(D - 1 downto 0);        -- Input from se receiver pin
            rxioclk            : in  std_logic;                               -- IO Clock network
            rxserdesstrobe     : in  std_logic;                               -- Parallel data capture strobe
            reset              : in  std_logic;                               -- Reset line
            gclk               : in  std_logic;                               -- Global clock
            bitslip            : in  std_logic;                               -- Bitslip control line
            debug_in           : in  std_logic_vector(1 downto 0);            -- input debug data
            data_out           : out std_logic_vector((D * S) - 1 downto 0);  -- Output data
            debug              : out std_logic_vector((3 * D) + 5 downto 0)); -- Debug bus, 3D+5 = 3 lines per input (from inc, mux and ce) + 6, leave nc if debug not required
    end component;

    signal reset_s : std_logic;
begin

    reset_s <= not reset_n_i;

    iter : for i in 0 to lane_count_c - 1
        generate
        signal delayed_s, delay_shift_s, delay_mark_s, slip_shift_s, slip_mark_s : std_ulogic;

        signal slip_count : integer range 0 to 9;

        signal mark_s        : std_ulogic;
        signal old_mark_s    : std_ulogic;
        signal invert_s      : boolean;
        signal high_nlow_s   : boolean;
        signal parallel_s    : std_logic_vector(0 to 4);
        signal gearbox_reg_s : std_logic_vector(0 to 4);
        signal d             : std_logic_vector(0 to 9);
    begin

        aligner : nsl_io.delay.input_delay_aligner
        generic map(
            stabilization_delay_c => 15,
            stabilization_cycle_c => 3
        )
        port map(
            clock_i   => gearbox_clock_i,
            reset_n_i => reset_n_i,

            delay_shift_o  => delay_shift_s,
            delay_mark_i   => '1',
            serdes_shift_o => slip_shift_s,
            serdes_mark_i  => slip_mark_s,

            restart_i => align_restart_i,
            valid_i   => align_valid_i(i),
            ready_o   => align_ready_o(i)
        );

        slip_tracker : process (gearbox_clock_i) is
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    slip_count <= 9;
                else
                    if slip_shift_s = '1' then
                        if slip_count = 0 then
                            slip_count <= 9;
                        else
                            slip_count <= slip_count - 1;
                        end if;
                    end if;
                end if;
            end if;

        end process;

        mark_s <= '1' when slip_count = 0 else
                  '0';
        slip_mark_s <= mark_s;

        invert_s <= slip_count >= 5;

        serdes_1_to_n_data_s8_se_inst : serdes_1_to_n_data_s8_se
        generic map(
            S      => 5,
            D      => 1,
            USE_PD => true
        )
        port map(
            use_phase_detector => '1',
            datain(0)          => pad_i(i),
            rxioclk            => bit_clock_i,
            rxserdesstrobe     => serdes_strobe_i,
            reset              => reset_s,
            gclk               => gearbox_clock_i,
            bitslip            => slip_shift_s,
            debug_in           => "00",
            data_out           => parallel_s
        );

        gearbox_proc : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    high_nlow_s <= true;
                    old_mark_s  <= mark_s;
                else
                    if high_nlow_s then
                        gearbox_reg_s <= parallel_s;
                    else
                        if not invert_s then
                            d <= gearbox_reg_s & parallel_s;
                        else
                            d <= parallel_s & gearbox_reg_s;
                        end if;
                    end if;

                    old_mark_s <= mark_s;
                    if not (mark_s = '1' and old_mark_s = '0') then
                        high_nlow_s <= not high_nlow_s;
                    end if;
                end if;
            end if;
        end process;

        word_proc : process (clock_i)
        begin
            if rising_edge(clock_i) then
                lane_o(i) <= std_ulogic_vector(d);
            end if;
        end process;
    end generate;

end architecture;
