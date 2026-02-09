-- CUFF Receiver for Spartan-6 based on XAPP495
-- Integrates IODELAY2 + ISERDES2 + calibration FSM + gearbox + alignment
--
-- Two architectures available:
--   - xc6: Hardware synthesis (DIFF_PHASE_DETECTOR + RETIMED) - XAPP495 style
--   - xc6_sim: Simulation-compatible (NETWORKING interface) - default for GHDL
--
-- For synthesis, explicitly select architecture xc6
-- For GHDL simulation, xc6_sim is the default (defined last)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library nsl_cuff;
use nsl_cuff.protocol.all;

library unisim;
use unisim.vcomponents.all;

entity cuff_receiverclaude is
    generic (
        lane_count_c           : natural;
        input_fixed_delay_ps_c : natural := 0;
        has_input_alignment_c  : boolean := true;
        sdr_ddr_c              : string  := "sdr"
    );
    port (
        clock_i         : in std_ulogic; -- Word clock (1x, e.g. 100MHz)
        gearbox_clock_i : in std_ulogic; -- Gearbox clock (2x, e.g. 200MHz)
        bit_clock_i     : in std_ulogic; -- Bit clock (10x, e.g. 1GHz)
        reset_n_i       : in std_ulogic;
        serdes_strobe_i : in std_ulogic; -- From BUFPLL

        pad_i  : in  std_ulogic_vector(0 to lane_count_c - 1); -- Single-ended inputs
        lane_o : out cuff_code_vector(0 to lane_count_c - 1);  -- 10-bit outputs

        align_restart_i : in  std_ulogic;
        align_valid_i   : in  std_ulogic_vector(0 to lane_count_c - 1);
        align_ready_o   : out std_ulogic_vector(0 to lane_count_c - 1)
    );
end entity;

--------------------------------------------------------------------------------
-- Hardware synthesis architecture using XAPP495 approach
-- Uses DIFF_PHASE_DETECTOR + RETIMED for optimal hardware performance
--------------------------------------------------------------------------------
architecture xc6 of cuff_receiverclaude is
    constant left_to_right_c : boolean := false;

begin

    lane_gen : for i in 0 to lane_count_c - 1 generate
        signal delay_m_dataout : std_ulogic;
        signal delay_s_dataout : std_ulogic;
        signal delay_m_dout    : std_ulogic;
        signal delay_s_dout    : std_ulogic;
        signal delay_busy      : std_ulogic;
        signal delay_cal       : std_ulogic;
        signal delay_ce        : std_ulogic;
        signal delay_inc       : std_ulogic;
        signal delay_rst       : std_ulogic;
        signal serdes_cascade  : std_ulogic;
        signal serdes_valid    : std_ulogic;
        signal serdes_incdec   : std_ulogic;
        signal serdes_bitslip  : std_ulogic;
        signal parallel_s      : std_ulogic_vector(0 to 4);
        signal d               : std_ulogic_vector(0 to 9);
        signal d_word_sync     : std_ulogic_vector(0 to 9);
        signal gearbox_reg_s   : std_ulogic_vector(0 to 4);
        signal high_nlow_s     : boolean;
        signal invert_s        : boolean;
        signal slip_count      : integer range 0 to 9;
        signal mark_s          : std_ulogic;
        signal old_mark_s      : std_ulogic;
        signal cal_state       : unsigned(2 downto 0);
        signal cal_counter     : unsigned(8 downto 0);
        signal enable_cal      : std_ulogic;
        signal pdcounter       : unsigned(4 downto 0);
        signal pd_edge         : std_ulogic;
        signal ce_p            : std_ulogic;
        signal inc_p           : std_ulogic;
        type align_state_t is (ALIGNING, ALIGNED);
        signal align_state     : align_state_t;
        signal align_delay_cnt : unsigned(3 downto 0);
        signal reset_s         : std_ulogic;
    begin
        reset_s <= not reset_n_i;

        iodelay_m : IODELAY2
        generic map(
            DATA_RATE          => "SDR",
            IDELAY_VALUE       => 0,
            IDELAY2_VALUE      => 0,
            IDELAY_MODE        => "NORMAL",
            ODELAY_VALUE       => 0,
            IDELAY_TYPE        => "DIFF_PHASE_DETECTOR",
            COUNTER_WRAPAROUND => "STAY_AT_LIMIT",
            DELAY_SRC          => "IDATAIN",
            SERDES_MODE        => "MASTER",
            SIM_TAPDELAY_VALUE => 49
        )
        port map(
            IDATAIN  => pad_i(i),
            TOUT     => open,
            DOUT     => delay_m_dout,
            T        => '1',
            ODATAIN  => '0',
            DATAOUT  => delay_m_dataout,
            DATAOUT2 => open,
            IOCLK0   => bit_clock_i,
            IOCLK1   => '0',
            CLK      => gearbox_clock_i,
            CAL      => delay_cal,
            INC      => delay_inc,
            CE       => delay_ce,
            RST      => delay_rst,
            BUSY     => delay_busy
        );

        iodelay_s : IODELAY2
        generic map(
            DATA_RATE          => "SDR",
            IDELAY_VALUE       => 0,
            IDELAY2_VALUE      => 0,
            IDELAY_MODE        => "NORMAL",
            ODELAY_VALUE       => 0,
            IDELAY_TYPE        => "DIFF_PHASE_DETECTOR",
            COUNTER_WRAPAROUND => "WRAPAROUND",
            DELAY_SRC          => "IDATAIN",
            SERDES_MODE        => "SLAVE",
            SIM_TAPDELAY_VALUE => 49
        )
        port map(
            IDATAIN  => pad_i(i),
            TOUT     => open,
            DOUT     => delay_s_dout,
            T        => '1',
            ODATAIN  => '0',
            DATAOUT  => delay_s_dataout,
            DATAOUT2 => open,
            IOCLK0   => bit_clock_i,
            IOCLK1   => '0',
            CLK      => gearbox_clock_i,
            CAL      => delay_cal,
            INC      => delay_inc,
            CE       => delay_ce,
            RST      => delay_rst,
            BUSY     => open
        );

        iserdes_m : ISERDES2
        generic map(
            BITSLIP_ENABLE => TRUE,
            DATA_RATE      => "SDR",
            DATA_WIDTH     => 5,
            INTERFACE_TYPE => "RETIMED",
            SERDES_MODE    => "MASTER"
        )
        port map(
            Q1        => parallel_s(3),
            Q2        => parallel_s(2),
            Q3        => parallel_s(1),
            Q4        => parallel_s(0),
            SHIFTOUT  => serdes_cascade,
            VALID     => serdes_valid,
            INCDEC    => serdes_incdec,
            D         => delay_m_dataout,
            CE0       => '1',
            CLK0      => bit_clock_i,
            CLK1      => '0',
            CLKDIV    => gearbox_clock_i,
            BITSLIP   => serdes_bitslip,
            IOCE      => serdes_strobe_i,
            RST       => reset_s,
            SHIFTIN   => delay_s_dout,
            CFB0      => open,
            CFB1      => open,
            DFB       => open,
            FABRICOUT => open
        );

        iserdes_s : ISERDES2
        generic map(
            BITSLIP_ENABLE => TRUE,
            DATA_RATE      => "SDR",
            DATA_WIDTH     => 5,
            INTERFACE_TYPE => "RETIMED",
            SERDES_MODE    => "SLAVE"
        )
        port map(
            Q1        => open,
            Q2        => open,
            Q3        => open,
            Q4        => parallel_s(4),
            SHIFTOUT  => open,
            VALID     => open,
            INCDEC    => open,
            D         => delay_s_dataout,
            CE0       => '1',
            CLK0      => bit_clock_i,
            CLK1      => '0',
            CLKDIV    => gearbox_clock_i,
            BITSLIP   => serdes_bitslip,
            IOCE      => serdes_strobe_i,
            RST       => reset_s,
            SHIFTIN   => serdes_cascade,
            CFB0      => open,
            CFB1      => open,
            DFB       => open,
            FABRICOUT => open);

        pd_filter : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    pdcounter <= to_unsigned(16, 5);
                    ce_p      <= '0';
                    inc_p     <= '0';
                    pd_edge   <= '0';
                else
                    ce_p  <= '0';
                    inc_p <= '0';
                    if enable_cal = '1' and serdes_valid = '1' then
                        pd_edge <= '1';
                        if serdes_incdec = '1' and pdcounter /= 31 then
                            pdcounter <= pdcounter + 1;
                        elsif serdes_incdec = '0' and pdcounter /= 0 then
                            pdcounter <= pdcounter - 1;
                        end if;
                        if pdcounter = 31 then
                            ce_p      <= '1';
                            inc_p     <= '1';
                            pdcounter <= to_unsigned(16, 5);
                        elsif pdcounter = 0 then
                            ce_p      <= '1';
                            inc_p     <= '0';
                            pdcounter <= to_unsigned(16, 5);
                        end if;
                    end if;
                end if;
            end if;
        end process;

        calibration_fsm : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    cal_state   <= (others => '0');
                    cal_counter <= (others => '0');
                    delay_cal   <= '0';
                    delay_ce    <= '0';
                    delay_inc   <= '0';
                    delay_rst   <= '0';
                    enable_cal  <= '0';
                else
                    delay_cal <= '0';
                    delay_ce  <= '0';
                    delay_inc <= '0';
                    delay_rst <= '0';
                    case to_integer(cal_state) is
                        when 0 =>
                            cal_counter <= (others => '0');
                            cal_state   <= cal_state + 1;
                        when 1 =>
                            delay_cal <= '1';
                            cal_state <= cal_state + 1;
                        when 2 =>
                            if delay_busy = '1' then
                                cal_state <= cal_state + 1;
                            end if;
                        when 3 =>
                            if delay_busy = '0' then
                                delay_rst <= '1';
                                cal_state <= cal_state + 1;
                            end if;
                        when 4 =>
                            enable_cal  <= '1';
                            cal_counter <= cal_counter + 1;
                            delay_ce    <= ce_p;
                            delay_inc   <= inc_p;
                            if cal_counter(8) = '1' then
                                cal_counter <= (others => '0');
                                cal_state   <= cal_state + 1;
                            end if;
                        when 5 =>
                            enable_cal <= '0';
                            delay_cal  <= '1';
                            cal_state  <= cal_state + 1;
                        when 6 =>
                            if delay_busy = '1' then
                                cal_state <= cal_state + 1;
                            end if;
                        when 7 =>
                            if delay_busy = '0' then
                                cal_state <= "100";
                            end if;
                        when others =>
                            cal_state <= (others => '0');
                    end case;
                end if;
            end if;
        end process;

        slip_tracker : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    slip_count <= 9;
                elsif serdes_bitslip = '1' then
                    if slip_count = 0 then
                        slip_count <= 9;
                    else
                        slip_count <= slip_count - 1;
                    end if;
                end if;
            end if;
        end process;
        mark_s <= '1' when slip_count = 0 else
                  '0';
        invert_s <= slip_count >= 5;

        gearbox_proc : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' then
                    high_nlow_s   <= true;
                    old_mark_s    <= mark_s;
                    gearbox_reg_s <= (others => '0');
                    d             <= (others => '0');
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

        word_sync_proc : process (clock_i)
        begin
            if rising_edge(clock_i) then
                d_word_sync <= d;
            end if;
        end process;

        output_proc : process (d_word_sync)
        begin
            if not left_to_right_c then
                lane_o(i) <= d_word_sync;
            else
                for j in 0 to 9 loop
                    lane_o(i)(9 - j) <= d_word_sync(j);
                end loop;
            end if;
        end process;

        alignment_fsm : process (gearbox_clock_i)
        begin
            if rising_edge(gearbox_clock_i) then
                if reset_n_i = '0' or align_restart_i = '1' then
                    align_state      <= ALIGNING;
                    align_delay_cnt  <= (others => '0');
                    serdes_bitslip   <= '0';
                    align_ready_o(i) <= '0';
                else
                    serdes_bitslip <= '0';
                    case align_state is
                        when ALIGNING =>
                            align_ready_o(i) <= '0';
                            if align_valid_i(i) = '1' then
                                align_state <= ALIGNED;
                            elsif align_delay_cnt = 0 then
                                serdes_bitslip  <= '1';
                                align_delay_cnt <= to_unsigned(7, 4);
                            else
                                align_delay_cnt <= align_delay_cnt - 1;
                            end if;
                        when ALIGNED =>
                            align_ready_o(i) <= '1';
                    end case;
                end if;
            end if;
        end process;
    end generate;
end architecture;
