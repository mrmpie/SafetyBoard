-- File: SafetyBoard.vhd
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package safety_pkg is
    -- Router type definition
    type router_type_t is (FULLFLEX, HIGHFLEX);
    
    -- Contactor states
    type contactor_state_t is (OPEN_STATE, CLOSED_STATE, ERROR_STATE);
    
    -- Constants
    constant MAX_CONTACTORS : integer := 21;
    constant NUM_POWER_UNITS : integer := 6;
end package;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.safety_pkg.all;

entity SafetyBoard is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        router_type : in router_type_t;
        
        -- Commands to power routers
        plus_router_cmd : out std_logic_vector(20 downto 0);
        minus_router_cmd : out std_logic_vector(11 downto 0);
        
        -- Feedback from power routers
        router_feedback : in std_logic_vector(41 downto 0);
        
        -- Control signals
        power_unit_valid : in std_logic_vector(5 downto 0);
        output_request : in std_logic_vector(5 downto 0)
    );
end entity;

architecture rtl of SafetyBoard is
    -- Internal state tracking
    signal output_connected : std_logic_vector(5 downto 0);
    signal unit_connected : std_logic_vector(5 downto 0);
    signal contactor_state : std_logic_vector(20 downto 0);
    
    -- Connection matrices
    type matrix_t is array (0 to 5) of std_logic_vector(5 downto 0);
    signal connection_matrix : matrix_t;
    signal output_matrix : matrix_t;
    
    -- Error flags
    signal multiple_output_error : std_logic;
    signal invalid_connection_error : std_logic;
    signal feedback_mismatch_error : std_logic;
    
    -- Number of active contactors
    signal num_contactors : integer range 0 to MAX_CONTACTORS;

    -- Function to get interlink index
    function get_interlink_idx(unit1, unit2 : integer; r_type : router_type_t) return integer is
    begin
        if r_type = FULLFLEX then
            return (unit1 * 5 + unit2 - 1);
        else
            if ((unit2 = unit1 + 1) or (unit1 = 5 and unit2 = 0)) then
                return unit1;
            else
                return -1;  -- Invalid connection for HighFlex
            end if;
        end if;
    end function;

    -- Function to get output index
    function get_output_idx(unit_num : integer; r_type : router_type_t) return integer is
    begin
        if r_type = FULLFLEX then
            return 15 + unit_num;  -- Output contactors start after interlinks
        else
            return 6 + unit_num;   -- Output contactors start after interlinks in HighFlex
        end if;
    end function;

    -- Function to validate connection request
    function is_valid_connection(
        from_unit, to_out : integer;
        r_type : router_type_t;
        unit_conn, out_conn : std_logic_vector(5 downto 0)
    ) return boolean is
        variable valid : boolean := true;
        variable has_valid_path : boolean := false;
        variable current_units : std_logic_vector(5 downto 0);
        variable next_units : std_logic_vector(5 downto 0);
    begin
        -- Check if unit or output already connected
        if unit_conn(from_unit) = '1' or out_conn(to_out) = '1' then
            return false;
        end if;
        
        -- For HighFlex, verify ring topology rules
        if r_type = HIGHFLEX then
            current_units := (others => '0');
            current_units(from_unit) := '1';
            
            -- Traverse possible paths
            for i in 0 to 5 loop
                next_units := (others => '0');
                for u in 0 to 5 loop
                    if current_units(u) = '1' then
                        -- Check adjacent units
                        if u = 0 then
                            next_units(5) := '1';
                        else
                            next_units(u-1) := '1';
                        end if;
                        if u = 5 then
                            next_units(0) := '1';
                        else
                            next_units(u+1) := '1';
                        end if;
                    end if;
                end loop;
                current_units := next_units;
                if current_units(to_out) = '1' then
                    has_valid_path := true;
                    exit;
                end if;
            end loop;
            valid := has_valid_path;
        end if;
        
        return valid;
    end function;

begin
    -- Set number of contactors based on router type
    num_contactors <= MAX_CONTACTORS when router_type = FULLFLEX else 12;

    -- Main control process
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            output_connected <= (others => '0');
            unit_connected <= (others => '0');
            contactor_state <= (others => '0');
            connection_matrix <= (others => (others => '0'));
            output_matrix <= (others => (others => '0'));
            multiple_output_error <= '0';
            invalid_connection_error <= '0';
            feedback_mismatch_error <= '0';
            
        elsif rising_edge(clk) then
            -- Process connection requests
            for i in 0 to 5 loop
                if output_request(i) = '1' and power_unit_valid(i) = '1' then
                    if is_valid_connection(i, i, router_type, unit_connected, output_connected) then
                        contactor_state(get_output_idx(i, router_type)) <= '1';
                        output_connected(i) <= '1';
                        unit_connected(i) <= '1';
                        output_matrix(i)(i) <= '1';
                    end if;
                end if;
            end loop;

            -- Verify feedback matches commands
            feedback_mismatch_error <= '0';  -- Default state
            
            for i in 0 to MAX_CONTACTORS-1 loop
                if i < num_contactors then
                    if router_feedback(2*i+1 downto 2*i) /= 
                       (contactor_state(i) & contactor_state(i)) then
                        feedback_mismatch_error <= '1';
                        -- Safety response: Open all contactors
                        contactor_state <= (others => '0');
                        output_connected <= (others => '0');
                        unit_connected <= (others => '0');
                        connection_matrix <= (others => (others => '0'));
                        output_matrix <= (others => (others => '0'));
                    end if;
                end if;
            end loop;
        end if;
    end process;

    -- Output assignment process
    process(contactor_state, router_type)
    begin
        if router_type = FULLFLEX then
            plus_router_cmd <= contactor_state;
            minus_router_cmd <= contactor_state(11 downto 0);
        else
            plus_router_cmd <= "000000000" & contactor_state(11 downto 0);
            minus_router_cmd <= contactor_state(11 downto 0);
        end if;
    end process;

end architecture;