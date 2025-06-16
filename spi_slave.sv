module spi_slave #(
    parameter int WORD_LENGTH = 8
)(
    // SPI interface
    input  logic sclk,
    input  logic cs_n,
    input  logic mosi,
    output logic miso,
    input  logic rst_n,

    // SafetyBoard interface
    output logic [20:0] spi_requests,
    input  logic [20:0] contactor_status,
    input  logic [41:0] router_feedback,
    input  status_reg_t status,
    input  control_reg_t control,
    output control_reg_t control_out,
    output logic [5:0]  spi_shutdown_cmd,
    input  logic [5:0]  shutdown_status,
    output logic [5:0]  spi_pg_shutdown
);
    import spi_pkg::*;

    // Internal registers
    logic [7:0] cmd_reg;
    logic [7:0] addr_reg;
    logic [7:0] data_reg;
    logic [7:0] shift_reg;
    logic [7:0] checksum_reg;
    logic [7:0] calculated_checksum;
    logic [2:0] bit_counter;
    logic [2:0] word_counter;  // Extended to handle checksum
    logic cmd_received;
    logic addr_received;
    logic data_received;
    logic checksum_valid;
    logic transaction_valid;
    
    // Calculate running checksum (XOR of all bytes)
    always_ff @(posedge sclk or negedge rst_n) begin
        if (!rst_n) begin
            calculated_checksum <= '0;
        end else if (!cs_n) begin
            if (bit_counter == 7) begin
                case (word_counter)
                    3'b000: calculated_checksum <= {shift_reg[6:0], mosi};  // First byte
                    3'b001, 3'b010: calculated_checksum <= calculated_checksum ^ {shift_reg[6:0], mosi};
                    default: ;  // Don't include checksum byte in calculation
                endcase
            end
        end else begin
            calculated_checksum <= '0;
        end
    end

    // Command processing
    always_ff @(posedge sclk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_reg <= '0;
            addr_reg <= '0;
            data_reg <= '0;
            shift_reg <= '0;
            checksum_reg <= '0;
            bit_counter <= '0;
            word_counter <= '0;
            cmd_received <= '0;
            addr_received <= '0;
            data_received <= '0;
            checksum_valid <= '0;
            transaction_valid <= '0;
            spi_requests <= '0;
            control_out <= '0;
            spi_shutdown_cmd <= '0;
            spi_pg_shutdown <= '0;
        end else begin
            logic current_checksum_is_valid; // Moved declaration here
            if (!cs_n) begin
            // Shift register operation
            shift_reg <= {shift_reg[6:0], mosi};
            bit_counter <= bit_counter + 1;

            // Word complete
            if (bit_counter == 7) begin
                case (word_counter)
                    3'b000: begin  // Command byte
                        cmd_reg <= {shift_reg[6:0], mosi};
                        cmd_received <= 1'b1;
                            word_counter <= 3'b001; // Advance to Address state
                    end
                    3'b001: begin  // Address/Index byte
                        addr_reg <= {shift_reg[6:0], mosi};
                        addr_received <= 1'b1;
                            word_counter <= 3'b010; // Advance to Data state
                    end
                    3'b010: begin  // Data byte
                        data_reg <= {shift_reg[6:0], mosi};
                        data_received <= 1'b1;
                            word_counter <= 3'b011; // Advance to Checksum state
                    end
                    3'b011: begin  // Checksum byte
                        checksum_reg <= {shift_reg[6:0], mosi};
                            current_checksum_is_valid = ({shift_reg[6:0], mosi} == calculated_checksum);
                            checksum_valid <= current_checksum_is_valid; // Update status

                            if (current_checksum_is_valid) begin
                                transaction_valid <= 1'b1; // Mark transaction as valid
                                // Process write commands
                            if (cmd_received && addr_received && data_received) begin
                                case (cmd_reg)
                                    CMD_WRITE_CONTACTOR: begin
                                        if (addr_reg < 21) begin
                                            spi_requests[addr_reg] <= data_reg[0];
                                        end
                                    end
                                    CMD_WRITE_CONTROL: begin
                                        control_out.reset_req <= data_reg[0];
                                        control_out.clear_errors <= data_reg[1];
                                        control_out.reserved <= data_reg[7:2];
                                    end
                                    CMD_WRITE_SHUTDOWN: begin
                                        if (addr_reg < 6) begin
                                            spi_shutdown_cmd[addr_reg] <= data_reg[0];
                                        end
                                    end
                                    CMD_WRITE_PG_SHUTDOWN: begin
                                        if (addr_reg < 6) begin
                                            spi_pg_shutdown[addr_reg] <= data_reg[0];
                                        end
                                    end
                                endcase
                            end
                            end else begin
                                transaction_valid <= 1'b0; // Mark transaction as invalid
                            end

                            // Logical transaction complete. Reset for the NEXT logical transaction.
                            word_counter <= 3'b000;
                            cmd_received <= 1'b0;
                            addr_received <= 1'b0;
                            data_received <= 1'b0;
                            // checksum_valid and transaction_valid reflect the just-completed transaction.
                    end
                endcase
                    // bit_counter is reset when CS_N goes high (see below)
            end
            end else begin // CS_N is HIGH (inactive)
                // When CS_N goes high, it signifies the end of the current *byte* transfer.
                // Only reset the bit_counter. word_counter and other transaction state persist.
            bit_counter <= '0;
                // Note: control_out.reset_req is NOT auto-cleared here.
                // It should be cleared by the master via a CMD_WRITE_CONTROL or by system logic.
            end
        end
    end

    // Read data preparation
    logic [7:0] read_data;
    always_comb begin
        case (cmd_reg)
            CMD_VERSION: begin
                read_data = 8'h42; // FPGA version byte
            end
            CMD_READ_CONTACTOR: begin
                read_data = addr_reg < 21 ? {7'b0, contactor_status[addr_reg]} : 8'b0;
            end
            CMD_READ_FEEDBACK: begin
                read_data = addr_reg < 21 ? {6'b0, router_feedback[2*addr_reg +: 2]} : 8'b0;
            end
            CMD_READ_STATUS: begin
                read_data = {status.reserved[3:0], status.thermal_shutdown[1:0], status.invalid_request, status.feedback_timeout_error};
            end
            CMD_READ_SHUTDOWN: begin
                read_data = addr_reg < 6 ? {7'b0, shutdown_status[addr_reg]} : 8'b0;
            end
            CMD_READ_CONTROL: begin
                // Pack control register fields into a byte (customize as needed)
                read_data = {control.reserved[3:0], control.clear_errors, control.reset_req, 2'b0};
            end
            default: read_data = 8'b0;
        endcase
    end

    logic miso_internal;
    logic next_miso_bit_value;

    localparam DEFAULT_MISO_CMD_PHASE_WORD = 8'hA5; // Default MISO word for command phase. Change as needed.

    // Combinational logic to determine the data bit to be shifted out
    always_comb begin
        // Default to 0. This value is used if none of the cases match,
        // or for phases where MISO doesn't actively drive data from read_data/checksum.
        next_miso_bit_value = DEFAULT_MISO_CMD_PHASE_WORD[7];

        case (word_counter)
            3'b000: next_miso_bit_value = DEFAULT_MISO_CMD_PHASE_WORD[7-bit_counter];
            3'b001: next_miso_bit_value = 1'b0;  // Address phase (master writes, slave reads)
            3'b010: begin  // Data phase (slave writes, master reads)
                next_miso_bit_value = read_data[7-bit_counter];
            end
            3'b011: begin  // Checksum phase (slave writes, master reads)
                logic [7:0] checksum_val; // Temporary variable for checksum calculation
                checksum_val = cmd_reg ^ addr_reg ^ read_data;
                next_miso_bit_value = checksum_val[7-bit_counter];
            end
            default: next_miso_bit_value = DEFAULT_MISO_CMD_PHASE_WORD[7];
        endcase
    end

    // Register the MISO bit on the falling edge of SCLK
    always_ff @(negedge sclk or negedge rst_n) begin
        if (!rst_n) begin
            miso_internal <= 1'b0;
        end else begin
            // miso_internal latches the value that should be driven on MISO.
            // This happens on the falling edge of sclk.
            miso_internal <= next_miso_bit_value;
        end
    end

    // Final MISO output assignment
    // MISO is driven by the registered miso_internal when cs_n is low (active).
    // Otherwise, MISO is driven low.
    assign miso = (!cs_n) ? miso_internal : 1'b0;

endmodule
