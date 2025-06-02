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
        end else if (!cs_n) begin
            // Shift register operation
            shift_reg <= {shift_reg[6:0], mosi};
            bit_counter <= bit_counter + 1;

            // Word complete
            if (bit_counter == 7) begin
                case (word_counter)
                    3'b000: begin  // Command byte
                        cmd_reg <= {shift_reg[6:0], mosi};
                        cmd_received <= 1'b1;
                        word_counter <= 3'b001;
                    end
                    3'b001: begin  // Address/Index byte
                        addr_reg <= {shift_reg[6:0], mosi};
                        addr_received <= 1'b1;
                        word_counter <= 3'b010;
                    end
                    3'b010: begin  // Data byte
                        data_reg <= {shift_reg[6:0], mosi};
                        data_received <= 1'b1;
                        word_counter <= 3'b011;
                    end
                    3'b011: begin  // Checksum byte
                        checksum_reg <= {shift_reg[6:0], mosi};
                        checksum_valid <= ({shift_reg[6:0], mosi} == calculated_checksum);
                        word_counter <= 3'b100;
                        
                        // Process write commands only if checksum matches
                        if ({shift_reg[6:0], mosi} == calculated_checksum) begin
                            transaction_valid <= 1'b1;
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
                        end
                    end
                endcase
            end
        end else begin
            // Reset for next transaction
            bit_counter <= '0;
            word_counter <= '0;
            cmd_received <= '0;
            addr_received <= '0;
            data_received <= '0;
            checksum_valid <= '0;
            transaction_valid <= '0;
            control_out.reset_req <= 1'b0;  // Auto-clear reset request
        end
    end

    // Read data preparation
    logic [7:0] read_data;
    always_comb begin
        case (cmd_reg)
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

    // MISO output logic with checksum
    always_comb begin
        if (cs_n) begin
            miso = 1'b0;
        end else begin
            case (word_counter)
                3'b000: miso = 1'b0;  // Command phase (write-only)
                3'b001: miso = 1'b0;  // Address phase (write-only)
                3'b010: begin  // Data phase
                    miso = read_data[7-bit_counter];
                end
                3'b011: begin  // Checksum phase
                    // For reads, checksum is XOR of command, address, and read_data
                    logic [7:0] read_checksum;
                    read_checksum = cmd_reg ^ addr_reg ^ read_data;
                    miso = read_checksum[7-bit_counter];
                end
                default: miso = 1'b0;
            endcase
        end
    end

endmodule
