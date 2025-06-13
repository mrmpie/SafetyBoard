interface SafetyBoardInterface;
    // Request and command signals - 21 bits total:
    // [5:0]   - Output connection requests (diagonal of the matrix)
    // [20:6]  - Interlink requests in order of:
    //           - 5 bits for distance 1 (PG0-PG1, PG1-PG2, PG2-PG3, PG3-PG4, PG4-PG5)
    //           - 4 bits for distance 2 (PG0-PG2, PG1-PG3, PG2-PG4, PG3-PG5)
    //           - 3 bits for distance 3 (PG0-PG3, PG1-PG4, PG2-PG5)
    //           - 2 bits for distance 4 (PG0-PG4, PG1-PG5)
    //           - 1 bit  for distance 5 (PG0-PG5)
    logic [20:0] requests;          // Connection requests
    logic [20:0] router_cmd;        // Command to power routers (same for plus and minus)
    
    // Feedback from power routers - 2 feedback signals per connection
    logic [41:0] router_feedback;   // 2 feedback bits per router command bit
    
    // Control signals
    logic clk;
    logic rst_n;
    logic keep_alive;               // Keep-alive signal, toggles on state changes

    // SPI signals
    logic spi_mosi;
    logic spi_miso;
    logic spi_sclk;
    logic spi_cs_n;
    
    // Status signals
    logic invalid_request;          // Indicates unsafe interlink request
    logic feedback_timeout_error;   // Indicates feedback timeout occurred
    
    // Removed debug and shutdown signals for minimal build
    
    modport SafetyBoard (
        input  clk, rst_n, requests, router_feedback, spi_mosi, spi_sclk, spi_cs_n,
        output router_cmd, invalid_request, feedback_timeout_error, spi_miso, keep_alive
    );

    modport Test (
        output clk, rst_n, requests, router_feedback, spi_mosi, spi_sclk, spi_cs_n,
        input  router_cmd, invalid_request, feedback_timeout_error, spi_miso, keep_alive
    );
endinterface

module SafetyBoard #(
    parameter int FEEDBACK_TIMEOUT_CYCLES = 100
)(
    SafetyBoardInterface sif
);
    import spi_pkg::*;

    // State machine states
    typedef enum logic [2:0] {
        IDLE          = 3'b000,
        BUILD_MATRIX  = 3'b001,
        VALIDATE_REQ  = 3'b010,
        WAIT_FEEDBACK = 3'b011,
        FEEDBACK_ERROR = 3'b100,
        COMMAND_READY  = 3'b101
    } state_t;
    
    // Request validation states
    typedef enum logic [2:0] {
        INIT_CHECK     = 3'b000,
        CHECK_OUTPUTS  = 3'b001,
        FWD_CHECK      = 3'b010,
        BWD_CHECK      = 3'b011,
        CHECK_COMPLETE = 3'b100
    } validate_state_t;
    
    // Internal state tracking
    state_t current_state;
    validate_state_t validate_state;
    state_t prev_state; // Track previous state for keep_alive toggling
    logic [20:0] contactor_commanded;
    logic [5:0][5:0] request_matrix;
    
    // Validation state tracking
    logic [2:0] check_i, check_j;
    logic validation_invalid;
    
    // Feedback tracking
    logic [$clog2(FEEDBACK_TIMEOUT_CYCLES):0] feedback_timeout_counter;
    logic [20:0] pending_command;
    logic [20:0] captured_requests;  // Store requests at start of validation
    

    // Array to store output assignments for each PG (0 = no output, 1-6 = connected to that output)
    int pg_output_assignments[6];      // Used for validating new requests
    int pg_output_assignments_current[6];  // Used for shutdown logic, reflects current valid state

    // Internal signals for SPI interface
    logic [20:0] spi_requests;
    logic [5:0]  spi_shutdown_cmd;
    logic [5:0]  spi_pg_shutdown;
    control_reg_t spi_control;
    control_reg_t control_reg;
    status_reg_t  status;

    // Status register assignment
    assign status.feedback_timeout_error = sif.feedback_timeout_error;
    assign status.invalid_request = sif.invalid_request;
    assign status.reserved = '0;

    // Instantiate SPI slave
    spi_slave #(
        .WORD_LENGTH(`SPI_WORD_LENGTH)
    ) spi_inst (
        .sclk(sif.spi_sclk),
        .cs_n(sif.spi_cs_n),
        .mosi(sif.spi_mosi),
        .miso(sif.spi_miso),
        .rst_n(combined_rst_n),
        .spi_requests(spi_requests),
        .contactor_status(contactor_commanded),
        .router_feedback(sif.router_feedback),
        .status(status),
        .control(control_reg),
        .control_out(spi_control),
        .spi_shutdown_cmd(spi_shutdown_cmd),
//        .shutdown_status(sif.shutdown_commands),
//        .spi_pg_shutdown(spi_pg_shutdown)
    );

    // Combined reset signal
    logic combined_rst_n;
    assign combined_rst_n = sif.rst_n & ~spi_control.reset_req;  // Combined reset: active-low if hardware reset (sif.rst_n low) OR SPI reset (spi_control.reset_req high)

    // Helper function to check and propagate output assignments between two PGs
    function automatic logic check_pg_connection(int pg1, int pg2);
        logic is_invalid = 1'b0;
        
        if (pg_output_assignments[pg1] != 0 && pg_output_assignments[pg2] != 0) begin
            // Both PGs have outputs assigned
            if (pg_output_assignments[pg1] != pg_output_assignments[pg2]) begin
                is_invalid = 1'b1;  // Different outputs - invalid!
            end
        end 
        else if (pg_output_assignments[pg1] != 0 && pg_output_assignments[pg2] == 0) begin
            pg_output_assignments[pg2] = pg_output_assignments[pg1];
        end
        else if (pg_output_assignments[pg2] != 0 && pg_output_assignments[pg1] == 0) begin
            pg_output_assignments[pg1] = pg_output_assignments[pg2];
        end
        
        return is_invalid;
    endfunction

    function automatic logic check_invalid_request();
        logic invalid = 1'b0;
        
        // Initialize array
        for (int i = 0; i < 6; i++) begin
            pg_output_assignments[i] = 0;
            if (request_matrix[i][i]) begin
                pg_output_assignments[i] = i + 1;
            end
        end
        
        // Forward pass: Check upper triangular part
        for (int i = 0; i < 5 && !invalid; i++) begin
            for (int j = i + 1; j < 6 && !invalid; j++) begin
                if (request_matrix[i][j]) begin
                    invalid = check_pg_connection(i, j);
                end
            end
        end
        
        // Backward pass: Check upper triangular part in reverse
        for (int i = 4; i >= 0 && !invalid; i--) begin
            for (int j = 5; j > i && !invalid; j--) begin
                if (request_matrix[i][j]) begin
                    invalid = check_pg_connection(i, j);
                end
            end
        end
        
        return invalid;
    endfunction

    // Helper function to set symmetric matrix entries
    function automatic void set_matrix_entry(int i, int j, logic value);
        request_matrix[i][j] = value;
        request_matrix[j][i] = value;  // Mirror entry
    endfunction
    
    // Helper function to calculate interlink index
    function automatic int get_interlink_idx(int i, int j);
        // Calculate the distance between i and j
        int distance = j - i;
        // Calculate starting index for this distance group
        // distance 1 starts at 6, distance 2 at 11, distance 3 at 15, distance 4 at 18, distance 5 at 20
        int start_idx = 6;
        for (int d = 1; d < distance; d++) begin
            start_idx += (6 - d);
        end
        // Add offset within this distance group
        return start_idx + i;
    endfunction

    // Build request matrix from individual requests
    function automatic void build_request_matrix();
        // Clear matrix first
        request_matrix = '0;
        
        // Set diagonal (output requests) with feedback OR
        for (int i = 0; i < 6; i++) begin
            request_matrix[i][i] = captured_requests[i] | (sif.router_feedback[2*i +: 2] != 2'b00);
        end
        
        // Set interlink requests using request bits and include feedback
        for (int i = 0; i < 6; i++) begin
            for (int j = i + 1; j < 6; j++) begin
                int idx = get_interlink_idx(i, j);
                // Set matrix entries with feedback OR
                set_matrix_entry(i, j, captured_requests[idx] | (sif.router_feedback[2*idx +: 2] != 2'b00));
            end
        end
    endfunction

        // Assign current states to interface for debug
    // assign sif.current_state = current_state;
    // assign sif.validate_state = validate_state;

    // Helper function to check feedback for a specific connection
    function automatic logic check_connection_feedback(int index);
        return (sif.router_feedback[2*index +: 2] == {contactor_commanded[index], contactor_commanded[index]});
    endfunction

    // Helper function to check all feedback matches commands
    function automatic logic check_all_feedback();
        logic all_match = 1'b1;
        for (int i = 0; i < 21; i++) begin
            if (!check_connection_feedback(i)) begin
                all_match = 1'b0;
                break;
            end
        end
        return all_match;
    endfunction
    // Combinatorial logic for shutdown mapping using the pg_output_assignments
    // always_comb begin
    //     sif.pg_shutdown = '0;
    //     // For each PG
    //     for (int pg = 0; pg < 6; pg++) begin
    //         if (pg_output_assignments_current[pg] != 0) begin
    //             if (sif.shutdown_commands[pg_output_assignments_current[pg] - 1] || 
    //                 spi_shutdown_cmd[pg_output_assignments_current[pg] - 1]) begin
    //                 sif.pg_shutdown[pg] = 1'b1;
    //             end
    //         end
    //     end
    // end

        // Add direct PG shutdown from SPI
    //    sif.pg_shutdown = sif.pg_shutdown | spi_pg_shutdown;
    //end

    // Main control logic
    always_ff @(posedge sif.clk or negedge combined_rst_n) begin
        if (!combined_rst_n) begin
            contactor_commanded <= '0;
            sif.invalid_request <= '0;
            sif.feedback_timeout_error <= '0;
            feedback_timeout_counter <= '0;
            for (int i = 0; i < 6; i++) begin
                pg_output_assignments[i] = 0;
                pg_output_assignments_current[i] = 0;
            end
            check_i <= '0;
            check_j <= '0;
            validation_invalid <= '0;
            captured_requests <= '0;
            pending_command <= '0;
            request_matrix <= '0;
            current_state <= IDLE;
            validate_state <= INIT_CHECK;
            control_reg <= '0;
            sif.keep_alive <= 0;
            prev_state <= IDLE;
        end else begin
            // Toggle keep_alive on state change
            if (current_state != prev_state) begin
                sif.keep_alive <= ~sif.keep_alive;
                prev_state <= current_state;
            end
            // Clear errors if requested through SPI
            if (spi_control.clear_errors) begin
                sif.feedback_timeout_error <= '0;
                sif.invalid_request <= '0;
            end

            case (current_state)
                IDLE: begin
                    // Capture requests from both direct and SPI interfaces
                    captured_requests <= sif.requests | spi_requests;
                    current_state <= BUILD_MATRIX;
                end

                BUILD_MATRIX: begin
                    // Build matrix using captured requests
                    build_request_matrix();
                    
                    // Initialize validation state
                    validation_invalid <= 1'b0;
                    validate_state <= INIT_CHECK;
                    check_i <= '0;
                    check_j <= '0;
                    current_state <= VALIDATE_REQ;
                end

                VALIDATE_REQ: begin
                    case (validate_state)
                        INIT_CHECK: begin
                            // Initialize array using request_matrix which includes feedback
                            for (int i = 0; i < 6; i++) begin
                                pg_output_assignments[i] = 0;
                                if (request_matrix[i][i]) begin  // request_matrix already includes feedback
                                    pg_output_assignments[i] = i + 1;
                                end
                            end
                            validate_state <= CHECK_OUTPUTS;
                        end

                        CHECK_OUTPUTS: begin
                            // Verify no conflicts in direct output assignments
                            if (check_i < 6) begin
                                if (check_j < 6) begin
                                    // Check for output assignment conflicts including stuck contactors
                                    if (check_i != check_j && 
                                        pg_output_assignments[check_i] != 0 && 
                                        pg_output_assignments[check_j] != 0 &&
                                        pg_output_assignments[check_i] == pg_output_assignments[check_j]) begin
                                        validation_invalid <= 1'b1;
                                    end
                                    check_j <= check_j + 1;
                                end else begin
                                    check_j <= '0;
                                    check_i <= check_i + 1;
                                end
                            end else begin
                                check_i <= '0;
                                check_j <= '0;
                                validate_state <= FWD_CHECK;
                            end
                        end

                        FWD_CHECK: begin
                            // Forward pass: Check upper triangular part
                            if (check_i < 5 && !validation_invalid) begin
                                if (check_j <= 5) begin
                                    if (check_j > check_i && request_matrix[check_i][check_j]) begin
                                        validation_invalid <= check_pg_connection(check_i, check_j);
                                    end
                                    check_j <= check_j + 1;
                                end else begin
                                    check_j <= check_i + 2;
                                    check_i <= check_i + 1;
                                end
                            end else begin
                                check_i <= 4;
                                check_j <= 5;
                                validate_state <= BWD_CHECK;
                            end
                        end

                        BWD_CHECK: begin
                            // Backward pass: Check upper triangular part in reverse
                            if (check_i >= 0 && check_i < 5 && !validation_invalid) begin
                                if (check_j > check_i) begin
                                    if (request_matrix[check_i][check_j]) begin
                                        validation_invalid <= check_pg_connection(check_i, check_j);
                                    end
                                    check_j <= check_j - 1;
                                end else begin
                                    check_j <= 5;
                                    check_i <= check_i - 1;
                                end
                            end else begin
                                validate_state <= CHECK_COMPLETE;
                            end
                        end

                        CHECK_COMPLETE: begin
                            sif.invalid_request <= validation_invalid;
                            
                            if (!validation_invalid) begin
                                // Use captured requests for command
                                pending_command = captured_requests;
                                contactor_commanded <= pending_command;
                            end
                            
                            // Move to feedback wait state
                            feedback_timeout_counter <= '0;
                            current_state <= WAIT_FEEDBACK;
                        end
                    endcase
                end

                WAIT_FEEDBACK: begin
                    logic feedback_match;
                    feedback_match = check_all_feedback();
                    
                    // Always check for feedback match first
                    if (feedback_match) begin
                        current_state <= COMMAND_READY;
                        feedback_timeout_counter <= '0;
                        sif.feedback_timeout_error <= 1'b0; // Clear timeout error if feedback matches
                    end else begin
                        // Check for timeout
                        if (feedback_timeout_counter >= FEEDBACK_TIMEOUT_CYCLES) begin
                            sif.feedback_timeout_error <= 1'b1;
                            current_state <= FEEDBACK_ERROR;
                        end else begin
                            feedback_timeout_counter <= feedback_timeout_counter + 1;
                        end
                    end
                end

                FEEDBACK_ERROR: begin
                    // Just transition to COMMAND_READY, feedback timeout is handled in WAIT_FEEDBACK
                    current_state <= COMMAND_READY;
                end

                COMMAND_READY: begin
                    // Copy pg_output_assignments to current since this configuration is now valid
                    for (int i = 0; i < 6; i++) begin
                        pg_output_assignments_current[i] = pg_output_assignments[i];
                    end
                    current_state <= IDLE;
                end

                default: current_state <= IDLE;
            endcase
        end
    end

    // Output assignment
    always_comb begin
        sif.router_cmd = contactor_commanded;
    end

endmodule

// Basic testbench
module SafetyBoard_tb;
    import spi_pkg::*;

    // Clock generation - 250ns period (4MHz)
    localparam CLK_PERIOD = 250;
    
    // Interface instance
    SafetyBoardInterface sif();
    
    // Clock generation
    initial begin
        sif.clk = 0;
        forever #(CLK_PERIOD/2) sif.clk = ~sif.clk;
    end
    
    // SPI clock generation (1MHz)
    localparam SPI_CLK_PERIOD = 1000;
    initial begin
        sif.spi_sclk = 0;
        forever #(SPI_CLK_PERIOD/2) sif.spi_sclk = ~sif.spi_sclk;
    end
    
    // DUT instance
    SafetyBoard dut(sif.SafetyBoard);
    
    // Test stimulus
    initial begin
        // Initialize signals
        sif.rst_n = 0;
        sif.requests = '0;
        sif.router_feedback = '0;
        sif.shutdown_commands = '0;
        sif.spi_cs_n = 1;
        sif.spi_mosi = 0;
        
        // Reset
        #(CLK_PERIOD*10);
        sif.rst_n = 1;
        #(CLK_PERIOD*10);
        
        // Test 1: Basic Connection Test
        $display("\nTest 1: Basic Connection Test");
        sif.requests[0] = 1'b1;  // Request PG0 output
        #(CLK_PERIOD*10);
        
        // Add feedback
        sif.router_feedback[1:0] = 2'b11;  // Feedback for PG0
        #(CLK_PERIOD*10);
        
        // Test 2: SPI Connection Request
        $display("\nTest 2: SPI Connection Request");
        spi_write(CMD_WRITE_CONTACTOR, 21'h000002);  // Request PG1 output
        #(CLK_PERIOD*10);
        
        // Add feedback
        sif.router_feedback[3:2] = 2'b11;  // Feedback for PG1
        #(CLK_PERIOD*10);
        
        // Test 3: Read Status Register
        $display("\nTest 3: Read Status Register");
        spi_read(CMD_READ_STATUS);
        #(CLK_PERIOD*10);
        
        // Test 4: Shutdown Command Test
        $display("\nTest 4: Shutdown Command Test");
        sif.shutdown_commands[0] = 1'b1;  // Direct shutdown for PG0
        #(CLK_PERIOD*10);
        
        // Test 5: SPI Shutdown Command
        $display("\nTest 5: SPI Shutdown Command");
        spi_write(CMD_WRITE_SHUTDOWN, {26'b0, 6'h02});  // SPI shutdown for PG1
        #(CLK_PERIOD*10);
        
        // Test 6: Read Shutdown Status
        $display("\nTest 6: Read Shutdown Status");
        spi_read(CMD_READ_SHUTDOWN);
        #(CLK_PERIOD*10);
        
        // Test 7: Error Injection and Clearing
        $display("\nTest 7: Error Injection and Clearing");
        // Force a feedback timeout
        sif.router_feedback = '0;
        #(CLK_PERIOD*200);  // Wait for timeout
        
        // Read status
        spi_read(CMD_READ_STATUS);
        #(CLK_PERIOD*10);
        
        // Clear errors via SPI
        spi_write(CMD_WRITE_CONTROL, {24'b0, 8'h01});  // Set clear_errors bit
        #(CLK_PERIOD*10);
        
        // Read status again
        spi_read(CMD_READ_STATUS);
        #(CLK_PERIOD*10);
        
        $finish;
    end
    
    // Task to perform SPI write
    task automatic spi_write(input logic [7:0] cmd, input logic [31:0] data);
        logic [31:0] word = {data[30:0], 1'b0};  // Shift data left by 1 for first bit
        
        // Start transaction
        @(posedge sif.spi_sclk);
        sif.spi_cs_n = 0;
        
        // Send command byte
        for (int i = 0; i < 8; i++) begin
            @(negedge sif.spi_sclk);
            sif.spi_mosi = cmd[7-i];
        end
        
        // Send data
        for (int i = 0; i < `SPI_WORD_LENGTH-8; i++) begin
            @(negedge sif.spi_sclk);
            sif.spi_mosi = word[(`SPI_WORD_LENGTH-9)-i];
        end
        
        // End transaction
        @(posedge sif.spi_sclk);
        sif.spi_cs_n = 1;
    endtask
    
    // Task to perform SPI read
    task automatic spi_read(input logic [7:0] cmd);
        logic [31:0] read_data;
        
        // Start transaction
        @(posedge sif.spi_sclk);
        sif.spi_cs_n = 0;
        
        // Send command byte
        for (int i = 0; i < 8; i++) begin
            @(negedge sif.spi_sclk);
            sif.spi_mosi = cmd[7-i];
        end
        
        // Read data
        for (int i = 0; i < `SPI_WORD_LENGTH-8; i++) begin
            @(posedge sif.spi_sclk);
            read_data[(`SPI_WORD_LENGTH-9)-i] = sif.spi_miso;
        end
        
        // End transaction
        @(posedge sif.spi_sclk);
        sif.spi_cs_n = 1;
        
        // Display read data
        $display("SPI Read: cmd=0x%h, data=0x%h", cmd, read_data);
    endtask
    
    // Monitor for displaying state changes
    always @(sif.current_state) begin
        $display("\nState Change at time %0t:", $time);
        $display("Current State: %s", 
            sif.current_state == 3'b000 ? "IDLE" :
            sif.current_state == 3'b001 ? "BUILD_MATRIX" :
            sif.current_state == 3'b010 ? "VALIDATE_REQ" :
            sif.current_state == 3'b011 ? "WAIT_FEEDBACK" :
            sif.current_state == 3'b100 ? "FEEDBACK_ERROR" :
            sif.current_state == 3'b101 ? "COMMAND_READY" : "UNKNOWN");
        $display("Router Command: %b", sif.router_cmd);
        $display("Invalid Request: %b", sif.invalid_request);
        $display("Feedback Timeout Error: %b", sif.feedback_timeout_error);
        $display("PG Shutdown Status: %b", sif.pg_shutdown);
    end

endmodule
