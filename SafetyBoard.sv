package safety_pkg;
    typedef enum logic [1:0] {
        OPEN = 2'b00,
        CLOSED = 2'b01,
        ERROR = 2'b11
    } contactor_state_t;
endpackage

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
    
    // Status signals
    logic invalid_request;          // Indicates unsafe interlink request
    logic feedback_timeout_error;   // Indicates feedback timeout occurred
    
    // Debug signals
    logic [2:0] current_state;      // Current state of the state machine
    logic [2:0] validate_state;     // Current state of the validation state machine
    
    // New signals
    logic [5:0] shutdown_commands;  // From outputs to safety board
    logic [5:0] pg_shutdown;        // From safety board to power groups
    
    modport SafetyBoard (
        input  clk, rst_n, requests, router_feedback,
        output router_cmd, invalid_request, feedback_timeout_error, 
               current_state, validate_state,
        input  shutdown_commands,
        output pg_shutdown
    );

    modport Test (
        output clk, rst_n, requests, router_feedback,
        input  router_cmd, invalid_request, feedback_timeout_error,
              current_state, validate_state,
        output shutdown_commands,
        input  pg_shutdown
    );
endinterface

module SafetyBoard #(
    parameter int FEEDBACK_TIMEOUT_CYCLES = 100
)(
    SafetyBoardInterface sif
);
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
    
    // Combinatorial logic for shutdown mapping using the pg_output_assignments
    always_comb begin
        sif.pg_shutdown = '0;
        
        // For each PG
        for (int pg = 0; pg < 6; pg++) begin
            // If this PG is mapped to an output (value 1-6)
            if (pg_output_assignments_current[pg] != 0) begin
                // Check if that output is requesting shutdown
                // Subtract 1 because array is 0-5 but assignments are 1-6
                if (sif.shutdown_commands[pg_output_assignments_current[pg] - 1]) begin
                    sif.pg_shutdown[pg] = 1'b1;
                end
            end
        end
    end

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

    // Helper function to set interlink entries based on distance array
    function automatic void set_interlink_entries(int distance, logic [4:0] req_array);
        int max_idx = 6 - distance;  // Number of connections for this distance
        for (int i = 0; i < max_idx; i++) begin
            set_matrix_entry(i, i + distance, req_array[i]);
        end
    endfunction
    
    // Helper function to get output index
    function automatic int get_output_idx(int group);
        return group;  // Direct mapping for outputs
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
    assign sif.current_state = current_state;
    assign sif.validate_state = validate_state;

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


    // Main control logic
    always_ff @(posedge sif.clk or negedge sif.rst_n) begin
        if (!sif.rst_n) begin
            contactor_commanded <= '0;
            sif.invalid_request <= '0;
            sif.feedback_timeout_error <= '0;
            feedback_timeout_counter <= '0;
            pending_command <= '0;
            request_matrix <= '0;
            current_state <= IDLE;
            validate_state <= INIT_CHECK;
            check_i <= '0;
            check_j <= '0;
            validation_invalid <= '0;
            captured_requests <= '0;
        end else begin
            case (current_state)
                IDLE: begin
                    // Only capture requests in IDLE
                    captured_requests <= sif.requests;
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
    // Clock generation - 250ns period (4MHz)
    logic clk = 0;
    always #125 clk = ~clk;  // 125ns high, 125ns low = 250ns period

    // Interface instance
    SafetyBoardInterface sif();
    
    // DUT instantiation
    SafetyBoard dut (
        .sif(sif.SafetyBoard)
    );

    // Connect clock to interface
    assign sif.clk = clk;

    // Test stimulus
    initial begin
        // Initialize
        sif.rst_n = 0;
        sif.requests = '0;
        sif.router_feedback = '0;
        
        // Hold reset for 1us (4 clock cycles)
        #1000 sif.rst_n = 1;
        
        // Wait a few clocks
        @(posedge clk);
        @(posedge clk);
        
        // Enable default connection for Power Group 1 (index 0)
        sif.requests[0] = 1'b1;  // PG1 = index 0
        
        @(posedge clk);
        
        // Enable default connection for Power Group 2 (index 1)
        sif.requests[1] = 1'b0;  // PG2 = index 1
        
        @(posedge clk);
        
        // Request interlink between PG1 and PG3 (index 0 and 2)
        sif.requests[6] = 1'b1;  // Connect PG1 (0) to PG3 (2)
        
        // Monitor outputs
        @(posedge clk);
        $display("Time=%0t Router_cmd=%b", $time, sif.router_cmd);
        $display("Time=%0t Invalid_request=%b", $time, sif.invalid_request);
        $display("Time=%0t Feedback_timeout=%b", $time, sif.feedback_timeout_error);
        
        // Add feedback - each bit duplicated for the 2-bit feedback
        for (int i = 0; i < 21; i++) begin
            sif.router_feedback[2*i +: 2] = {sif.router_cmd[i], sif.router_cmd[i]};
        end
        
        // Wait a few clocks and check the state
        repeat(5) @(posedge clk);
        
        // Display final state
        $display("\nFinal State:");
        $display("Router_cmd=%b", sif.router_cmd);
        $display("Invalid_request=%b", sif.invalid_request);
        $display("Feedback Timeout Error: %b", sif.feedback_timeout_error);
        
        // Run for a few more microseconds
        #5000;
        
        // End simulation
        $finish;
    end

    // Optional: Waveform dumping
    initial begin
        $dumpfile("SafetyBoard.vcd");
        $dumpvars(0, SafetyBoard_tb);
    end

    // Helper task to display the current state
    task display_state;
        $display("\nCurrent State at time %0t:", $time);
        $display("State: %s", 
            sif.current_state == 3'b000 ? "IDLE" :
            sif.current_state == 3'b001 ? "BUILD_MATRIX" :
            sif.current_state == 3'b010 ? "VALIDATE_REQ" :
            sif.current_state == 3'b011 ? "WAIT_FEEDBACK" :
            sif.current_state == 3'b100 ? "FEEDBACK_ERROR" :
            sif.current_state == 3'b101 ? "COMMAND_READY" : "UNKNOWN"
        );
        $display("Default Connections: %b", sif.requests[5:0]);
        $display("Interlink Request [0][1]: %b", sif.requests[6]);
        $display("Router Cmd: %b", sif.router_cmd);
        $display("Invalid Request: %b", sif.invalid_request);
        $display("Feedback Timeout Error: %b", sif.feedback_timeout_error);
        $display("----------------------------------------");
    endtask

endmodule
