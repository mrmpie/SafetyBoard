package safety_pkg;
    typedef enum logic [1:0] {
        OPEN = 2'b00,
        CLOSED = 2'b01,
        ERROR = 2'b11
    } contactor_state_t;
endpackage

interface SafetyBoardInterface;
    // Output connection requests (diagonal of the matrix)
    logic [5:0] output_requests;    // Request to connect power group to its output
    
    // Interlink requests - split by distance between groups
    logic [4:0] interlink_req_dist1;  // PG_N to PG_N+1 (PG0-PG1, PG1-PG2, PG2-PG3, PG3-PG4, PG4-PG5)
    logic [3:0] interlink_req_dist2;  // PG_N to PG_N+2 (PG0-PG2, PG1-PG3, PG2-PG4, PG3-PG5)
    logic [2:0] interlink_req_dist3;  // PG_N to PG_N+3 (PG0-PG3, PG1-PG4, PG2-PG5)
    logic [1:0] interlink_req_dist4;  // PG_N to PG_N+4 (PG0-PG4, PG1-PG5)
    logic       interlink_req_dist5;  // PG_N to PG_N+5 (PG0-PG5)
    
    // Command to power routers (same for plus and minus)
    logic [20:0] router_cmd;   
    
    // Feedback from power routers - 2 feedback signals per connection
    logic [41:0] router_feedback;     // 2 feedback bits per router command bit
    
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
        input  clk, rst_n, output_requests, 
              interlink_req_dist1, interlink_req_dist2, 
              interlink_req_dist3, interlink_req_dist4, 
              interlink_req_dist5,
              router_feedback,
        output router_cmd, invalid_request, feedback_timeout_error, 
               current_state, validate_state,
        input  shutdown_commands,
        output pg_shutdown
    );

    modport Test (
        output clk, rst_n, output_requests,
              interlink_req_dist1, interlink_req_dist2,
              interlink_req_dist3, interlink_req_dist4,
              interlink_req_dist5,
              router_feedback,
        input  router_cmd, invalid_request, feedback_timeout_error,
              current_state, validate_state,
        output shutdown_commands,
        input  pg_shutdown
    );
endinterface

module SafetyBoard #(
    parameter int FEEDBACK_TIMEOUT_CYCLES = 4
)(
    SafetyBoardInterface sif
);
    // State machine states
    typedef enum logic [2:0] {
        IDLE          = 3'b000,
        VALIDATE_REQ  = 3'b001,
        WAIT_FEEDBACK = 3'b010,
        FEEDBACK_ERROR = 3'b011,
        COMMAND_READY  = 3'b100
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
    
    // Helper function to set interlink entries based on distance array
    function automatic void set_interlink_entries(int distance, logic [4:0] req_array);
        int max_idx = 6 - distance;  // Number of connections for this distance
        for (int i = 0; i < max_idx; i++) begin
            set_matrix_entry(i, i + distance, req_array[i]);
        end
    endfunction
    
    // Build request matrix from individual requests
    function automatic void build_request_matrix();
        // Clear matrix first
        request_matrix = '0;
        
        // Set diagonal (output requests)
        for (int i = 0; i < 6; i++) begin
            request_matrix[i][i] = sif.output_requests[i];
        end
        
        // Set interlink requests using helper function
        set_interlink_entries(1, sif.interlink_req_dist1);
        set_interlink_entries(2, sif.interlink_req_dist2[3:0]);
        set_interlink_entries(3, sif.interlink_req_dist3[2:0]);
        set_interlink_entries(4, sif.interlink_req_dist4[1:0]);
        set_matrix_entry(0, 5, sif.interlink_req_dist5);
    endfunction

    // Assign current states to interface for debug
    assign sif.current_state = current_state;
    assign sif.validate_state = validate_state;

    // Helper function to check feedback for a specific connection
    function automatic logic check_connection_feedback(int index);
        return (sif.router_feedback[2*index +: 2] == {contactor_commanded[index], contactor_commanded[index]});
    endfunction

    // Helper function to get output index
    function automatic int get_output_idx(int group);
        return group;  // Direct mapping for outputs
    endfunction

    // Helper function to get interlink index
    function automatic int get_interlink_idx(int from_group, int to_group);
        int base_idx = 6;  // Start after output contactors
        int distance = to_group - from_group;
        int idx;
        
        case (distance)
            1: begin  // Distance 1 connections: indices 6-10
                idx = base_idx + from_group;
            end
            2: begin  // Distance 2 connections: indices 11-14
                idx = base_idx + 5 + from_group;
            end
            3: begin  // Distance 3 connections: indices 15-17
                idx = base_idx + 9 + from_group;
            end
            4: begin  // Distance 4 connections: indices 18-19
                idx = base_idx + 12 + from_group;
            end
            5: begin  // Distance 5 connection: index 20
                idx = base_idx + 14;  // Only one connection: PG0-PG5
            end
            default: idx = 0;  // Invalid distance
        endcase
        
        return idx;
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
        end else begin
            case (current_state)
                IDLE: begin
                    // Build request matrix first
                    build_request_matrix();
                    
                    // Initialize validation state machine
                    validate_state <= INIT_CHECK;
                    check_i <= '0;
                    check_j <= '0;
                    validation_invalid <= '0;
                    
                    // Move to validation state
                    current_state <= VALIDATE_REQ;
                end

                VALIDATE_REQ: begin
                    case (validate_state)
                        INIT_CHECK: begin
                            // Initialize array
                            for (int i = 0; i < 6; i++) begin
                                pg_output_assignments[i] = 0;
                                if (request_matrix[i][i]) begin
                                    pg_output_assignments[i] = i + 1;
                                end
                            end
                            validate_state <= CHECK_OUTPUTS;
                        end

                        CHECK_OUTPUTS: begin
                            // Verify no conflicts in direct output assignments
                            if (check_i < 6) begin
                                if (check_j < 6) begin
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
                                // Process output requests from matrix diagonal
                                for (int i = 0; i < 6; i++) begin
                                    if (request_matrix[i][i]) begin
                                        pending_command[get_output_idx(i)] = 1'b1;
                                    end else begin
                                        pending_command[get_output_idx(i)] = 1'b0;
                                    end
                                end

                                // Process interlink requests from upper triangle
                                for (int i = 0; i < 5; i++) begin
                                    for (int j = i + 1; j < 6; j++) begin
                                        if (request_matrix[i][j]) begin
                                            pending_command[get_interlink_idx(i, j)] = 1'b1;
                                        end else begin
                                            pending_command[get_interlink_idx(i, j)] = 1'b0;
                                        end
                                    end
                                end
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
                    
                    if (!sif.feedback_timeout_error) begin
                        if (feedback_match) begin
                            current_state <= COMMAND_READY;
                            feedback_timeout_counter <= '0;
                        end else begin
                            // Check for timeout
                            if (feedback_timeout_counter >= FEEDBACK_TIMEOUT_CYCLES) begin
                                sif.feedback_timeout_error <= 1'b1;
                                current_state <= FEEDBACK_ERROR;
                            end else begin
                                feedback_timeout_counter <= feedback_timeout_counter + 1;
                            end
                        end
                    end else begin
                        current_state <= FEEDBACK_ERROR;
                    end
                end

                FEEDBACK_ERROR: begin
                    // Safety response: Open all contactors
                    contactor_commanded <= '0;
                    request_matrix <= '0;
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
        sif.output_requests = '0;
        sif.interlink_req_dist1 = '0;
        sif.interlink_req_dist2 = '0;
        sif.interlink_req_dist3 = '0;
        sif.interlink_req_dist4 = '0;
        sif.interlink_req_dist5 = '0;
        sif.router_feedback = '0;
        
        // Hold reset for 1us (4 clock cycles)
        #1000 sif.rst_n = 1;
        
        // Wait a few clocks
        @(posedge clk);
        @(posedge clk);
        
        // Enable default connection for Power Group 1 (index 0)
        sif.output_requests[0] = 1'b1;  // PG1 = index 0
        
        @(posedge clk);
        
        // Enable default connection for Power Group 2 (index 1)
        sif.output_requests[1] = 1'b0;  // PG2 = index 1
        
        @(posedge clk);
        
        // Request interlink between PG1 and PG3 (index 0 and 2)
        sif.interlink_req_dist1[0] = 1'b1;  // Connect PG1 (0) to PG3 (2)
        
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
            sif.current_state == 3'b001 ? "VALIDATE_REQ" :
            sif.current_state == 3'b010 ? "WAIT_FEEDBACK" :
            sif.current_state == 3'b011 ? "FEEDBACK_ERROR" :
            sif.current_state == 3'b100 ? "COMMAND_READY" : "UNKNOWN"
        );
        $display("Default Connections: %b", sif.output_requests);
        $display("Interlink Request [0][1]: %b", sif.interlink_req_dist1[0]);
        $display("Router Cmd: %b", sif.router_cmd);
        $display("Invalid Request: %b", sif.invalid_request);
        $display("Feedback Timeout Error: %b", sif.feedback_timeout_error);
        $display("----------------------------------------");
    endtask

endmodule
