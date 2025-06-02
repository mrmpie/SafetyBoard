package spi_pkg;
    // Word length configuration
    `define SPI_WORD_LENGTH 8

    // SPI Commands
    typedef enum logic [7:0] {
        CMD_VERSION           = 8'h00,  // Get FPGA version
        // Read commands
        CMD_READ_CONTACTOR    = 8'h01,  // Read contactor status at index
        CMD_READ_FEEDBACK     = 8'h02,  // Read feedback at index
        CMD_READ_STATUS       = 8'h03,  // Read status register
        CMD_READ_SHUTDOWN     = 8'h04,  // Read shutdown status
        CMD_READ_CONTROL      = 8'h05,  // Read control register

        // Write commands
        CMD_WRITE_CONTACTOR   = 8'h81,  // Write contactor command at index
        CMD_WRITE_CONTROL     = 8'h82,  // Write control register
        CMD_WRITE_SHUTDOWN    = 8'h83,  // Write shutdown command
        CMD_WRITE_PG_SHUTDOWN = 8'h84   // Write direct PG shutdown
    } spi_cmd_t;

    // Status register
    typedef struct packed {
        logic feedback_timeout_error;
        logic invalid_request;
        logic [1:0] thermal_shutdown;
        logic [3:0] reserved;
    } status_reg_t;

    // Control register
    typedef struct packed {
        logic reset_req;        // Bit 0: Reset request
        logic clear_errors;     // Bit 1: Clear error flags
        logic [5:0] reserved;   // Bits 7-2: Reserved for future use
    } control_reg_t;

    // Contactor data format
    typedef struct packed {
        logic [5:0] reserved;
        logic [1:0] feedback;  // {plus_feedback, minus_feedback}
    } contactor_data_t;

endpackage
