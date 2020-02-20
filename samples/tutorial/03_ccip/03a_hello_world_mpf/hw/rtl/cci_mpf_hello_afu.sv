//
// Copyright (c) 2017, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

`include "cci_mpf_if.vh"
`include "csr_mgr.vh"
`include "afu_json_info.vh"

//`define SHARED_MEM_SIZE 1024
//`define SHARED_MEM_BITS $clog2(SHARED_MEM_SIZE)

module app_afu
   (
    input  logic clk,

    // Connection toward the host.  Reset comes in here.
    cci_mpf_if.to_fiu fiu,

    // CSR connections
    app_csrs.app csrs,

    // MPF tracks outstanding requests.  These will be true as long as
    // reads or unacknowledged writes are still in flight.
    input  logic c0NotEmpty, // Memory read channel
    input  logic c1NotEmpty  // Memory write channel
    );

    // Local reset to reduce fan-out
    logic reset = 1'b1;
    always @(posedge clk)
    begin
        reset <= fiu.reset;
    end


    // ====================================================================
    //
    //  CSRs (simple connections to the external CSR management engine)
    //
    // ====================================================================

    always_comb
    begin
        // The AFU ID is a unique ID for a given program.  Here we generated
        // one with the "uuidgen" program and stored it in the AFU's JSON file.
        // ASE and synthesis setup scripts automatically invoke afu_json_mgr
        // to extract the UUID into afu_json_info.vh.
        csrs.afu_id = `AFU_ACCEL_UUID;

        // Default
        for (int i = 0; i < NUM_APP_CSRS; i = i + 1)
        begin
            csrs.cpu_rd_csrs[i].data = 64'(0);
        end
    end


    //
    // Consume configuration CSR writes
    //

    // We use CSR 0 to set the memory address.
    logic is_mem_addr_csr_write;
    assign is_mem_addr_csr_write = csrs.cpu_wr_csrs[0].en;

    // Memory address to which this AFU will read / write.
    t_ccip_clAddr mem_addr;

    always_ff @(posedge clk)
    begin
        if (is_mem_addr_csr_write)
        begin
            mem_addr <= t_ccip_clAddr'(csrs.cpu_wr_csrs[0].data);
        end
    end


    // =========================================================================
    //
    //   Main AFU logic
    //
    // =========================================================================

    //
    // States in our simple example.
    //
    typedef enum logic [1:0]
    {
        STATE_IDLE,
        STATE_READ,
        STATE_WRITE
    } t_state;

    t_state state;

    //
    // State machine
    //
    logic rd_needed;

    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            state <= STATE_IDLE;
            rd_needed <= 1'b0;
        end
        else
        begin
            // Trigger the AFU when mem_addr is set above.  (When the CPU
            // tells us the address of the FPGA shared memory buffer 
            if ((state == STATE_IDLE) && is_mem_addr_csr_write)
            begin
                state <= STATE_READ;
                $display("AFU Reading...");
                rd_needed <= 0;
            end

            // The AFU READS data into SRAM
            if ((state == STATE_READ && cci_c0Rx_isReadRsp(fiu.c0Rx)) )
            begin
                state <= STATE_WRITE;
                $display("AFU Writing...");
                rd_needed <= 1'b1;
            end

            // The AFU Writes SHARED_MEM_SIZE data back to the shared buffer after adding 1 
            if ((state == STATE_WRITE) && !fiu.c1TxAlmFull)
            begin
                state <= STATE_IDLE;
                $display("AFU FINISHED");
                rd_needed <= 1'b0;
            end

       end
    end


    //
    // Read shared memory into SRAM when in STATE_RUN 
    //
    
    // Construct a memory read header
    t_cci_mpf_c0_ReqMemHdr rd_hdr;
    t_cci_mpf_ReqMemHdrParams rd_hdr_params;
    //t_ccip_clAddr rd_addr;

    always_comb
    begin
        // Use Physical addresses
        rd_hdr_params = cci_mpf_defaultReqHdrParams(0);
        // Let FIU pick the channel
        //rd_hdr_params.vc_sel = eVC_VA;
        // Read 1 line
        //rd_hdr_params.cl_len = eCL_LEN_1;

        // Generate the header
        rd_hdr = cci_mpf_c0_genReqHdr(  eREQ_RDLINE_I,
                                        mem_addr,
                                        t_cci_mdata'(0),
                                        rd_hdr_params);
    end

    // Send read requests to FIU
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            fiu.c0Tx.valid <= 1'b0;
        end
        else begin
            // Generate a read request when FIU isn't full
            fiu.c0Tx <= cci_mpf_genC0TxReadReq(rd_hdr, (!fiu.c0TxAlmFull && rd_needed));
        end
    end 


    // Read Response handling
    logic [63:0] write_message;

    always_ff @(posedge clk) 
    begin
        if (cci_c0Rx_isReadRsp(fiu.c0Rx))
        begin
            $display("  Read data %0d", fiu.c0Rx.data[63:0]);
            write_message <= fiu.c0Rx.data[63:0] + 1'b1;
        end
    end


    //
    // Write shared memory from SRAM back to shared buf in STATE_WRITE
    //
    
    // Construct a memory write request header.  For this AFU it is always
    // the same, since we write to only one address.
    t_cci_mpf_c1_ReqMemHdr wr_hdr;
    assign wr_hdr = cci_mpf_c1_genReqHdr(eREQ_WRLINE_I,
                                         mem_addr,
                                         t_cci_mdata'(0),
                                         cci_mpf_defaultReqHdrParams());

    assign fiu.c1Tx.data = t_ccip_c1Data'(write_message);

    // Control logic for memory writes
    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            fiu.c1Tx.valid <= 1'b0;
        end
        else
        begin
            // Request the write as long as the channel isn't full.
            fiu.c1Tx.valid <= ((state == STATE_RUN) && ! fiu.c1TxAlmFull);
        end

        fiu.c1Tx.hdr <= wr_hdr;
    end


    //
    // This AFU never handles MMIO reads.
    //
    assign fiu.c0Tx.valid = 1'b0;
    assign fiu.c2Tx.mmioRdValid = 1'b0;

endmodule // app_afu

