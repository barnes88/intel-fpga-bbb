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

`include "rtl/TopLevel.v"

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

    logic chisel_wrEnable;
    logic [9:0] chisel_wrAddr;
    logic [63:0] chisel_wrData_0;
    logic [63:0] chisel_wrData_1;
    logic [63:0] chisel_wrData_2;
    logic [63:0] chisel_wrData_3;
    logic [63:0] chisel_rdData_0;
    logic [63:0] chisel_rdData_1;
    logic [63:0] chisel_rdData_2;
    logic [63:0] chisel_rdData_3;
    logic [9:0] chisel_rdAddr;
    //assign chisel_rdAddr = 10'h1;

    
    //
    // Consume configuration CSR writes
    //

    // We use CSR 0 to set the memory address.
    logic is_rd_addr_csr_write;
    assign is_rd_addr_csr_write = csrs.cpu_wr_csrs[0].en;

    // Memory address to which this AFU will read 
//    t_ccip_clAddr  rd_addr ;
//
//    always_ff @(posedge clk)
//    begin
//        if (is_ rd_addr _csr_write)
//        begin
//            rd_addr<= t_ccip_clAddr'(csrs.cpu_wr_csrs[0].data);
//        end
//    end

    // Use CSR 1 to set memory address for which AFU will write
    logic is_wr_addr_csr_write;
    assign is_wr_addr_csr_write = csrs.cpu_wr_csrs[1].en;
    t_ccip_clAddr wr_addr;

   
    // =========================================================================
    //
    //   Main AFU logic
    // =========================================================================

    //
    // States in our simple example.
    //
    typedef enum logic [1:0]
    {
        STATE_IDLE,
        STATE_READ_DATA,
        STATE_WRITE_DATA,
        STATE_FINISHED
    } t_state;

    t_state state;

    //
    // State machine
    //
    logic wr_inflight;
    logic rd_inflight;

    always_ff @(posedge clk)
    begin
        if (reset)
        begin
            state <= STATE_IDLE;
        end
        else
        begin
            // Trigger the AFU when rd_addr is set above.  (When the CPU
            // tells us the address of the FPGA shared memory buffer 
            if ((state == STATE_IDLE) && is_rd_addr_csr_write)
            begin
                state <= STATE_READ_DATA;
                $display("AFU Reading...");
            end

            // The AFU READS data into SRAM
            if ((state == STATE_READ_DATA && cci_c0Rx_isReadRsp(fiu.c0Rx)) && chisel_wrAddr == 10'h2)
            begin
                state <= STATE_WRITE_DATA;
                $display("AFU Writing...");

            end

            // The AFU Writes data back to the shared buffer 
            if ((state == STATE_WRITE_DATA) && chisel_rdAddr == 10'h4)
            begin
                state <= STATE_FINISHED;
                $display("AFU FINISHED");

            end

            if(state == STATE_FINISHED)
            begin

            end

       end
    end

    //
    // CSR0 written 1 when AFU is finished, otherwise write 0
    //
    always_comb
    begin
        // The AFU ID is a unique ID for a given program.  Here we generated
        // one with the "uuidgen" program and stored it in the AFU's JSON file.
        // ASE and synthesis setup scripts automatically invoke afu_json_mgr
        // to extract the UUID into afu_json_info.vh.
        csrs.afu_id = `AFU_ACCEL_UUID;

        // Default
        for (int i = 1; i < NUM_APP_CSRS; i = i + 1)
        begin
            csrs.cpu_rd_csrs[i].data = 64'(0);
        end
        csrs.cpu_rd_csrs[0].data = (state == STATE_FINISHED) ? 64'h1 : 64'h0;
    end


    //
    // Read shared memory into SRAM when in STATE_RUN 
    //
    
    // Construct a memory read header
    t_cci_mpf_c0_ReqMemHdr rd_hdr;
    t_cci_mpf_ReqMemHdrParams rd_hdr_params;

    t_ccip_clAddr  rd_addr;

    // Increment read address by 64 (1 cacheline)
//    always_ff @(posedge clk)
//        rd_addr <= is_rd_addr_csr_write ? t_ccip_clAddr'(csrs.cpu_wr_csrs[0].data) : rd_addr + 64'h40;

    always_comb
    begin
        // Use Physical addresses
        rd_hdr_params = cci_mpf_defaultReqHdrParams(0);
        // Let FIU pick the channel
        //rd_hdr_params.vc_sel = eVC_VA;
        // Read 1 cache line (64 Bytes)
        rd_hdr_params.cl_len = eCL_LEN_1;

        // Generate the header
        rd_hdr = cci_mpf_c0_genReqHdr(  eREQ_RDLINE_I,
                                         rd_addr ,
                                        t_cci_mdata'(0),
                                        rd_hdr_params);
    end

    // Send read requests to FIU
    always_ff @(posedge clk)
    begin
        if (reset || state == STATE_IDLE)
        begin
            fiu.c0Tx.valid <= 1'b0;
            chisel_wrAddr <= 10'd1023; // first write will wrap around to 0
            rd_inflight <= 1'b0;
        end
        else begin
            // Generate a read request when FIU isn't full and a read request isn't already in flight
            if (!fiu.c0TxAlmFull && state == STATE_READ_DATA && !rd_inflight) begin
                fiu.c0Tx <= cci_mpf_genC0TxReadReq(rd_hdr, 1'b1);
                $display("  Sent Read request with addr 0x%x", rd_addr);
                rd_inflight <= 1'b1;

            end
            else
                fiu.c0Tx <= cci_mpf_genC0TxReadReq(rd_hdr, 1'b0);
        end
    end 


    // Read Response handling

    always_ff @(posedge clk) 
    begin
        if (cci_c0Rx_isReadRsp(fiu.c0Rx) && state == STATE_READ_DATA)
        begin
            chisel_wrEnable <= 1'b1;
            chisel_wrData_0 <= fiu.c0Rx.data[63:0];
            chisel_wrData_1 <= fiu.c0Rx.data[127:64];
            chisel_wrData_2 <= fiu.c0Rx.data[191:128];
            chisel_wrData_3 <= fiu.c0Rx.data[255:192];
            chisel_wrAddr <= chisel_wrAddr + 10'b1;
            rd_addr <= rd_addr + 64'h1; // cci address is 64-byte aligned!
            rd_inflight <= 1'b0;
            $display("  Read data Hex: 0x%x", fiu.c0Rx.data[511:0]);
            $display("  Chisel wrAddr: 0x%x", chisel_wrAddr+10'b1);
        end
        else
        begin
            chisel_wrEnable <= 1'b0;
            rd_addr <= is_rd_addr_csr_write ? t_ccip_clAddr'(csrs.cpu_wr_csrs[0].data) : rd_addr;
        end
    end
    


    //
    // Write shared memory from SRAM back to shared buf in STATE_WRITE_DATA
    //
    
    // Construct a memory write request header.  For this AFU it is always
    // the same, since we write to only one address.
    t_cci_mpf_c1_ReqMemHdr wr_hdr;
    assign wr_hdr = cci_mpf_c1_genReqHdr(eREQ_WRLINE_I,
                                         wr_addr,
                                         t_cci_mdata'(0),
                                         cci_mpf_defaultReqHdrParams());

    assign fiu.c1Tx.data = t_ccip_clData'({chisel_rdData_3, chisel_rdData_2, chisel_rdData_1, chisel_rdData_0});

    // Control logic for memory write requests
    always_ff @(posedge clk)
    begin
        if (reset || state != STATE_WRITE_DATA)
        begin
            fiu.c1Tx.valid <= 1'b0;
            wr_inflight <= 1'b0;
            chisel_rdAddr <= 10'b0;
        end
        else
        begin
            // Request the write as long as the channel isn't full.
            fiu.c1Tx.valid <= ((state == STATE_WRITE_DATA) && ! fiu.c1TxAlmFull && !wr_inflight);
            if ((state == STATE_WRITE_DATA) && ! fiu.c1TxAlmFull && !wr_inflight)
            begin
                //write_sent <= 1'b1;
                wr_inflight <= 1'b1;
                $display("  sent Write request to addr %0h", wr_addr);
                $display("  writen data was: %0h", fiu.c1Tx.data);
                $display("  chisel_rdAddr was: %0h", chisel_rdAddr);
                $display("  chisel_wrEnable was: %d", chisel_wrEnable);
                chisel_rdAddr <= chisel_rdAddr + 1'b1;
            end
        end

        fiu.c1Tx.hdr <= wr_hdr;
    end


    // Handle write acks
    always_ff @(posedge clk)
    begin
        if (cci_c1Rx_isWriteRsp(fiu.c1Rx))
        begin
            $display("Received write ack for wr_addr: 0x%x", wr_addr);
            wr_inflight <= 1'b0;
            wr_addr <= wr_addr + 64'h1;
        end
        else
            wr_addr <= is_wr_addr_csr_write ? t_ccip_clAddr'(csrs.cpu_wr_csrs[1].data) : wr_addr;
    end

    //
    // This AFU never handles MMIO reads.
    //
    assign fiu.c2Tx.mmioRdValid = 1'b0;

    //
    // Hookup Chisel Module it doesn't do anything right now
    //

//    TopLevel assigntask (
//        .clock(clk),
//        .reset(reset),
//        .io_writeDataMem_wrEnable(chisel_wrEnable),
//        .io_writeDataMem_wrAddr(chisel_wrAddr),
//        .io_writeDataMem_wrData_0(chisel_wrData_0),
//        .io_writeDataMem_wrData_1(chisel_wrData_1),
//        .io_writeDataMem_wrData_2(chisel_wrData_2),
//        .io_writeDataMem_wrData_3(chisel_wrData_3),
//        .io_readIDMem_rdAddr(),
//        .io_readIDMem_rdData(),
//        .io_writeSharedMem_wrEnable(),
//        .io_writeSharedMem_wrAddr(),
//        .io_writeSharedMem_wrData(),
//        .io_startWorking(1'b0),
//        .io_doneWorking()
//    );

    DataMemory chisel_mem (
        .clock(clk),
        .io_read_rdAddr(chisel_rdAddr),
        .io_read_rdData_0(chisel_rdData_0),
        .io_read_rdData_1(chisel_rdData_1),
        .io_read_rdData_2(chisel_rdData_2),
        .io_read_rdData_3(chisel_rdData_3),
        .io_write_wrEnable(chisel_wrEnable),
        .io_write_wrAddr(chisel_wrAddr),
        .io_write_wrData_0(chisel_wrData_0),
        .io_write_wrData_1(chisel_wrData_1),
        .io_write_wrData_2(chisel_wrData_2),
        .io_write_wrData_3(chisel_wrData_3)
    );

endmodule : app_afu // app_afu

