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

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>

#include <iostream>
#include <string>

using namespace std;

#include "opae_svc_wrapper.h"
#include "csr_mgr.h"

// State from the AFU's JSON file, extracted using OPAE's afu_json_mgr script
#include "afu_json_info.h"

int main(int argc, char *argv[])
{
    // Find and connect to the accelerator
    OPAE_SVC_WRAPPER fpga(AFU_ACCEL_UUID);
    assert(fpga.isOk());

    // Connect the CSR manager
    CSR_MGR csrs(fpga);

    // Allocate a single page memory buffer (AFU will read from)
    auto buf_handle = fpga.allocBuffer(getpagesize());
    auto buf = reinterpret_cast<volatile unsigned long long*>(buf_handle->c_type());
    uint64_t buf_pa = buf_handle->io_address();
    assert(NULL != buf);

    // Allocate a single page memory buffer (AFU will read from)
    auto out_buf_handle = fpga.allocBuffer(getpagesize());
    auto out_buf = reinterpret_cast<volatile unsigned long long*>(out_buf_handle->c_type());
    uint64_t out_buf_pa = out_buf_handle->io_address();
    assert(NULL != out_buf);

    // Write a 1 to accelerator
    //buf[0] = 1;
    //out_buf[0] = 0;

    // Write 4 cache-lines worth of data to accelerator (1-CL = 64 Bytes = 16 Ints = 8 long longs)
    // top 256 bits should be padded with 1s
    int data_counter =1;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            buf[i*8+j] = static_cast<unsigned long long>(data_counter++); // data
        }
        for (int j = 4; j < 8; j++) {
            buf[i*8+j] = static_cast<unsigned long long>(0xFFFFFFFFFFFFFFFF); // padding
        }
     }


    // Initializes csr 2 to 0, will use to indicate AFU is finished
    // csrs.writeCSR(2, 0)

    // Tell the accelerator the address of the buffer using cache line
    // addresses by writing to application CSR 0.  The CSR manager maps
    // its registers to MMIO space.  The accelerator will respond by
    // writing to the buffer.
    csrs.writeCSR(1, out_buf_pa / CL(1));
    csrs.writeCSR(0, buf_pa / CL(1));

    // Writing to buf[0] kicks off the accelerator
    //buf[0] = static_cast<unsigned long long>();

    // Spin, waiting for the value in memory to change to something non-zero.
    while (0 == out_buf[24])

    // Spin, waiting for the last bit of CSR 1 to flip from 0 to 1
    //while (!(csrs.readCSR(0) && 0x01))
    {
        // A well-behaved program would use _mm_pause(), nanosleep() or
        // equivalent to save power here.
    };

    // Print the string written by the FPGA
    //cout << (char*)buf << endl;
    for (int i = 0; i < 32; i++) {
        cout << "OUTPUT WAS: " << (unsigned long long*)out_buf[i] << endl;
    }
    //cout << "OUTPUT WAS: " << (unsigned int*)out_buf[0] << endl;
    cout << "CSR0 output was " << csrs.readCSR(0) << endl;

    // Ask the FPGA-side CSR manager the AFU's frequency
    cout << endl
         << "# AFU frequency: " << csrs.getAFUMHz() << " MHz"
         << (fpga.hwIsSimulated() ? " [simulated]" : "")
         << endl;

    // All shared buffers are automatically released and the FPGA connection
    // is closed when their destructors are invoked here.
    return 0;
}
