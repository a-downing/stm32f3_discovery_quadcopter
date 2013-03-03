/*
Copyright (c) 2013, Andrew Downing
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cstdint>

// dont pass 0
constexpr uint32_t least_set_bit(uint32_t val, uint32_t pos = 0)
{
    return (val & (1 << pos)) ? pos : least_set_bit(val, pos + 1);
}

namespace USB
{
    constexpr void * const PERIPHERAL_ADDR = reinterpret_cast<void * const>(0x40005C00L);
    constexpr void * const SRAM_ADDR = reinterpret_cast<void * const>(0x40006000L);
    const uint32_t BTABLE_END_OFFSET = 64;
    const uint32_t NUM_ENDPOINTS = 8;

    struct Registers
    {
        volatile uint32_t EPR[16];
        volatile uint32_t CNTR;
        volatile uint32_t ISTR;
        volatile uint32_t FNR;
        volatile uint32_t DADDR;
        volatile uint32_t BTABLE;
    };

    struct BufferTable
    {
        volatile uint32_t ADDR_TX;
        volatile uint32_t COUNT_TX;
        volatile uint32_t ADDR_RX;
        volatile uint32_t COUNT_RX;
    };

    namespace CNTR
    {
        enum : uint32_t
        {
            CTRM_bm = 0x00008000,
            PMAOVRM_bm = 0x00004000,
            ERRM_bm = 0x00002000,
            WKUPM_bm = 0x00001000,
            SUSPM_bm = 0x00000800,
            RESETM_bm = 0x00000400,
            SOFM_bm = 0x00000200,
            ESOFM_bm = 0x00000100,
            RESUME_bm = 0x00000010,
            FSUSP_bm = 0x00000008,
            LP_MODE_bm = 0x00000004,
            PDWN_bm = 0x00000002,
            FRES_bm = 0x00000001,

            CTRM_bp = least_set_bit(CTRM_bm),
            PMAOVRM_bp = least_set_bit(PMAOVRM_bm),
            ERRM_bp = least_set_bit(ERRM_bm),
            WKUPM_bp = least_set_bit(WKUPM_bm),
            SUSPM_bp = least_set_bit(SUSPM_bm),
            RESETM_bp = least_set_bit(RESETM_bm),
            SOFM_bp = least_set_bit(SOFM_bm),
            ESOFM_bp = least_set_bit(ESOFM_bm),
            RESUME_bp = least_set_bit(RESUME_bm),
            FSUSP_bp = least_set_bit(FSUSP_bm),
            LP_MODE_bp = least_set_bit(LP_MODE_bm),
            PDWM_bp = least_set_bit(PDWN_bm),
            FRES_bp = least_set_bit(FRES_bm)
        };

        namespace CTRM
        {
            enum : uint32_t
            {
                CTR_INT_DISABLED = 0,
                CTR_INT_ENABLED = 1
            };
        }

        namespace PMAOVRM
        {
            enum : uint32_t
            {
                PMAOVRM_INT_DISABLED = 0,
                PMAOVRM_INT_ENABLED = 1
            };
        }

        namespace ERRM
        {
            enum : uint32_t
            {
                ERR_INT_DISABLED = 0,
                ERR_INT_ENABLED = 1
            };
        }

        namespace WKUPM
        {
            enum : uint32_t
            {
                WKUP_INT_DISABLED = 0,
                WKUP_INT_ENABLED = 1
            };
        }

        namespace SUSPM
        {
            enum : uint32_t
            {
                SUSP_INT_DISABLED = 0,
                SUSP_INT_ENABLED = 1
            };
        }

        namespace RESETM
        {
            enum : uint32_t
            {
                RESET_INT_DISABLED = 0,
                RESET_INT_ENABLED = 1
            };
        }

        namespace SOFM
        {
            enum : uint32_t
            {
                SOF_INT_DISABLED = 0,
                SOF_INT_ENABLED = 1
            };
        }

        namespace ESOFM
        {
            enum : uint32_t
            {
                ESOF_INT_DISABLED = 0,
                ESOF_INT_ENABLED = 1
            };
        }

        namespace RESUME
        {
            enum : uint32_t
            {
                RESUME_REQUEST = 1
            };
        }

        namespace FSUSP
        {
            enum : uint32_t
            {
                ENTER_SUSPEND = 1
            };
        }

        namespace LP_MODE
        {
            enum : uint32_t
            {
                EXIT_LP_MODE = 0,
                ENTER_LP_MODE = 1
            };
        }

        namespace PDWN
        {
            enum : uint32_t
            {
                EXIT_PDWN = 0,
                ENTER_PDWN = 1
            };
        }

        namespace FRES
        {
            enum : uint32_t
            {
                CLEAR_RESET = 0,
                FORCE_RESET = 1
            };
        }
    }

    namespace ISTR
    {
        enum : uint32_t
        {
            CTR_bm = 0x00008000,
            PMAOVR_bm = 0x00004000,
            ERR_bm = 0x00002000,
            WKUP_bm = 0x00001000,
            SUSP_bm = 0x00000800,
            RESET_bm = 0x00000400,
            SOF_bm = 0x00000200,
            ESOF_bm = 0x00000100,
            DIR_bm = 0x00000010,
            EP_ID_bm = 0x0000000F,

            CTR_bp = least_set_bit(CTR_bm),
            PMAOVR_bp = least_set_bit(PMAOVR_bm),
            ERR_bp = least_set_bit(ERR_bm),
            WKUP_bp = least_set_bit(WKUP_bm),
            SUSP_bp = least_set_bit(SUSP_bm),
            RESET_bp = least_set_bit(RESET_bm),
            SOF_bp = least_set_bit(SOF_bm),
            ESOF_bp = least_set_bit(ESOF_bm),
            DIR_bp = least_set_bit(DIR_bm),
            EP_ID_bp = least_set_bit(EP_ID_bm)
        };

        namespace CTR
        {
            enum : uint32_t
            {
                CORRECT_TRANSFER = 1
            };
        }

        namespace PMAOVR
        {
            enum : uint32_t
            {
                CLEAR_OVERRUN = 0,
                OVERRUN = 1
            };
        }

        namespace ERR
        {
            enum : uint32_t
            {
                CLEAR_ERROR = 0,
                ERROR = 1
            };
        }

        namespace WKUP
        {
            enum : uint32_t
            {
                CLEAR_WAKEUP = 0,
                WAKEUP = 1
            };
        }

        namespace SUSP
        {
            enum : uint32_t
            {
                CLEAR_SUSPEND = 0,
                SUSPEND = 1
            };
        }

        namespace RESET
        {
            enum : uint32_t
            {
                CLEAR_RESET = 0,
                RESET = 1
            };
        }

        namespace SOF
        {
            enum : uint32_t
            {
                CLEAR_SOF = 0,
                SOF = 1
            };
        }

        namespace ESOF
        {
            enum : uint32_t
            {
                CLEAR_EXPECTED_SOF = 0,
                EXPECTED_SOF = 1
            };
        }

        namespace DIR
        {
            enum : uint32_t
            {
                IN = 0,
                OUT = 1
            };
        }
    }

    namespace FNR
    {
        enum : uint32_t
        {
            RXDP_bm = 0x00008000,
            RXDM_bm = 0x00004000,
            LCK_bm = 0x00002000,
            LSOF_bm = 0x00001800,
            FN_bm = 0x000007FF,

            RXDP_bp = least_set_bit(RXDP_bm),
            RXDM_bp = least_set_bit(RXDM_bm),
            LCK_bp = least_set_bit(LCK_bm),
            LSOF_bp = least_set_bit(LSOF_bm),
            FN_bp = least_set_bit(FN_bm)
        };
    }

    namespace DADDR
    {
        enum : uint32_t
        {
            EF_bm = 0x00000080,
            ADD6_bm = 0x00000040,
            ADD5_bm = 0x00000020,
            ADD4_bm = 0x00000010,
            ADD3_bm = 0x00000008,
            ADD2_bm = 0x00000004,
            ADD1_bm = 0x00000002,
            ADD0_bm = 0x00000001,
            ADDR_bm = 0x0000007F,

            EF_bp = least_set_bit(EF_bm),
            ADD6_bp = least_set_bit(ADD6_bm),
            ADD5_bp = least_set_bit(ADD5_bm),
            ADD4_bp = least_set_bit(ADD4_bm),
            ADD3_bp = least_set_bit(ADD3_bm),
            ADD2_bp = least_set_bit(ADD2_bm),
            ADD1_bp = least_set_bit(ADD1_bm),
            ADD0_bp = least_set_bit(ADD0_bm),
            ADDR_bp = least_set_bit(ADDR_bm)
        };

        namespace EF
        {
            enum : uint32_t
            {
                DISABLE = 0,
                ENABLE = 1
            };
        }
    }

    namespace BTABLE
    {
        // the addr starts at 0, the first 3 bits are always forced by hardware to 0
        enum : uint32_t
        {
            BTABLE_bm = 0x0000FFF8,

            BTABLE_bp = least_set_bit(0x0000FFFF)
        };
    }

    namespace EPR
    {
        enum : uint32_t
        {
            CTR_RX_bm = 0x00008000,
            DTOG_RX_bm = 0x00004000,
            STAT_RX_bm = 0x00003000,
            SETUP_bm = 0x00000800,
            EP_TYPE_bm = 0x00000600,
            EP_KIND_bm = 0x00000100,
            CTR_TX_bm = 0x00000080,
            DTOG_TX_bm = 0x00000040,
            STAT_TX_bm = 0x00000030,
            EA_bm = 0x0000000F,

            CTR_RX_bp = least_set_bit(CTR_RX_bm),
            DTOG_RX_bp = least_set_bit(DTOG_RX_bm),
            STAT_RX_bp = least_set_bit(STAT_RX_bm),
            SETUP_bp = least_set_bit(SETUP_bm),
            EP_TYPE_bp = least_set_bit(EP_TYPE_bm),
            EP_KIND_bp = least_set_bit(EP_KIND_bm),
            CTR_TX_bp = least_set_bit(CTR_TX_bm),
            DTOG_TX_bp = least_set_bit(DTOG_TX_bm),
            STAT_TX_bp = least_set_bit(STAT_TX_bm),
            EA_bp = least_set_bit(EA_bm)
        };

        namespace CTR_RX
        {
            enum : uint32_t
            {
                CLEAR_CORRECT_XFER = 0,
                CORRECT_XFER = 1
            };
        }

        namespace DTOG_RX
        {
            enum : uint32_t
            {
                DATA0 = 0,
                DATA1 = 1,
                TOGGLE = 1
            };
        }

        namespace STAT_RX
        {
            enum : uint32_t
            {
                DISABLED = 0,
                STALL = 1,
                NAK = 2,
                VALID = 3
            };
        }

        namespace SETUP
        {
            enum : uint32_t
            {
                NOT_SETUP = 0,
                SETUP = 1
            };
        }

        namespace EP_TYPE
        {
            enum : uint32_t
            {
                BULK = 0,
                CONTROL = 1,
                ISO = 2,
                INTERRUPT = 3
            };
        }

        namespace EP_KIND
        {
            enum : uint32_t
            {
                NOT_DOUBLE_BUF = 0,
                NOT_STATUS_OUT = 0,
                DOUBLE_BUF = 1,
                STATUS_OUT = 1
            };
        }

        namespace CTR_TX
        {
            enum : uint32_t
            {
                CLEAR_CORRECT_XFER = 0,
                CORRECT_XFER = 1
            };
        }

        namespace DTOG_TX
        {
            enum : uint32_t
            {
                DATA0 = 0,
                DATA1 = 1,
                TOGGLE = 1
            };
        }

        namespace STAT_TX
        {
            enum : uint32_t
            {
                DISABLED = 0,
                STALL = 1,
                NAK = 2,
                VALID = 3
            };
        }
    }

    namespace COUNT_RX
    {
        enum : uint32_t
        {
            BL_SIZE_bm = 0x00008000,
            NUM_BLOCK_bm = 0x00007C00,
            COUNT_bm = 0x000003FF,

            BL_SIZE_bp = least_set_bit(BL_SIZE_bm),
            NUM_BLOCK_bp = least_set_bit(NUM_BLOCK_bm),
            COUNT_bp = least_set_bit(COUNT_bm)
        };

        namespace BL_SIZE
        {
            enum : uint32_t
            {
                BS_2_BYTES = 0,
                BS_32_BYTES = 1
            };
        }
    }

    namespace COUNT_TX
    {
        enum : uint32_t
        {
            COUNT_bm = 0x000003FF,

            COUNT_bp = least_set_bit(COUNT_bm)
        };
    }

    namespace ADDR_RX
    {
        enum : uint32_t
        {
            COUNT_bm = 0x0000FFFE, // starts at 0, but bit 0 must be 0

            COUNT_bp = least_set_bit(0x0000FFFF)
        };
    }

    namespace ADDR_TX
    {
        enum : uint32_t
        {
            COUNT_bm = 0x0000FFFE, // starts at 0, but bit 0 must be 0

            COUNT_bp = least_set_bit(0x0000FFFF)
        };
    }
}