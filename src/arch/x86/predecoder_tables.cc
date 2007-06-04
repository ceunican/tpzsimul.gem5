/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/x86/predecoder.hh"
#include "arch/x86/types.hh"

namespace X86ISA
{
    const uint8_t CS = CSOverride;
    const uint8_t DS = DSOverride;
    const uint8_t ES = ESOverride;
    const uint8_t FS = FSOverride;
    const uint8_t GS = GSOverride;
    const uint8_t SS = SSOverride;

    const uint8_t OO = OperandSizeOverride;
    const uint8_t AO = AddressSizeOverride;
    const uint8_t LO = Lock;
    const uint8_t RE = Rep;
    const uint8_t RN = Repne;
    const uint8_t RX = RexPrefix;

    //This table identifies whether a byte is a prefix, and if it is,
    //which prefix it is.
    const uint8_t Predecoder::Prefixes[256] =
    {    //LSB
// MSB   0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F
/*   0*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   1*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   2*/ 0 , 0 , 0 , 0 , 0 , 0 , ES, 0 , 0 , 0 , 0 , 0 , 0 , 0 , CS, 0,
/*   3*/ 0 , 0 , 0 , 0 , 0 , 0 , SS, 0 , 0 , 0 , 0 , 0 , 0 , 0 , DS, 0,
/*   4*/ RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX, RX,
/*   5*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   6*/ 0 , 0 , 0 , 0 , FS, GS, OO, AO, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   7*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   8*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   9*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   A*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   B*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   C*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   D*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   E*/ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*   F*/ LO, 0 , RN, RE, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
    };

    //This table identifies whether a particular opcode uses the ModRM byte
    const uint8_t Predecoder::UsesModRM[2][256] =
    {//For one byte instructions
        {    //LSB
//     MSB   0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F
/*      0 */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0,
/*      1 */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0,
/*      2 */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0,
/*      3 */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0,
/*      4 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      5 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      6 */ 0 , 0 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0,
/*      7 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      8 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      9 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      A */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      B */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      C */ 1 , 1 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      D */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      E */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      F */ 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1
        },
    //For two byte instructions
        {    //LSB
//     MSB   0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F
/*      0 */ 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 1,
/*      1 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      2 */ 1 , 1 , 1 , 1 , 1 , 0 , 1 , 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      3 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      4 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      5 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      6 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      7 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1,
/*      8 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      9 */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      A */ 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1,
/*      B */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 1 , 1 , 1 , 1 , 1 , 1,
/*      C */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0,
/*      D */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      E */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1,
/*      F */ 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0
        }
    };

    enum SizeType {
        NoImm,
        NI = NoImm,
        ByteImm,
        BY = ByteImm,
        WordImm,
        WO = WordImm,
        DWordImm,
        DW = DWordImm,
        QWordImm,
        QW = QWordImm,
        OWordImm,
        OW = OWordImm,
        VWordImm,
        VW = VWordImm,
        ZWordImm,
        ZW = ZWordImm,
        //The enter instruction takes -2- immediates for a total of 3 bytes
        Enter,
        EN = Enter,
        Pointer,
        PO = Pointer
    };

    const uint8_t Predecoder::SizeTypeToSize[3][10] =
    {
//       noimm byte word dword qword oword vword zword enter pointer
        {0,    1,   2,   4,    8,    16,   2,    2,    3,    4      }, //16 bit
        {0,    1,   2,   4,    8,    16,   4,    4,    3,    6      }, //32 bit
        {0,    1,   2,   4,    8,    16,   8,    4,    3,    0      }  //64 bit
    };

    //This table determines the immediate type. The first index is the
    //number of bytes in the instruction, and the second is the meaningful
    //byte of the opcode. I didn't use the NI constant here for the sake
    //of clarity.
    const uint8_t Predecoder::ImmediateType[2][256] =
    {//For one byte instructions
        {    //LSB
//     MSB   0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F
/*      0 */ 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 , 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 ,
/*      1 */ 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 , 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 ,
/*      2 */ 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 , 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 ,
/*      3 */ 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 , 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 ,
/*      4 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      5 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      6 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , ZW, ZW, BY, BY, 0 , 0 , 0 , 0 ,
/*      7 */ BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY, BY,
/*      8 */ BY, ZW, BY, BY, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      9 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      A */ BY, VW, BY, VW, 0 , 0 , 0 , 0 , BY, ZW, 0 , 0 , 0 , 0 , 0 , 0 ,
/*      B */ BY, BY, BY, BY, BY, BY, BY, BY, VW, VW, VW, VW, VW, VW, VW, VW,
/*      C */ BY, BY, WO, 0 , 0 , 0 , BY, ZW, EN, 0 , WO, 0 , 0 , BY, 0 , 0 ,
/*      D */ 0 , 0 , 0 , 0 , BY, BY, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      E */ BY, BY, BY, BY, BY, BY, BY, BY, ZW, ZW, PO, BY, 0 , 0 , 0 , 0 ,
/*      F */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
        },
    //For two byte instructions
        {    //LSB
//     MSB   0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | A | B | C | D | E | F
/*      0 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      0 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      2 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      3 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      4 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      5 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      6 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      7 */ BY, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      8 */ ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW, ZW,
/*      9 */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      A */ 0 , 0 , 0 , 0 , BY, 0 , 0 , 0 , 0 , 0 , 0 , 0 , BY, 0 , 0 , 0 ,
/*      B */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , ZW, 0 , BY, 0 , 0 , 0 , 0 , 0 ,
/*      C */ 0 , 0 , BY, 0 , BY, BY, BY, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      D */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      E */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,
/*      F */ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
        }
    };
}
