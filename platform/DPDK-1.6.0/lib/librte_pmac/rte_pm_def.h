/*-
 *      Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *      All rights reserved.
 *   
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions
 *      are met:
 *   
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *        * Neither the name of Intel Corporation nor the names of its
 *          contributors may be used to endorse or promote products derived
 *          from this software without specific prior written permission.
 *  
 *        * The use of this source code in binary form is permitted to only
 *          be used on Intel® Architecture Processors.
 *   
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *      AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *      IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *      ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *      LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *      DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *      OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef pm_search_method
#define	pm_search_method(num, name)
#endif /* pm_search_method */

#ifndef pm_search_method_end
#define	pm_search_method_end(num)
#endif /* pm_search_method_end */

pm_search_method(RTE_PM_SEARCH_UNDEF,         "nil")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4,      "AC2_L1x4")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4_MB,   "AC2_L1x4_MB")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4_MH,   "AC2_L1x4_MH")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4_MH5,  "AC2_L1x4_MH5")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4_MH7,  "AC2_L1x4_MH7")
pm_search_method(RTE_PM_SEARCH_AC2_L1x4_MH11, "AC2_L1x4_MH11")
pm_search_method(RTE_PM_SEARCH_AC2_R128x2,    "AC2_R128x2")
pm_search_method(RTE_PM_SEARCH_AC2_R256x2,    "AC2_R256x2")
pm_search_method(RTE_PM_SEARCH_AC2_P1,        "AC2_P1")
pm_search_method_end(RTE_PM_SEARCH_NUM)

#undef pm_search_method
#undef pm_search_method_end
