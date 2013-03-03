/*
Copyright (c) 2013, Vegard Storheil Eriksen
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

#ifndef RBLOG_H
#define RBLOG_H

#include <stdint.h>
//#include <os/time.h>

template <uint32_t E, uint32_t A = 0>
class RBLog {
	private:
		static const uint32_t num_entries = E;
		static const uint32_t num_arguments = A;
		
		struct entry_t {
			uint32_t timestamp;
			const char* string;
			uint32_t arguments[num_arguments];
		};
		
		entry_t entries[num_entries];
		uint32_t index;
	
	public:
		RBLog() : index(0) {
			for(entry_t& entry : entries) {
				entry = {0, 0};
			}
		}
		
		template <typename... Arguments>
		void log(const char* s, Arguments... a) {
			entries[index] = {/*Time::time()*/ 0, s, {a...}};
			
			if(++index >= num_entries) {
				for(;;);
				index = 0;
			}
		}
};

#endif
