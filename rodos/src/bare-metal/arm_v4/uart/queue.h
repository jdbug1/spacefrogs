// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2010, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Roboterclub Aachen e.V. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef	XPCC_ATOMIC__QUEUE_HPP
#define	XPCC_ATOMIC__QUEUE_HPP

#include <cstddef>
#include <stdint.h>
#include "accessor.h"

namespace xpcc
{
	namespace atomic
	{
		/**
		 * @brief	Interrupt save queue
		 * @author	Fabian Greif
		 */
		template<typename T,
				 std::size_t N>
		class Queue
		{
			typedef uint_fast8_t IndexType;
			
		public:
			Queue();
			
			bool
			isFull();
			
			bool
			isEmpty();
			
			IndexType
			getMaxSize();
			
			const T&
			get() const;
			
			/**
			 * @brief	Append something
			 * 
			 * @return	\c false if the queue was full, \c true otherwise
			 */
			bool
			push(const T& value);
			
			void
			pop();
	
		private:
			IndexType head;
			IndexType tail;
			
			T buffer[N+1];
		};
	}
}

#include "queue_impl.h"

#endif	// XPCC_ATOMIC__QUEUE_HPP
