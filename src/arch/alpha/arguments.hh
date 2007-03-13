/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 * Authors: Nathan Binkert
 */

#ifndef __ARCH_ALPHA_ARGUMENTS_HH__
#define __ARCH_ALPHA_ARGUMENTS_HH__

#include <assert.h>

#include "arch/alpha/vtophys.hh"
#include "base/refcnt.hh"
#include "mem/vport.hh"
#include "sim/host.hh"

class ThreadContext;

namespace AlphaISA {

class Arguments
{
  protected:
    ThreadContext *tc;
    int number;
    uint64_t getArg(bool fp = false);

  protected:
    class Data : public RefCounted
    {
      public:
        Data(){}
        ~Data();

      private:
        std::list<char *> data;

      public:
        char *alloc(size_t size);
    };

    RefCountingPtr<Data> data;

  public:
    Arguments(ThreadContext *ctx, int n = 0)
        : tc(ctx), number(n), data(NULL)
        { assert(number >= 0); data = new Data;}
    Arguments(const Arguments &args)
        : tc(args.tc), number(args.number), data(args.data) {}
    ~Arguments() {}

    ThreadContext *getThreadContext() const { return tc; }

    const Arguments &operator=(const Arguments &args) {
        tc = args.tc;
        number = args.number;
        data = args.data;
        return *this;
    }

    Arguments &operator++() {
        ++number;
        assert(number >= 0);
        return *this;
    }

    Arguments operator++(int) {
        Arguments args = *this;
        ++number;
        assert(number >= 0);
        return args;
    }

    Arguments &operator--() {
        --number;
        assert(number >= 0);
        return *this;
    }

    Arguments operator--(int) {
        Arguments args = *this;
        --number;
        assert(number >= 0);
        return args;
    }

    const Arguments &operator+=(int index) {
        number += index;
        assert(number >= 0);
        return *this;
    }

    const Arguments &operator-=(int index) {
        number -= index;
        assert(number >= 0);
        return *this;
    }

    Arguments operator[](int index) {
        return Arguments(tc, index);
    }

    template <class T>
    operator T() {
        assert(sizeof(T) <= sizeof(uint64_t));
        T data = static_cast<T>(getArg());
        return data;
    }

    template <class T>
    operator T *() {
        T *buf = (T *)data->alloc(sizeof(T));
        CopyData(tc, buf, getArg(), sizeof(T));
        return buf;
    }

    operator char *() {
        char *buf = data->alloc(2048);
        CopyStringOut(tc, buf, getArg(), 2048);
        return buf;
    }
};

}; // namespace AlphaISA

#endif // __ARCH_ALPHA_ARGUMENTS_HH__
