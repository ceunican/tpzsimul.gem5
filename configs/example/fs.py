# Copyright (c) 2010-2012 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Ali Saidi

import optparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')

from FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
import CacheConfig
from Caches import *
import Options

parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# driver system CPU is always simple... note this is an assignment of
# a class, not an instance.
DriveCPUClass = AtomicSimpleCPU
drive_mem_mode = 'atomic'

# Check if KVM support has been enabled, we might need to do VM
# configuration if that's the case.
have_kvm_support = 'BaseKvmCPU' in globals()
def is_kvm_cpu(cpu_class):
    return have_kvm_support and cpu_class != None and \
        issubclass(cpu_class, BaseKvmCPU)

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

TestCPUClass.clock = options.cpu_clock
DriveCPUClass.clock = options.cpu_clock

# Match the memories with the CPUs, the driver system always simple,
# and based on the options for the test system
DriveMemClass = SimpleMemory
TestMemClass = Simulation.setMemClass(options)

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)
else:
    if options.dual:
        bm = [SysConfig(disk=options.disk_image, mem=options.mem_size), SysConfig(disk=options.disk_image, mem=options.mem_size)]
    else:
        bm = [SysConfig(disk=options.disk_image, mem=options.mem_size)]

np = options.num_cpus

if buildEnv['TARGET_ISA'] == "alpha":
    test_sys = makeLinuxAlphaSystem(test_mem_mode, TestMemClass, bm[0])
elif buildEnv['TARGET_ISA'] == "mips":
    test_sys = makeLinuxMipsSystem(test_mem_mode, TestMemClass, bm[0])
elif buildEnv['TARGET_ISA'] == "sparc":
    test_sys = makeSparcSystem(test_mem_mode, TestMemClass, bm[0])
elif buildEnv['TARGET_ISA'] == "x86":
    test_sys = makeLinuxX86System(test_mem_mode, TestMemClass,
                                  options.num_cpus, bm[0])
elif buildEnv['TARGET_ISA'] == "arm":
    test_sys = makeArmSystem(test_mem_mode, options.machine_type,
                             TestMemClass, bm[0], options.dtb_filename,
                             bare_metal=options.bare_metal)
else:
    fatal("Incapable of building %s full system!", buildEnv['TARGET_ISA'])

test_sys.clock = options.sys_clock

if options.kernel is not None:
    test_sys.kernel = binary(options.kernel)

if options.script is not None:
    test_sys.readfile = options.script

test_sys.init_param = options.init_param

test_sys.cpu = [TestCPUClass(cpu_id=i) for i in xrange(np)]

if is_kvm_cpu(TestCPUClass) or is_kvm_cpu(FutureClass):
    test_sys.vm = KvmVM()

if options.caches or options.l2cache:
    # By default the IOCache runs at the system clock
    test_sys.iocache = IOCache(addr_ranges = test_sys.mem_ranges)
    test_sys.iocache.cpu_side = test_sys.iobus.master
    test_sys.iocache.mem_side = test_sys.membus.slave
else:
    test_sys.iobridge = Bridge(delay='50ns', ranges = test_sys.mem_ranges)
    test_sys.iobridge.slave = test_sys.iobus.master
    test_sys.iobridge.master = test_sys.membus.slave

# Sanity check
if options.fastmem:
    if TestCPUClass != AtomicSimpleCPU:
        fatal("Fastmem can only be used with atomic CPU!")
    if (options.caches or options.l2cache):
        fatal("You cannot use fastmem in combination with caches!")

for i in xrange(np):
    if options.fastmem:
        test_sys.cpu[i].fastmem = True
    if options.checker:
        test_sys.cpu[i].addCheckerCpu()
    test_sys.cpu[i].createThreads()

CacheConfig.config_cache(options, test_sys)

if len(bm) == 2:
    if buildEnv['TARGET_ISA'] == 'alpha':
        drive_sys = makeLinuxAlphaSystem(drive_mem_mode, DriveMemClass, bm[1])
    elif buildEnv['TARGET_ISA'] == 'mips':
        drive_sys = makeLinuxMipsSystem(drive_mem_mode, DriveMemClass, bm[1])
    elif buildEnv['TARGET_ISA'] == 'sparc':
        drive_sys = makeSparcSystem(drive_mem_mode, DriveMemClass, bm[1])
    elif buildEnv['TARGET_ISA'] == 'x86':
        drive_sys = makeX86System(drive_mem_mode, DriveMemClass, np, bm[1])
    elif buildEnv['TARGET_ISA'] == 'arm':
        drive_sys = makeArmSystem(drive_mem_mode, options.machine_type,
                                  DriveMemClass, bm[1])

    drive_sys.clock = options.sys_clock

    drive_sys.cpu = DriveCPUClass(cpu_id=0)
    drive_sys.cpu.createThreads()
    drive_sys.cpu.createInterruptController()
    drive_sys.cpu.connectAllPorts(drive_sys.membus)
    if options.fastmem:
        drive_sys.cpu.fastmem = True
    if options.kernel is not None:
        drive_sys.kernel = binary(options.kernel)

    if is_kvm_cpu(DriveCPUClass):
        drive_sys.vm = KvmVM()

    drive_sys.iobridge = Bridge(delay='50ns',
                                ranges = drive_sys.mem_ranges)
    drive_sys.iobridge.slave = drive_sys.iobus.master
    drive_sys.iobridge.master = drive_sys.membus.slave

    drive_sys.init_param = options.init_param
    root = makeDualRoot(True, test_sys, drive_sys, options.etherdump)
elif len(bm) == 1:
    root = Root(full_system=True, system=test_sys)
else:
    print "Error I don't know how to create more than 2 systems."
    sys.exit(1)

if options.timesync:
    root.time_sync_enable = True

if options.frame_capture:
    VncServer.frame_capture = True

Simulation.setWorkCountOptions(test_sys, options)
Simulation.run(options, root, test_sys, FutureClass)
