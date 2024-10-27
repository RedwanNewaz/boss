#!/usr/bin/env python

import os
from waflib import Task, TaskGen, Logs, Utils
from waflib.Tools import c_preproc, cxx
PCH_COMPILER_OPTIONS = {
	'clang++': [['-include'], '.pch', ['-x', 'c++-header']],
	'g++':     [['-include'], '.gch', ['-x', 'c++-header']],
}
def configure(conf):
    conf.load('pch')
    # conf.load('xcode')
    if (conf.options.with_pch and conf.env['COMPILER_CXX'] in PCH_COMPILER_OPTIONS.keys()):
        if Utils.unversioned_sys_platform() == "darwin" and conf.env['CXX_NAME'] == 'clang':
            version = tuple(int(i) for i in conf.env['CC_VERSION'])
            if version < (6, 1, 0):
                # Issue #2804
                return
        conf.env.WITH_PCH = True
        flags = PCH_COMPILER_OPTIONS[conf.env['COMPILER_CXX']]
        conf.env.CXXPCH_F = flags[0]
        conf.env.CXXPCH_EXT = flags[1]
        conf.env.CXXPCH_FLAGS = flags[2]

def options(opt):
    opt.add_option('--with-pch', action='store_true', default=False, help='Enable precompiled headers')
    opt.load('pch', tooldir='.waf-tools')
    # opt.load('xcode')

def build(bld):
    bld(features='cxx cxxprogram pch xcode',
        source='boss2.cpp',
        headers='boss_pch.h', 
        includes='. ../../src /usr/local/include',
        target='boss2',
        uselib='BOOST EIGEN TBB LIBCMAES NLOPT YAML-CPP FCL LIBCCD PCH',
        libpath=['/usr/local/lib', '/opt/local/lib'],  # Adjust this path
        rpath=['/usr/local/lib', '/opt/local/lib'],
        lib=['yaml-cpp', 'fcl', 'ccd'],  # Adjust this path
        use='limbo')
