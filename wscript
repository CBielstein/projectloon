# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('projectloon', ['core'])
    module.source = [
        'model/projectloon.cc',
        'helper/projectloon-helper.cc',
        ]

    module_test = bld.create_ns3_module_test_library('projectloon')
    module_test.source = [
        'test/projectloon-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'projectloon'
    headers.source = [
        'model/projectloon.h',
        'helper/projectloon-helper.h',
        ]

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    bld.recurse('src');

    # bld.ns3_python_bindings()

