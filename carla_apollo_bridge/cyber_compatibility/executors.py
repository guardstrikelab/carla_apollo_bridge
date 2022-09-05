#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

class Executor(object):
    def add_node(self, node):
        pass

class SingleThreadedExecutor(Executor):
    pass

class MultiThreadedExecutor(Executor):
    pass
