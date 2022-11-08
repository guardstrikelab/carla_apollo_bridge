#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

class CyberException(Exception):
    pass

class CyberInterruptException(Exception):
    pass

class ServiceException(Exception):
    pass
