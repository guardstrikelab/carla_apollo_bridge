#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import logging
import sys
from logging.handlers import TimedRotatingFileHandler

FORMATTER = logging.Formatter("[%(asctime)s] [%(name)s] [%(levelname)s]: %(message)s")
LOG_FILE = "carla_bridge.log"


def get_console_handler():
   console_handler = logging.StreamHandler(sys.stdout)
   console_handler.setFormatter(FORMATTER)
   return console_handler

def get_file_handler():
   file_handler = TimedRotatingFileHandler(LOG_FILE, when='midnight')
   file_handler.setFormatter(FORMATTER)
   return file_handler

def get_logger(logger_name):
   logger = logging.getLogger(logger_name)
   logger.setLevel(logging.DEBUG) # better to have too much log than not enough
   logger.addHandler(get_console_handler())
   logger.addHandler(get_file_handler())
   # with this pattern, it's rarely necessary to propagate the error up to parent
   logger.propagate = False
   return logger

logger = get_logger("carla_bridge")

def logdebug(msg):
    logger.debug(msg)

def loginfo(msg):
    logger.info(msg)

def logwarn(msg):
    logger.warning(msg)

def logerr(msg):
    logger.error(msg)

def logfatal(msg):
    logger.critical(msg)
