#!/usr/bin/env python
"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2022 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""
import loguru


def init_log(
    log_file: str, log_level="DEBUG", log_size="10 MB", keep_log_days="7 days"
):
    # sink = log_file + "_{time}.log"
    sink = log_file
    level = log_level
    rotation = log_size  # must be "10 MB" format
    retention = keep_log_days
    loguru.logger.add(
        sink=sink, level=level, rotation=rotation, retention=retention, enqueue=True
    )
    return loguru.logger


def demo():
    log = init_log("demo")
    log.debug("this is a debug message")
    log.info("this is another info message")
    log.warning("this is another warning message")
    log.error("this is another error message")
    log.success("this is success message!")
    log.critical("this is critical message!")


if __name__ == "__main__":
    demo()
