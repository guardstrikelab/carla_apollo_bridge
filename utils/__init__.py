#!/usr/bin/env python

import pathlib
from utils.logger import global_config

root_dir = pathlib.Path(__file__).resolve().parent.parent

log = global_config("oasis_bridge", f"{root_dir}/logs", "INFO")

__all__ = ["log"]
