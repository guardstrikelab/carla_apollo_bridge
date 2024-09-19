#!/bin/bash

ps -ef | grep oasis_bridge.py | grep -v 'grep' | awk '{print $2}' | xargs kill -9
