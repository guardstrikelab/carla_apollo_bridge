"""
/*******************************************************************************
 * SYNKROTRON Confidential
 * Copyright (C) 2023 SYNKROTRON Inc. All rights reserved.
 * The source code for this program is not published
 * and protected by copyright controlled
 *******************************************************************************/
"""
import os
import sys
import loguru
import orjson
import pprint
import pathlib
import platform
import datetime


def _set_extras(record):
    """
    record 对象是由 loguru 在内部创建的，不需要在代码中显式创建它。只需要在配置日志处理器时指定如何使用这个对象。
    """
    utc_now = datetime.datetime.now(datetime.timezone.utc)
    # 转换为本地时区时间
    local_now = utc_now.astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    record["extra"]["datetime"] = local_now
    record["extra"]["host_name"] = os.getenv("HOSTNAME", os.getenv("HOSTNAME", platform.node())).split(".")[0]
    record["extra"]["pid"] = os.getpid()


def _format_record(record: dict) -> str:
    """
    Return a custom format for loguru loggers.
    """
    format_string = "<green>{extra[datetime]}</green> | "
    # format_string += "<green>{extra[host_name]}</green> | "
    # format_string += "<green>{extra[host]}</green> | "
    format_string += "<green>{extra[pid]}</green> | "
    format_string += "<level>{level: <8}</level> | "
    format_string += "<cyan>{name}</cyan>:"
    format_string += "<cyan>{file}</cyan>: "
    # format_string += "<cyan>{function}</cyan>: "
    format_string += "<cyan>{line}</cyan> | "
    format_string += "<level>{message}</level>"

    # 针对含有 payload 的记录，使用 pprint 格式化输出
    # logger.bind(payload=data_object).info("Received data")
    if record["extra"].get("payload") is not None:
        record["extra"]["payload"] = pprint.pformat(record["extra"]["payload"], indent=4, compact=True, width=88)
        format_string += "\n<level>{extra[payload]}</level>"

    format_string += "{exception}\n"
    return format_string


def _orjson_log_sink(msg):
    r = msg.record
    rec = {
        "datetime": r["extra"].get("datetime", ""),
        "host": r["extra"].get("host", ""),
        "pid": r["extra"].get("pid", ""),
        "level": r["level"].name,
        "message": r["message"],
        "line": r["line"],
        "exception": str(r["exception"]) if r["exception"] else "",
        "extra": {k: v for k, v in r["extra"].items() if k not in ("datetime", "host", "pid", "message", "exception")},
        "process": {"id": r["process"].id, "name": r["process"].name},
        "thread": {"id": r["thread"].id, "name": r["thread"].name},
        "file": r["file"].path,
    }
    rec_str = orjson.dumps(rec, default=str).decode("utf-8")
    print(rec_str)


# 定义日志文件的分割策略
def _rotate_policy(msg, file):
    # 20MB 文件大小阈值
    rotation_size = 20480000
    # 时间分割规则（按天）
    rotation_time = datetime.datetime.now().strftime("%Y-%m-%d")

    # 获取当前文件大小
    size = file.tell()
    # 文件大小超过阈值，需要进行文件分割
    if size > rotation_size:
        return True

    # 比较当前时间与上次分割时间, 到达时间间隔，需要进行文件分割
    last_rotation_time = getattr(file, "last_rotation_time", None)
    if last_rotation_time is None:
        file.last_rotation_time = rotation_time
        return False
    elif rotation_time != last_rotation_time:
        file.last_rotation_time = rotation_time
        return True

    return False


def _filter_apscheduler(record):
    if "apscheduler" in record["name"]:
        return False
    return True


def global_config(name, path, log_level="INFO", is_json=False):
    # 确保日志目录存在
    pathlib.Path(path).mkdir(parents=True, exist_ok=True)

    # TODO: json 目前无法写到文件中，只能输出到控制台，待解决
    json_handlers = [
        {
            "sink": _orjson_log_sink,
            "level": log_level,
            "enqueue": True,
            "filter": _filter_apscheduler,
            "backtrace": True,
            "diagnose": True,
        }
    ]

    '''
    serialize 指定是否在输出日志消息之前对它们进行序列化。如果设置为True，日志消息将被转换为字符串
    diagnose 当设置为True时，loguru 将在日志记录出现问题时打印诊断信息
    backtrace 如果设置为True，并且日志记录器捕获到异常，loguru 将打印异常的回溯信息
    enqueue 将日志消息放入一个队列中，然后由一个单独的线程或进程来处理这些消息。这种方式可以确保在多进程环境下日志的完整性和顺序性
    '''
    text_handlers = [
                {
                    "sink": sys.stdout,
                    "serialize": False,
                    "format": _format_record,
                    "diagnose": True,
                    "backtrace": True,
                    "enqueue": True,
                    "level": log_level,
                    "filter": _filter_apscheduler,
                },
                {
                    "sink": f"{path}/{name}.log",
                    # "sink": f"{path}/{name}_" + "{time}.log",
                    "rotation": _rotate_policy,
                    "retention": "15 days",
                    "serialize": False,
                    "format": _format_record,
                    "diagnose": True,
                    "backtrace": True,
                    "enqueue": True,
                    "level": log_level,
                    "filter": _filter_apscheduler,
                },
            ]

    loguru.logger.configure(
        patcher=_set_extras,
        handlers=json_handlers if is_json else text_handlers,
    )

    return loguru.logger


if __name__ == "__main__":
    log = global_config("test", "./logs", "DEBUG")
    log.debug("This is a debug message")
    log.info("This is an info message")
    log.warning("This is a warning message")
    log.error("This is an error message")
    log.critical("This is a critical message")
    log.exception("This is an exception message")
    log.bind(payload={"key": "value"}).info("This is a message with a payload")