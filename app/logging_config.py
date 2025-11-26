from __future__ import annotations

import logging
from logging import Logger

from .config import AppConfig


def setup_logging(config: AppConfig) -> Logger:
    logger = logging.getLogger("uav_comp")
    logger.setLevel(getattr(logging, config.logging.level.upper(), logging.INFO))
    logger.propagate = False

    class _NoDxDyFilter(logging.Filter):
        def filter(self, record: logging.LogRecord) -> bool:
            msg = record.getMessage()
            return ("dx=" not in msg) and ("dy=" not in msg)

    ch = logging.StreamHandler()
    ch.setLevel(logger.level)
    fmt = logging.Formatter(
        "[%(asctime)s] [%(levelname)s] %(name)s - %(message)s"
    )
    ch.setFormatter(fmt)
    ch.addFilter(_NoDxDyFilter())
    logger.addHandler(ch)

    logger.debug("Logger initialized.")
    return logger
