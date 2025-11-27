from __future__ import annotations

import logging
from logging import Logger
from pathlib import Path

from .config import AppConfig, PROJECT_ROOT


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

    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    fh = logging.FileHandler(Path(log_dir) / "uav.log", encoding="utf-8")
    fh.setLevel(logger.level)
    fh.setFormatter(fmt)
    logger.addHandler(fh)

    logger.debug("Logger initialized.")
    return logger
