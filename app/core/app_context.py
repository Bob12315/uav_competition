from __future__ import annotations

from dataclasses import dataclass
from logging import Logger

from app.config import AppConfig


@dataclass
class AppContext:
    config: AppConfig
    logger: Logger
