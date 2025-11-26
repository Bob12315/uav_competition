# Repository Guidelines

## Project Structure & Modules
- Entry point: `main.py` ties fake FC, vision, and mission loop for the desktop demo.
- Configuration: `config.yaml` and `app/config.py` load runtime parameters; `app/logging_config.py` centralizes logging.
- Core types/context: `app/core/app_context.py`, `app/core/models.py`.
- Functional modules: `app/io` (MAVLink clients, RC, gimbal, dropper), `app/vision` (camera + detection), `app/mission` (state machine, align control), `app/telemetry` (CSV logging).
- Simulators: `sims/sim_vision.py`, `sims/sim_mission.py` for module-level dry runs.

## Setup, Build, and Run
- Python 3.10+ recommended. Install demo deps: `pip install opencv-python numpy`.
- Run full desktop demo: `python main.py` (ESC to exit).
- Module demos: `python sims/sim_vision.py`, `python sims/sim_mission.py` to validate each subsystem independently before hardware.
- When adding packages, prefer `pip install ...` then document them; keep optional hardware extras behind import guards.

## Coding Style & Naming
- Follow PEP 8, 4-space indents, and type hints for public interfaces.
- Dataclasses for shared payloads (see `app/core/models.py`); avoid ad-hoc dicts between modules.
- ClassNames in `CamelCase`, functions/vars in `snake_case`, constants in `UPPER_SNAKE`.
- Keep module boundaries clean: IO does MAVLink only; Mission decides; Vision detects; HUD renderer will be reintroduced later; no cross-importing private helpers.

## Testing Guidelines
- Prefer fast sims first: adapt `sims/sim_*` scripts to cover new states or data flows when adding features.
- Add lightweight unit-style checks for pure logic (e.g., mission transitions, alignment math) near the module under test or in a `tests/` folder if created.
- When touching real hardware paths, include a fallback mock or flag to keep CI/local runs headless.

## Commit & Pull Request Practices
- Use concise Conventional Commit-style prefixes when possible (`feat:`, `fix:`, `refactor:`, `docs:`).
- Describe behavior changes and risks; include how you validated (commands run, sims used).
- Keep PRs scoped: one feature or fix; link issues/tasks; add screenshots or logs for vision changes.

## Security & Configuration Tips
- Do not commit real MAVLink endpoints, RC channel mappings, or credentials; keep them in `config.yaml` overlays or environment variables.
- Guard hardware actions (arming, servo, dropper) behind explicit flags/modes; default to safe no-op when unavailable.
- Log responsibly: avoid writing large frames by default; prefer CSV summaries via telemetry logger.
