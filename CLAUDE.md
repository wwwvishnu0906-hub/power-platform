# CLAUDE.md — Multi-Project Workspace

This repository is a multi-discipline workspace supporting Python, Web (HTML/CSS/JS), and Arduino/IoT development. Each project type lives in its own top-level directory and follows the conventions defined here.

---

## Repository Layout

```
/
├── python/       # Python scripts, packages, and services
├── web/          # HTML/CSS/JS frontends and static sites
├── arduino/      # Arduino sketches and embedded/IoT firmware
├── docs/         # Shared documentation and specs
├── scripts/      # Utility/automation scripts (bash, python, etc.)
├── tests/        # Cross-project integration tests
└── CLAUDE.md     # This file
```

Each subdirectory should contain its own `README.md` explaining its purpose and any project-specific setup.

---

## Git Protocol

- **Branch naming:** `<type>/<short-description>` — e.g. `feat/sensor-mqtt`, `fix/login-redirect`, `chore/update-deps`
- **Commit message format:** `<type>(<scope>): <subject>` — e.g. `feat(arduino): add DHT22 temperature reader`
  - Types: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`
  - Scope: the subdirectory or component (`python`, `web`, `arduino`, `ci`, etc.)
- **PRs:** Open against `main`. Describe what changed, why, and how to test it.
- **Never force-push to `main`.** Use feature branches; squash or rebase before merging.
- **Keep commits atomic.** One logical change per commit. Avoid mixing unrelated changes.
- **Tag releases** with semver: `v1.2.3`. Tag format: `python/v1.0.0`, `firmware/v0.3.1` for per-project versioning.

---

## Python

### Project Layout

```
python/
└── <project-name>/
    ├── src/
    │   └── <package>/
    ├── tests/
    ├── pyproject.toml    # or setup.cfg
    ├── requirements.txt
    └── README.md
```

### Environment Setup

```bash
# Create virtual environment (per project)
python -m venv .venv
source .venv/bin/activate          # Linux/macOS
.venv\Scripts\activate             # Windows

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt   # dev/test extras

# Install project in editable mode
pip install -e .
```

### Coding Conventions

- **Python version:** 3.10+ unless constrained by hardware.
- **Formatter:** `black` (line length 88). Run before every commit.
- **Linter:** `ruff` for fast lint checks. Config in `pyproject.toml`.
- **Type hints:** Required on all public functions and class methods.
- **Docstrings:** Google style for all public APIs.
- **Imports:** `isort` ordering — stdlib, third-party, local. Enforced by `ruff`.
- Do not use `print()` in library code; use the `logging` module.
- Prefer `pathlib.Path` over `os.path`.
- Never hardcode secrets. Use environment variables or `.env` files (via `python-dotenv`).

### Testing (Python)

- **Framework:** `pytest`
- **Coverage target:** 80% minimum; 90%+ for critical paths.
- **Test naming:** `test_<unit>_<scenario>.py` — e.g. `test_parser_empty_input.py`
- Unit tests live in `<project>/tests/`. Integration tests go in the root `tests/` folder.

```bash
# Run all tests
pytest python/<project>/tests/ -v

# With coverage
pytest --cov=src --cov-report=term-missing

# Run a specific test file
pytest python/<project>/tests/test_foo.py
```

### Useful Commands (Python)

```bash
black python/                        # Format all Python code
ruff check python/ --fix             # Lint and auto-fix
mypy python/<project>/src/           # Static type checking
pip list --outdated                  # Check for stale dependencies
pip freeze > requirements.txt        # Lock current deps
```

---

## Web (HTML / CSS / JS)

### Project Layout

```
web/
└── <project-name>/
    ├── index.html
    ├── css/
    ├── js/
    ├── assets/
    ├── package.json       # if using npm tooling
    └── README.md
```

### Coding Conventions

- **HTML:** Use semantic elements (`<main>`, `<article>`, `<nav>`, `<section>`). Always include `lang` on `<html>`. Close all tags properly.
- **CSS:**
  - Use CSS custom properties (variables) for colors, spacing, and typography.
  - BEM naming convention for class names: `.block__element--modifier`.
  - Mobile-first media queries. No inline styles in production code.
  - Avoid `!important` except in utility/reset layers.
- **JavaScript:**
  - ES2020+ syntax. Use `const`/`let`; never `var`.
  - Modules (`type="module"`) over global scripts where supported.
  - No framework unless justified — prefer vanilla JS for lightweight projects.
  - If using a framework (React, Vue, Svelte): document the choice in `README.md`.
  - Async/await over raw `.then()` chains.
  - No `console.log` in committed code (use a logger or remove before PR).
- **Accessibility:** All images need `alt`. Interactive elements need keyboard focus. Validate with axe or Lighthouse.
- **Performance:** Lazy-load images. Minify CSS/JS for production builds.

### Tooling (if using npm)

```bash
npm install             # Install dependencies
npm run dev             # Start dev server (e.g. Vite, Parcel)
npm run build           # Production build
npm run lint            # ESLint
npm run format          # Prettier
npm test                # Run tests
```

### Testing (Web)

- **Unit tests:** Vitest or Jest for JS logic.
- **E2E tests:** Playwright or Cypress — place specs in `web/<project>/tests/` or root `tests/e2e/`.
- **Manual checklist:** Test in Chrome, Firefox, Safari, and mobile viewport before PR.
- Lighthouse score targets: Performance 90+, Accessibility 95+, Best Practices 90+.

```bash
npx playwright test                  # E2E tests
npx eslint web/<project>/js/         # Lint JS
npx prettier --write web/<project>/  # Format all web files
npx lighthouse http://localhost:3000 # Audit (requires running server)
```

---

## Arduino / IoT (Embedded)

### Project Layout

```
arduino/
└── <project-name>/
    ├── <project-name>.ino     # Main sketch
    ├── src/                   # Supporting .cpp / .h files
    ├── lib/                   # Local libraries (not managed by Library Manager)
    ├── data/                  # SPIFFS/LittleFS filesystem data
    ├── platformio.ini         # PlatformIO config (preferred over Arduino IDE)
    └── README.md              # Target board, pinout diagram, wiring notes
```

### Coding Conventions

- **Language:** C++17 (PlatformIO default for modern boards).
- **Naming:** `camelCase` for variables and functions; `UPPER_SNAKE_CASE` for constants and macros; `PascalCase` for classes and structs.
- **`#define` vs `const`:** Prefer `const` or `constexpr` over `#define` for typed constants.
- **No `delay()`** in production code — use non-blocking patterns (`millis()`-based state machines).
- **Pin definitions:** Define all pins as `const uint8_t` at the top of the sketch or in a `pins.h` header. Never use bare numbers inside logic.
- **Serial output:** Wrap all `Serial.print` calls in `#ifdef DEBUG` guards so they compile out in production builds.
- **Memory:** Be explicit about RAM usage. Prefer `PROGMEM` for large string/table constants on AVR. Always check heap fragmentation on ESP8266/ESP32.
- **Error handling:** Check return values from sensors and peripheral calls. Use a watchdog timer where appropriate.
- **Libraries:** Pin library versions in `platformio.ini`. Document why each library is used.

### PlatformIO Setup

```bash
# Install PlatformIO CLI
pip install platformio

# Initialize a new project (from arduino/<project>/)
pio project init --board uno          # or esp32dev, nano33ble, etc.

# Build
pio run

# Upload to board
pio run --target upload

# Open serial monitor
pio device monitor --baud 115200

# Run unit tests (native or on-device)
pio test
```

### Testing (Arduino/IoT)

- **Unit tests:** Use the PlatformIO `test` framework with native environment for pure-logic tests (no hardware dependency).
- **Hardware-in-the-loop (HIL):** Document required hardware setup in `README.md`. Tag HIL tests with `[hardware]`.
- **Simulation:** Use Wokwi (wokwi.com) for circuit simulation. Include `wokwi.toml` and `diagram.json` if available.
- Keep hardware-dependent tests isolated so CI can run native tests without a board connected.

```bash
# Run native (no hardware) unit tests
pio test -e native

# Run all on-device tests (board must be connected)
pio test -e uno

# Check code style (cpplint or clang-format)
clang-format -i arduino/<project>/src/*.cpp arduino/<project>/src/*.h
```

### Useful Commands (Arduino/IoT)

```bash
pio boards                           # List supported boards
pio lib search "<library-name>"      # Search PlatformIO library registry
pio lib install "<lib>@<version>"    # Install specific library version
pio check                            # Static analysis (cppcheck)
pio run --target clean               # Clean build artifacts
pio system info                      # Environment and Python info
```

---

## Documentation (`docs/`)

- All documentation is written in Markdown.
- File names: `kebab-case.md`.
- Each major feature or subsystem should have its own doc file.
- Diagrams: prefer Mermaid (rendered by GitHub) embedded in Markdown. Use draw.io `.drawio` files for complex schematics saved in `docs/diagrams/`.
- API docs for Python: generated with `pdoc` or `mkdocs`.
- Hardware pinout tables belong in `arduino/<project>/README.md` and referenced from `docs/`.

---

## Scripts (`scripts/`)

- Shell scripts: POSIX-compatible `#!/bin/sh` unless bash-specific features are needed, in which case use `#!/usr/bin/env bash` and set `-euo pipefail`.
- Python scripts: follow Python conventions above; include a `if __name__ == "__main__":` guard.
- All scripts must be executable (`chmod +x`) and begin with a comment block explaining usage.
- Scripts that interact with hardware or external services must have a `--dry-run` flag where applicable.

---

## Cross-Project Integration Tests (`tests/`)

- Integration tests that span more than one project type (e.g. a Python script that consumes data from an Arduino over serial and displays it on a web dashboard) live here.
- Use `pytest` as the test runner for mixed-language integration tests.
- Document hardware/service prerequisites in `tests/README.md`.
- Tag slow or hardware-dependent tests so they can be excluded in CI: `@pytest.mark.slow`, `@pytest.mark.hardware`.

---

## CI/CD

- All pushes to `main` must pass linting, formatting checks, and unit tests.
- Recommended GitHub Actions matrix:
  - **Python:** lint (`ruff`), format (`black --check`), test (`pytest`), type check (`mypy`)
  - **Web:** lint (`eslint`), format (`prettier --check`), unit test (`vitest`), build check
  - **Arduino:** build sketch via PlatformIO, run native unit tests
- Hardware-dependent and E2E tests run on demand (workflow_dispatch) or nightly, not on every push.

---

## Environment Variables and Secrets

- Never commit secrets, API keys, tokens, or passwords.
- Use `.env` files locally (add `.env` to `.gitignore`).
- Provide a `.env.example` with all required keys and placeholder values.
- For Arduino/IoT: use a `secrets.h` file (gitignored) and provide `secrets.h.example`.
- In CI, use repository secrets or a secrets manager.

---

## Dependency Management

| Domain  | Tool                  | Lock file              |
|---------|-----------------------|------------------------|
| Python  | pip / pip-tools       | `requirements.txt`     |
| Web     | npm / pnpm            | `package-lock.json` / `pnpm-lock.yaml` |
| Arduino | PlatformIO            | `platformio.ini` (pinned versions) |

Update dependencies deliberately, not automatically. Test after any dependency bump.

---

## Quick Reference

```bash
# Python
black python/ && ruff check python/ --fix
pytest python/ -v --cov=src

# Web
npm run lint && npm run format && npm test

# Arduino
pio run && pio test -e native

# Git
git checkout -b feat/<scope>/<description>
git commit -m "feat(<scope>): <subject>"
```
