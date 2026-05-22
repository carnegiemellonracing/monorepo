# Compute

Python codebase for the Compute board: control loop, CAN I/O, gear logic,
and unit conversions between vehicle speed / wheel RPM / motor eRPM.

## Layout

| File / dir         | Purpose                                                |
| ------------------ | ------------------------------------------------------ |
| `main.py`          | Entry point — opens CAN, builds the control loop      |
| `controller.py`    | Top-level control loop                                 |
| `car.py`           | Car state container                                    |
| `driver_input.py`  | Pedal / driver inputs                                  |
| `motor_req.py`     | Motor request packets                                  |
| `motor_state.py`   | Motor state tracking                                   |
| `speed.py`         | Speed class — converts between m/s, mph, wheel/motor RPM, eRPM |
| `constants.py`     | Gear ratio, wheel radius, pole pairs, etc.             |
| `dti_protocol.py`  | DTI inverter protocol                                  |
| `kvaser_can.py`    | Kvaser CAN bus wrapper                                 |
| `gears/`           | Per-gear behavior (`neutral`, `fast`, …)               |
| `tests/`           | Pytest suite                                           |

## Getting started

Requires Python 3.11+ and `git`.

```bash
cd Compute
make install
```

This installs the Python dependencies from `requirements.txt` and activates
the project's git hooks (formatting on commit, tests on push). Run it once
per clone.

## Running the app

```bash
python main.py
```

Edit the constants at the top of `main.py` (`CAN_CHANNEL`, `BITRATE`,
`LOG_PATH`, `CONTROL_HZ`) for your hardware before running. Requires a
Kvaser CAN interface to be present.

## Make targets

| Command         | What it does                                        |
| --------------- | --------------------------------------------------- |
| `make install`  | Install deps and activate git hooks (one-time)      |
| `make hooks`    | Re-install pre-commit + pre-push hooks              |
| `make test`     | Run the pytest suite                                |
| `make lint`     | Check formatting with `black` and `isort`           |
| `make format`   | Auto-fix formatting with `black` and `isort`        |
| `make clean`    | Remove `__pycache__` and `.pytest_cache`            |

## Git hooks

`make install` activates [`pre-commit`](https://pre-commit.com) hooks
configured in [`../.pre-commit-config.yaml`](../.pre-commit-config.yaml):

- **On `git commit`** — `black` and `isort` run on staged Python files in
  `Compute/`. The commit is blocked if either reports changes; run
  `make format` and re-stage to proceed.
- **On `git push`** — the pytest suite runs. Push is blocked if any test
  fails.

To bypass in an emergency: `git commit --no-verify` or
`git push --no-verify`. The same checks run in CI
([Python Lint workflow](../.github/workflows/python-lint.yml),
[Python Test workflow](../.github/workflows/python-test.yml)), so anything
bypassed locally will still be caught on the PR.

## Tests

```bash
make test           # full suite
pytest tests/test_speed.py -v    # one file
pytest -k mph -v                 # by keyword
```

Test discovery is configured via `tests/conftest.py`, which adds the
`Compute/` directory to `sys.path` so tests can `import speed`, `import
constants`, etc. without packaging boilerplate.
