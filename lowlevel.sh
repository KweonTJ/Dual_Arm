#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
runtime_python="${DUAL_ARM_PYTHON:-$PWD/.venv-hardware/bin/python}"
if [[ ! -x "$runtime_python" ]]; then runtime_python=/usr/bin/python3; fi
runtime_site="$("$runtime_python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export PYTHONPATH="$runtime_site${PYTHONPATH:+:$PYTHONPATH}"
exec "$runtime_python" src/dual_arm_hardware/scripts/lowlevel_cli.py "${@:-demo}"
