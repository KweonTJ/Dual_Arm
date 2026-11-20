#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
runtime_python="$PWD/.venv-hardware/bin/python"
if [[ ! -x "$runtime_python" ]]; then
  printf '%s\n' 'Install src/dual_arm_hardware/requirements-lowlevel.txt into .venv-hardware first.' >&2
  exit 2
fi
hardware_site="$("$runtime_python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export PYTHONPATH="$hardware_site${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 src/dual_arm_hardware/scripts/manual_app.py "$@"
