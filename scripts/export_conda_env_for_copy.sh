#!/usr/bin/env bash
# Export a conda environment to a portable YAML you can copy to another machine.
#
# On the machine that already has the env:
#   source ~/miniconda3/etc/profile.d/conda.sh   # or anaconda3, micromamba, etc.
#   bash scripts/export_conda_env_for_copy.sh piper_new piper_new_portable.yml
#
# On the new machine (Linux/macOS with conda installed):
#   conda env create -f piper_new_portable.yml
#   conda activate piper_new
#
# Notes:
# - Uses --no-builds so builds are not tied to one OS/CPU stack when possible.
# - Strips the `prefix:` line so the path from this machine is not embedded.
# - Pip packages installed inside the env are included in the export.
set -euo pipefail

ENV_NAME="${1:-piper_new}"
OUT="${2:-piper_new_portable_environment.yml}"

if ! command -v conda >/dev/null 2>&1; then
  echo "conda not on PATH. Source conda first, e.g.:" >&2
  echo "  source \"\$HOME/miniconda3/etc/profile.d/conda.sh\"" >&2
  exit 1
fi

if ! conda env list | awk '!/^#/ && NF {print $1}' | grep -qx "$ENV_NAME"; then
  echo "Environment \"$ENV_NAME\" not found. Available envs:" >&2
  conda env list >&2
  exit 1
fi

conda env export -n "$ENV_NAME" --no-builds | sed '/^prefix:/d' >"$OUT"

echo "Wrote: $OUT"
echo "Copy that file to the other machine, then:"
echo "  conda env create -f $OUT"
echo "  conda activate $ENV_NAME"
