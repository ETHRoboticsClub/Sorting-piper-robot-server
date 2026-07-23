#!/bin/bash
# One-shot launcher for a HIL-SERL session: CAN bus, learner, actor, tensorboard.
#
# Runs preflight checks, then opens one terminal per process (the actor waits for
# the learner's gRPC port before starting, so ordering is automatic).
#
#   ./scripts/start_hilserl.sh                        # demo-free run (sac_piper_fresh)
#   ./scripts/start_hilserl.sh --config config/sac_piper.json   # seed from demos
#   ./scripts/start_hilserl.sh --check                # preflight only, launch nothing
#
# Options:
#   --config PATH   RL config                 (default config/sac_piper_fresh.json)
#   --no-can        skip CAN bring-up (already up / no robot)
#   --no-tb         skip tensorboard
#   --no-foxglove   skip the live Foxglove view (recordings still written)
#   --fresh         empty replay buffer (default: warm start from past recordings)
#   --fresh-policy  random policy init (default: warm start from newest checkpoint)
#   --check         run preflight checks and exit
#
# Optional env:
#   ENV_NAME    conda env            default piper_hilserl_rl
#   CAN_NAME    CAN interface        default left_piper
#   TB_PORT     tensorboard port     default 6006
#   FG_PORT     foxglove live port   default 8765
#   LEARNER_PORT  learner gRPC port  default 50051

set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT" || exit 1

ENV_NAME="${ENV_NAME:-piper_hilserl_rl}"
CAN_NAME="${CAN_NAME:-left_piper}"
TB_PORT="${TB_PORT:-6006}"
LEARNER_PORT="${LEARNER_PORT:-50051}"
TB_LOGDIR="outputs/tensorboard"

FG_PORT="${FG_PORT:-8765}"

CONFIG="config/sac_piper_fresh.json"
DO_CAN=1
DO_TB=1
DO_FG=1
WARM_START=1
WARM_POLICY=1
CHECK_ONLY=0

while [ $# -gt 0 ]; do
	case "$1" in
	--config)
		CONFIG="$2"
		shift 2
		;;
	--no-can)
		DO_CAN=0
		shift
		;;
	--no-tb)
		DO_TB=0
		shift
		;;
	--no-foxglove)
		DO_FG=0
		shift
		;;
	--fresh)
		WARM_START=0
		shift
		;;
	--fresh-policy)
		WARM_POLICY=0
		shift
		;;
	--check)
		CHECK_ONLY=1
		shift
		;;
	-h | --help)
		awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"
		exit 0
		;;
	*)
		echo "Unknown option: $1 (try --help)" >&2
		exit 1
		;;
	esac
done

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
ok() { printf '  \033[1;32mok\033[0m   %s\n' "$*"; }
warn() { printf '  \033[1;33mwarn\033[0m %s\n' "$*"; }
die() {
	printf '  \033[1;31mfail\033[0m %s\n' "$*" >&2
	exit 1
}

# --- locate conda (do not hardcode a home directory) -------------------------
find_conda_sh() {
	local base
	if [ -n "${CONDA_EXE:-}" ]; then
		base="$(dirname "$(dirname "$CONDA_EXE")")"
		[ -f "$base/etc/profile.d/conda.sh" ] && {
			echo "$base/etc/profile.d/conda.sh"
			return 0
		}
	fi
	for base in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3" /opt/conda; do
		[ -f "$base/etc/profile.d/conda.sh" ] && {
			echo "$base/etc/profile.d/conda.sh"
			return 0
		}
	done
	return 1
}

say "Preflight"

CONDA_SH="$(find_conda_sh)" || die "conda not found (looked at \$CONDA_EXE, ~/miniconda3, ~/anaconda3, ~/miniforge3, /opt/conda)"
ok "conda: $CONDA_SH"

# Conda's hook scripts are not `set -u` clean -- ruby_deactivate.sh tests
# "$ZSH_VERSION", which is unbound under bash and therefore fatal with -u. Those
# hooks run whenever an env is activated *on top of itself*, so launching from an
# already-activated shell used to abort here with no message at all. Disable -u
# for all conda work, and start from base so an inherited env cannot leak its
# LD_LIBRARY_PATH into every child terminal we spawn.
set +u
# shellcheck disable=SC1090
source "$CONDA_SH"

guard=0
while [ "${CONDA_SHLVL:-0}" -gt 0 ] && [ "$guard" -lt 10 ]; do
	conda deactivate >/dev/null 2>&1 || break
	guard=$((guard + 1))
done
[ "$guard" -gt 0 ] && deactivated=1 || deactivated=0

conda activate "$ENV_NAME" 2>/dev/null
activated=$?
set -u

[ "$activated" -eq 0 ] || die "conda env '$ENV_NAME' not found (create it, or set ENV_NAME=...)"
[ "$deactivated" -eq 1 ] && ok "left the pre-activated env first (starting from base)"
ok "env: $ENV_NAME ($(python -V 2>&1))"

[ -f "$CONFIG" ] || die "config not found: $CONFIG"
ok "config: $CONFIG"

TERM_EMU=""
for t in gnome-terminal konsole xfce4-terminal terminator xterm; do
	command -v "$t" >/dev/null 2>&1 && {
		TERM_EMU="$t"
		break
	}
done
if command -v tmux >/dev/null 2>&1; then
	ok "tmux available (used if no graphical terminal)"
fi
[ -n "$TERM_EMU" ] || command -v tmux >/dev/null 2>&1 ||
	die "no terminal emulator or tmux found; install tmux, or run the three commands manually (see HILSERL.md)"
[ -n "$TERM_EMU" ] && ok "terminal: $TERM_EMU"

# --- CAN --------------------------------------------------------------------
if [ "$DO_CAN" -eq 1 ]; then
	if ip link show "$CAN_NAME" >/dev/null 2>&1 &&
		ip -br link show "$CAN_NAME" 2>/dev/null | grep -q 'UP'; then
		ok "CAN: $CAN_NAME already up"
	else
		say "Bringing up CAN ($CAN_NAME) -- sudo password may be required"
		bash "$REPO_ROOT/scripts/restart_can.sh" || die "CAN bring-up failed (see scripts/restart_can.sh)"
		ip -br link show "$CAN_NAME" 2>/dev/null | grep -q 'UP' || die "$CAN_NAME did not come up"
		ok "CAN: $CAN_NAME up"
	fi
else
	warn "CAN bring-up skipped (--no-can)"
fi

# --- cameras: every configured serial must actually be attached -------------
# A configured-but-missing RealSense takes down the whole camera streamer, so
# catching it here is far cheaper than debugging it mid-run.
CAM_MSG="$(python - "$CONFIG" <<'PY' 2>/dev/null
import json, sys
try:
	cfg = json.load(open(sys.argv[1]))
	cams = cfg.get("env", {}).get("robot", {}).get("cameras", {}) or {}
	want = {n: str(c.get("serial_number_or_name", "")) for n, c in cams.items()}
	import pyrealsense2 as rs
	have = {d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()}
	missing = {n: s for n, s in want.items() if s and s not in have}
	if missing:
		print("MISSING|" + ", ".join(f"{n}={s}" for n, s in missing.items()) +
			  "|attached: " + (", ".join(sorted(have)) or "none"))
	else:
		print("OK|" + ", ".join(f"{n}={s}" for n, s in want.items()))
except Exception as e:
	print(f"SKIP|{type(e).__name__}: {e}")
PY
)"
case "${CAM_MSG%%|*}" in
OK) ok "cameras: ${CAM_MSG#OK|}" ;;
MISSING)
	body="${CAM_MSG#MISSING|}"
	die "camera(s) not attached: ${body%%|*} (${body#*|})"
	;;
*) warn "camera check skipped (${CAM_MSG#SKIP|})" ;;
esac

# --- foxglove ---------------------------------------------------------------
FOXGLOVE_BIN=""
if [ "$DO_FG" -eq 1 ]; then
	for f in foxglove-studio foxglove /opt/Foxglove/foxglove-studio; do
		command -v "$f" >/dev/null 2>&1 && {
			FOXGLOVE_BIN="$f"
			break
		}
		[ -x "$f" ] && {
			FOXGLOVE_BIN="$f"
			break
		}
	done
	if [ -n "$FOXGLOVE_BIN" ]; then
		ok "foxglove: $FOXGLOVE_BIN (live on ws://localhost:$FG_PORT)"
	else
		warn "Foxglove not installed -- live view skipped; recordings still written"
	fi
else
	warn "Foxglove skipped (--no-foxglove)"
fi

# --- gamepad ----------------------------------------------------------------
if ls /dev/input/js* >/dev/null 2>&1; then
	ok "gamepad: $(ls /dev/input/js* | tr '\n' ' ')"
else
	warn "no /dev/input/js* -- gamepad not detected; you cannot intervene without it"
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
	say "Preflight only (--check); nothing launched."
	exit 0
fi

# --- launch -----------------------------------------------------------------
mkdir -p "$TB_LOGDIR"
FG_DIR="outputs/foxglove/$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$FG_DIR"

# Each pane: enter the repo, activate the env, run, then stay open so a crash
# is readable instead of vanishing with the window.
wrap() { printf 'cd %q && source %q && conda activate %q && %s; echo; echo "[%s exited -- press enter to close]"; read -r _' \
	"$REPO_ROOT" "$CONDA_SH" "$ENV_NAME" "$1" "$2"; }

LEARNER_CMD="$(wrap "PIPER_WARM_START=$WARM_START PIPER_WARM_START_POLICY=$WARM_POLICY python scripts/run_sac_learner.py --config_path $CONFIG" learner)"
# The actor must not start before the learner is serving, or it fails to connect.
# PIPER_FG_DIR is fixed here so the episode recordings land somewhere predictable.
ACTOR_CMD="$(wrap "echo 'waiting for learner on port $LEARNER_PORT ...'; \
	until (exec 3<>/dev/tcp/127.0.0.1/$LEARNER_PORT) 2>/dev/null; do sleep 1; done; \
	exec 3>&-; echo 'learner up, starting actor'; \
	PIPER_FG_DIR=$(printf '%q' "$FG_DIR") PIPER_FG_PORT=$FG_PORT PIPER_WARM_START_POLICY=$WARM_POLICY \
	PIPER_SMOOTH_WEIGHT=${PIPER_SMOOTH_WEIGHT:-0.005} PIPER_SMOOTH_SCALE=${PIPER_SMOOTH_SCALE:-0.5} \
	python scripts/run_sac_actor.py --config_path $CONFIG" actor)"
TB_CMD="$(wrap "tensorboard --logdir $TB_LOGDIR --port $TB_PORT" tensorboard)"

# Install a layout whose panels match this config's cameras, so the live stream
# comes up already wired instead of showing whatever layout was last selected.
if [ "$DO_FG" -eq 1 ]; then
	python -m piper_teleop.lerobot_plugin.foxglove_layout --config "$CONFIG" >/dev/null 2>&1 &&
		ok "Foxglove layout installed (\"HIL-SERL Transitions\")" ||
		warn "could not install the Foxglove layout"
fi

launch() { # launch <title> <command>
	case "$TERM_EMU" in
	gnome-terminal) gnome-terminal --title="$1" -- bash -c "$2" ;;
	konsole) konsole -p tabtitle="$1" -e bash -c "$2" ;;
	xfce4-terminal) xfce4-terminal --title="$1" -x bash -c "$2" ;;
	terminator) terminator --title="$1" -x bash -c "$2" ;;
	xterm) xterm -T "$1" -e bash -c "$2" ;;
	"") tmux new-window -n "$1" "bash -c $(printf '%q' "$2")" ;;
	esac
}

if [ -z "$TERM_EMU" ]; then
	tmux has-session -t hilserl 2>/dev/null && die "tmux session 'hilserl' already exists (tmux kill-session -t hilserl)"
	tmux new-session -d -s hilserl -n learner "bash -c $(printf '%q' "$LEARNER_CMD")"
fi

say "Starting learner"
[ -n "$TERM_EMU" ] && launch "hilserl-learner" "$LEARNER_CMD"

say "Starting actor (waits for learner on port $LEARNER_PORT)"
launch "hilserl-actor" "$ACTOR_CMD"

if [ "$DO_TB" -eq 1 ]; then
	say "Starting tensorboard on http://localhost:$TB_PORT"
	launch "hilserl-tensorboard" "$TB_CMD"
fi

# Foxglove connects to the actor's live server, so give the actor a moment to
# bind the port -- it starts only after the learner is up.
if [ "$DO_FG" -eq 1 ] && [ -n "$FOXGLOVE_BIN" ]; then
	say "Opening Foxglove on ws://localhost:$FG_PORT (live)"
	(
		for _ in $(seq 1 60); do
			(exec 3<>/dev/tcp/127.0.0.1/"$FG_PORT") 2>/dev/null && {
				exec 3>&-
				break
			}
			sleep 1
		done
		"$FOXGLOVE_BIN" "foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2Flocalhost%3A$FG_PORT" \
			>/dev/null 2>&1 &
	) &
fi

[ -z "$TERM_EMU" ] && say "tmux session 'hilserl' started -- attach with: tmux attach -t hilserl"

cat <<EOF

$(printf '\033[1;36m==>\033[0m') Session started.

  learner      training + logs        (step / losses / buffer+memory every 10s)
  actor        drives the robot       (episode reward + intervention rate)
  tensorboard  http://localhost:$TB_PORT   (watch episode/reward, episode/intervention_rate)

  foxglove     live view              (ws://localhost:$FG_PORT, opens once the actor binds)

  Live shows the present; recordings let you scrub the past. Both run together.
  Episode recordings (one .mcap per episode, written as each episode ends):
    $FG_DIR/
  Replay one afterwards:
    python scripts/view_foxglove.py            # newest episode (local Foxglove app)
    python scripts/view_foxglove.py --list     # everything recorded

  Pick the "HIL-SERL Transitions" layout once from Foxglove's layout menu.

  Gamepad:  Share = take over / release      Options = pause / resume
            Square = success                 Triangle = fail

  Collision checking is OFF -- keep a hand on the gamepad.
  Stop everything with Ctrl+C in the learner and actor windows.
EOF
