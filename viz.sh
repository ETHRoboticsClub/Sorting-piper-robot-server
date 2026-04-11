#!/usr/bin/env bash
REPO_ID="/home/arc_user/Sorting-piper-robot-server/data/raw/2026-04-08_21-36-01"

i=0
while true; do
  echo "Showing episode $i"
  lerobot-dataset-viz \
    --repo-id "$REPO_ID" \
    --mode local \
    --episode-index "$i"

  read -rp "[n]ext, [p]rev, [q]uit: " ans
  case "$ans" in
    n) ((i++)) ;;
    p) ((i>0)) && ((i--)) ;;
    q) break ;;
  esac
done
