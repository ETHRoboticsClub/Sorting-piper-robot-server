```bash
lerobot-train \
  --dataset.repo_id=/home/arc_user/Sorting-piper-robot-server/data/2026-04-11_10-47-28_video \
  --policy.type=act \
  --output_dir=outputs/train/test_single_arm \
  --job_name=act_single_arm \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/act_policy
```

```bash
lerobot-edit-dataset \
  --new_repo_id /home/arc_user/Sorting-piper-robot-server/data/merged/20260408_0 \
  --repo_id /home/arc_user/Sorting-piper-robot-server/data/merged/20260408_0 \
  --operation.type merge \
  --operation.repo_ids "['/home/arc_user/Sorting-piper-robot-server/data/processed/20260408_1', '/home/arc_user/Sorting-piper-robot-server/data/processed/20260408_0', '/home/arc_user/Sorting-piper-robot-server/data/processed/20260407_0_sub']"
```

```bash
robotserver --policy --show-cameras
```

```bash
lerobot-edit-dataset \
  --repo_id /home/arc_user/Sorting-piper-robot-server/data/2026-04-08_21-33-57 \
  --operation.type delete_episodes \
  --operation.episode_indices "[4]"
```

Temporary fix to train model -> Some issues, also isssue wiht shwoing vide -> --show-camera
conda activate piper_new_backup
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

lerobot-train \
  --dataset.repo_id=/home/arc_user/Sorting-piper-robot-server/data/2026-04-11_10-47-28_video \
  --dataset.video_backend=pyav \
  --policy.type=act \
  --output_dir=outputs/train/test_single_arm111 \
  --job_name=test11 \
  --policy.device=cuda \
  --policy.repo_id=${HF_USER}/act_20260409_final_mix_smoke_pyav \
  --steps=500


  