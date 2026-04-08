```bash
lerobot-train \
  --dataset.repo_id=/home/arc_user/Sorting-piper-robot-server/data/merged_51_ep \
  --policy.type=act \
  --output_dir=outputs/train/act_merged \
  --job_name=act_merged \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/act_policy
```

```bash
lerobot-edit-dataset \
  --repo_id /home/arc_user/Sorting-piper-robot-server/data/merged_51_ep \
  --new_repo_id /home/arc_user/Sorting-piper-robot-server/data/merged_51_ep \
  --operation.type merge \
  --operation.repo_ids "['/home/arc_user/Sorting-piper-robot-server/data/xxx', '/home/arc_user/Sorting-piper-robot-server/data/yyy']"
```

```bash
robotserver --policy --show-cameras
```
