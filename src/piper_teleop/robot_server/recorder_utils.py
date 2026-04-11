import logging
from io import BytesIO
from pathlib import Path

import PIL
import numpy as np
import pyarrow.dataset as ds
import pyarrow.parquet as pq
import tqdm
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.io_utils import load_info, load_tasks
from lerobot.datasets.utils import INFO_PATH

logger = logging.getLogger(__name__)

# Valid placeholder for LeRobotDataset.create (must not be empty; local-only, not uploaded).
_LOCAL_VIDEO_EXPORT_REPO_ID = "local/piper_image_to_video"


def _replace_dict_str(data, a, b):
    if isinstance(data, str):
        return data.replace(a, b)
    elif isinstance(data, dict):
        return {k: _replace_dict_str(v, a, b) for k, v in data.items()}
    elif isinstance(data, list):
        return [_replace_dict_str(v, a, b) for v in data]
    else:
        # nothing to do?
        return data
def convert_image_dataset_to_video(
    root: Path | str,
    batch_parquet=50,
    image_writer_processes=0,
    image_writer_threads=12,
    output_root: Path | str | None = None,
):
    path = Path(root)
    if not (path / INFO_PATH).is_file():
        raise FileNotFoundError(f"Not a LeRobot dataset root (missing {INFO_PATH}): {path}")
    # Do not use LeRobotDatasetMetadata here: it requires meta/episodes/*.parquet, which older/local
    # recordings from this repo may not have. info.json + tasks.parquet + data/ are enough to convert.
    info = load_info(path)
    tasks_df = load_tasks(path)
    features_new = _replace_dict_str(info["features"], "image", "video")
    out_root = Path(output_root) if output_root is not None else path.with_name(path.name + "_video")
    new_dataset = LeRobotDataset.create(
        repo_id=_LOCAL_VIDEO_EXPORT_REPO_ID,
        root=out_root,
        features=features_new,
        fps=info["fps"],
        robot_type=info["robot_type"],
        use_videos=True,
        image_writer_processes=image_writer_processes,
        image_writer_threads=image_writer_threads,
    )
    ds_dataset = ds.dataset(path / "data")
    parquet_files = sorted(ds_dataset.files)
    ep_index = 0
    glob_index = 0
    total_episodes = int(info["total_episodes"])
    with tqdm.tqdm(total=total_episodes, desc="converting episodes") as pbar:
        for parquet_file in parquet_files:
            try:
                parquet_file_reader = pq.ParquetFile(parquet_file)
                for batch in parquet_file_reader.iter_batches(batch_size=batch_parquet):
                    for i in range(len(batch)):
                        row = {col: batch[col][i].as_py() for col in batch.column_names}
                        if ep_index < row['episode_index']:
                            ep_index = row['episode_index']
                            new_dataset.save_episode()
                            pbar.update(1)
                        task = tasks_df.index[int(row["task_index"])]
                        obs_action = {'action': np.array(row['action'], dtype=np.float32),
                                    'observation.state': np.array(row['observation.state'], dtype=np.float32),
                                    'task': task}
                        cam = {k: np.array(PIL.Image.open(BytesIO(v['bytes']))) for k, v in row.items() if 'observation.images' in k}
                        frame = {**obs_action, **cam}
                        new_dataset.add_frame(frame)
                        assert glob_index == row['index'], 'indexing error, this should not happen we iterate sequentially'
                        glob_index += 1
            except Exception as e:
                logger.error(f"Error processing {parquet_file}: {e}")
    new_dataset.save_episode()
    new_dataset.finalize()

    logger.info(f"Converting image dataset to video format: {path}")
    logger.info("Video conversion completed successfully!")
