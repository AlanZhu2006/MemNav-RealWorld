# Local runtime storage

Updated 2026-09-17 for the unified `Nav-graph-blind/projects/realworld` checkout.

`runtime/` remains outside Git. Preserve the original Survey datasets, run
configs, RGB recordings, paired outcomes and manifests. Before collecting,
labeling or cleaning experiments, read the active machine's protected
`runtime/go2/experiment_pairs/index.json`.

The local cleanup removed 13 rebuildable Survey state caches across the
workspace. Original inputs and metadata remain; the next preparation replays
RGB if no complete compatible checkpoint exists. The cache is a saved episode
state, separate from model weights and the optional compact historical archive.

For `runtime/experiment_archives/pair018_pair019_20260909T120057Z`, the expanded
`jetson/` and `gpu_snapshot/` directories were byte-compared with their retained
compressed archives and removed. `LOCAL_STORAGE_STATUS.md` in that directory
contains restore commands. The original archive hashes and transfer records
retain their historical meaning.

The workspace audit is `.workspace-maintenance/20260917-storage-prune/` in the
parent navigation repository. A complete audit copy and experimental arrays
are archived at `/data/workspace-archives/gem-storage-prune-20260917/`.
See the navigation repository's
[storage guide](https://github.com/AlanZhu2006/Nav/blob/main/docs/LOCAL_STORAGE.md)
for recovery. These local archives are not downloaded by cloning this repository.
