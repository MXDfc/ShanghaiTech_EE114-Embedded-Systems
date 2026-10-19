# Course archive — 2026-10-19

The course has ended. This archive preserves the local lab/source history and
the remote LD14/LD14P project, including the lidar mapping navigation update.

- Seventeen local commits were reconstructed without course video files.
  Each reconstructed commit records its original commit ID in `Original-Commit`.
- Existing remote history was retained; this update does not require a force push.
- Remaining local source files and course documents were added. Existing local
  deletions were retained; earlier non-video versions remain in Git history.
- `Lab2and3/Archived_Lab2and3_MXGen` preserves the standalone STM32 project
  whose nested MDK-ARM Git repository had no commits and no configured remote.
  Core, Drivers, Task, the CubeMX project and the Keil project are included;
  generated build files and machine-specific IDE state are omitted.
- Videos are excluded by `.gitignore`. Unpushed videos are not migrated to LFS.
  Video objects already present in previously published history are not rewritten.

This change archives existing code; it does not claim a new hardware test.
