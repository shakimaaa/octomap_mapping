# Changelog

## 2026-05-19

- Added optional OctoMap visibility cleanup for dynamic obstacle handling.
- Added per-occupied-voxel last-seen tracking so visible stale occupied cells are only cleared after `octomap.dynamic_clear_duration`.
- Added `octomap.visibility_cleanup_rate` to limit visibility cleanup frequency; current configs set it to `1.0` Hz.
- Enabled visibility cleanup in `fast_lio.yaml` and `super_lio.yaml`.
- Split point-cloud filtering bounds (`octomap.pointcloudMin/Max*`) from local-map retention bounds (`octomap.localmapMin/Max*`).
- Updated `fast_lio.yaml` and `super_lio.yaml` with inline comments describing OctoMap probability, filtering, local-map cleanup, inflation, dynamic cleanup, topic, and TF parameters.
