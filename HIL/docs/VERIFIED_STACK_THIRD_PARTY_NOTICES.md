# Source provenance and third-party notices

## FAST-LIO2

Upstream repository: https://github.com/hku-mars/FAST_LIO

Upstream base commit: `a4743b095409588842a5b30ddfa27e29d2f99164`.

This package includes ROS 2 Jazzy/C++17 adaptations and an explicit type-5
snapshot PointCloud2 preprocessor. The base commit identifies the upstream
revision; the included source also contains these modifications. The snapshot
handler assigns no synthetic rotating point timestamps.

Retain `vendor/FAST_LIO/LICENSE` and the source copyright notices when
redistributing the package. The supplied LICENSE contains GNU GPL version 2;
the upstream package.xml declares BSD. Preserve the supplied license text
and note this metadata discrepancy.

IKFoM and ikd-Tree source notices are retained in their respective files.

The packaged source adds build dependency declarations and guards optional
debug-file handles during shutdown. Sensor configurations are stored in the
top-level `config/` directory and loaded by the localization launcher.
Unused plotting/OpenCV wrapper headers and Python plotting linkage are omitted.

## Navigation

This package includes the DijkstraLattice implementation and its required grid,
graph, state lattice, safety and tracking modules under `src/dijkstra_lattice/`.
Imports and class names are adapted for standalone use. The navigation entry
point selects forward Dijkstra search; `run_hil.py` does not expose another
planner selection.
