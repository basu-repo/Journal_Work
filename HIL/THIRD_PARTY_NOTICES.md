# Source provenance

FAST-LIO upstream: https://github.com/hku-mars/FAST_LIO

The local experimental checkout reports commit
`a4743b095409588842a5b30ddfa27e29d2f99164`. The bundled files include local
ROS 2 Jazzy/C++17 adaptations and the explicit type-5 snapshot PointCloud2
preprocessor. This commit identifies the upstream base, not an unmodified tree.
No synthetic rotating point times are assigned by the snapshot handler.

The complete upstream LICENSE is preserved in vendor/FAST_LIO/LICENSE
(GNU GPL version 2). Upstream package.xml declares BSD; this metadata discrepancy
is retained here rather than treated as permission to replace the supplied license.
IKFoM and ikd-Tree source/header notices are retained in their bundled files.
The delivery adds missing build dependency declarations and installs only the
configuration directory because upstream RViz/launch examples are not needed
by the portable launcher.
The packaged C++ source also guards optional debug-file handles during shutdown,
so a source checkout without a Log directory can stop cleanly.

Navigation source is the current Work DijkstraLattice implementation, flattened
with its required grid, graph, state-lattice, safety and tracking modules into
src/dijkstra_lattice. Imports/class names are localized for portability. The
executable always selects forward Dijkstra search; shared graph infrastructure
does not expose another planner selection in run_hil.py.

The delivery includes the original execution report as historical simulation
evidence. Receiving-platform configuration and physical validation are separate.
