# FAST-LIO2 Navigation Execution Diagnosis and Validation

Date: 2026-10-24  
Cross-environment update: 2026-10-25

## Experimental boundary

The validated FAST-LIO2 and SLAM Toolbox localization configuration remained
unchanged. GNSS was disabled. Dijkstra, lattice motion primitives, Baylands
waypoints, arrival tolerances, sensor rates, obstacle thresholds, and Gazebo
ground-truth isolation remained unchanged except for the two execution changes
described below. Gazebo truth was used only for offline evaluation.

## Baseline diagnosis

The 1,800-second baseline run reached waypoint 1 and waypoint 2 but timed out
17.5 m from the final goal. Localization remained accurate: FAST-LIO2 position
RMSE was 0.028 m, SLAM-global position RMSE was 0.028 m, and heading RMSE was
0.265 degrees. Navigation was inefficient: 652.31 simulated seconds (45.11%)
had zero command, 89 primitives started, no primitive completed normally, and
all 85 primitive interruptions were `tracking_timeout`.

Reason-coded diagnostics proved that the newest 10 Hz LiDAR scan frequently
arrived before the corresponding FAST-LIO2 TF was available. Exact-time TF
lookup failed, navigation was disabled, the stored velocity reset to zero, and
the controller repeatedly accelerated from its minimum speed.

## Rejected intermediate

Reusing the last transformed obstacle cloud for as long as the existing
one-second sensor timeout eliminated the timeout cycle and completed three
missions. It was rejected because two runs contacted `House 2_2`. Cached-cloud
age averaged about 0.27 to 0.29 s and reached 0.85 s. Mission completion did
not override this failed safety gate.

## Final isolated changes

1. Exact-time LiDAR TF remains preferred. When it is unavailable, the newest
   LiDAR scan may use the latest available TF only when TF lag is no more than
   0.15 s. Older data causes a fail-closed stop. No old obstacle cloud is
   replayed.
2. If continuous-pose anchoring rejects a direct-progress primitive, the next
   planning attempt bypasses direct progress and invokes full Dijkstra search.
   A successfully activated primitive resets this rejection state.

The final chain is:

`FAST-LIO2 -> bounded scan/TF alignment -> SLAM global pose -> Dijkstra ->`
`continuous-pose primitive validation -> trajectory tracking`

## Final three-run results

| Metric | Run 1 | Run 2 | Run 3 | Mean +/- SD |
|---|---:|---:|---:|---:|
| Verified mission completion | Yes | Yes | Yes | 3/3 |
| Non-ground contacts | 0 | 0 | 0 | 0 |
| Wall time (s) | 391.00 | 331.99 | 324.88 | 349.29 +/- 36.30 |
| Simulated mission time (s) | 312.22 | 262.99 | 260.71 | 278.64 +/- 29.10 |
| Distance travelled (m) | 107.10 | 109.26 | 106.87 | 107.74 +/- 1.32 |
| WP1 physical error (m) | 0.211 | 0.244 | 0.238 | 0.231 +/- 0.018 |
| WP2 physical error (m) | 0.210 | 0.224 | 0.253 | 0.229 +/- 0.022 |
| Goal physical error (m) | 0.208 | 0.283 | 0.348 | 0.280 +/- 0.070 |
| FAST-LIO2 position RMSE (m) | 0.070 | 0.089 | 0.081 | 0.080 +/- 0.009 |
| SLAM-global position RMSE (m) | 0.070 | 0.089 | 0.081 | 0.080 +/- 0.009 |
| Heading RMSE (degrees) | 1.113 | 1.269 | 1.026 | 1.136 +/- 0.123 |
| Zero-command time (%) | 30.39 | 14.60 | 20.09 | 21.69 +/- 8.01 |
| Average moving speed (m/s) | 0.493 | 0.487 | 0.513 | 0.498 +/- 0.014 |
| Primitive completions | 48 | 44 | 47 | 46.33 +/- 2.08 |
| Primitive interruptions | 0 | 0 | 0 | 0 |
| Anchored primitive rejections | 4 | 0 | 1 | 1.67 +/- 2.08 |
| Recovery rounds | 0 | 0 | 0 | 0 |

## TF fallback and latency

The bounded fallback was exercised 260, 230, and 222 times. Across runs its
mean lag was 0.0217 +/- 0.0016 s, the 95th percentile was 0.1000 s, and the
maximum was 0.100000001 s in every run. It never approached or exceeded the
0.15 s acceptance bound. FAST-LIO2 message latency averaged 0.0589 +/- 0.0007
s; the per-run maximum averaged 0.1013 +/- 0.0134 s.

## Acceptance decision

The final configuration passes the defined gates:

- localization stayed accurate under full-mission dynamics;
- all three physical missions completed;
- every waypoint and goal arrival was independently verified;
- no non-ground collision was recorded;
- TF fallback remained bounded below 0.15 s;
- TF-induced tracking timeouts fell from 85 to zero;
- the rejected-direct-primitive loop did not recur;
- no recovery round was required;
- zero-command time fell from 45.11% in the long baseline to 21.69% on average.

These results validate the navigation integration under the tested Baylands
simulation conditions. They do not establish safety for untested speeds,
sensor delays, terrains, or real hardware.

## Cross-environment generalization test

The frozen Baylands configuration was subsequently tested without parameter
changes in Warehouse, Santorini, and sandHeightMap. Every run used the A300,
GNSS-denied localization, FAST-LIO2 local odometry, SLAM Toolbox global
correction, Dijkstra state-lattice navigation, headless Gazebo execution, and
the same 0.15 s bounded scan/TF fallback. Ground truth remained evaluation-only.

These are single exploratory runs per environment. They test whether the
validated configuration transfers at all; they are not yet a repeated-run
generalization benchmark.

| Metric | Warehouse | Santorini | sandHeightMap |
|---|---:|---:|---:|
| Independently verified mission | Failed | Complete | Complete |
| Verified targets | 0/3 | 3/3 | 3/3 |
| Wall time (s) | 49.89 | 683.94 | 445.01 |
| Simulated end time (s) | 45.48 | 586.51 | 378.83 |
| Actual planar travel (m) | 6.99 | 105.37 | 118.26 |
| WP1 physical error (m) | Not reached | 0.318 | 0.157 |
| WP2 physical error (m) | Not reached | 0.270 | 0.296 |
| Goal physical error (m) | Not reached | 0.216 | 0.112 |
| Primitive completions | 3 | 45 | 75 |
| Primitive interruptions | 0 | 8 | 8 |
| Primitive rejections | 0 | 2 | 4 |
| Lattice search budget exceeded | 0 | 59 | 6 |
| Search round-robin resets | 0 | 18 | 0 |
| Recovery rounds | 0 | 0 | 0 |
| Bounded TF fallbacks | 41 | 426 | 320 |
| Mean fallback lag (s) | 0.0171 | 0.0195 | 0.0250 |
| Maximum fallback lag (s) | 0.1000 | 0.1000 | 0.1000 |

### Warehouse failure

Warehouse failed during the first leg with approximately 7.9 m remaining to
waypoint 1. The robot had travelled 6.99 m and completed three primitives.
Planning then reported `start_occupied`; the immediate command check classified
both the grid and live LiDAR corridor as blocked, with the nearest LiDAR return
at approximately 0.746 m. Backup and rotation recovery commands were also
safety-blocked, so the supervisor terminated with `recovery_failed`.

This was not the original FAST-LIO2 synchronization failure: there were zero
primitive tracking interruptions, TF fallback remained within 0.1000 s, and
the localization pipeline continued operating. No obstacle or structure
contact was recorded. The result exposes a separate costmap/start-cell and
blocked-recovery limitation in Warehouse.

### Santorini result

Santorini physically reached all three targets. The run recorded 45 completed
primitives, eight interruptions, and no recovery round. Its difficult second
leg repeatedly exhausted the five-second lattice search budget: 59 budget
exhaustions and 18 round-robin resets produced a long stationary interval
before a valid route was eventually found. The mission therefore passed
completion, physical-arrival, collision, and bounded-TF gates, but it did not
match Baylands execution continuity.

All 4,157 reported contacts were wheel-to-terrain contacts with
`santorini_scaled::terrain_link::collision`; no structure or obstacle contact
was recorded.

### sandHeightMap result

sandHeightMap also physically reached all targets without a recovery round.
It recorded 75 completed primitives, eight interruptions, six search-budget
exhaustions, and no search round-robin reset. Detours temporarily increased
straight-line distance near waypoint 2, but the planner subsequently closed
the distance and completed the mission.

All 3,021 reported contacts were wheel-to-heightmap-ground contacts with
`sand_heightmap::link::ground`; no structure or obstacle contact was recorded.

### Generalization assessment

The frozen architecture transferred successfully to two of three additional
environments. Santorini and sandHeightMap support the claim that bounded TF
alignment and FAST-LIO2 localization are not specific to Baylands. Warehouse
prevents a broader claim of environment-independent navigation reliability.

The tests also separate two remaining navigation issues from localization:

- Warehouse can enter a fail-closed `start_occupied` state from which every
  recovery command is rejected.
- Santorini can spend substantial time in repeated time-limited lattice
  searches before finding a valid detour.

The validated Baylands configuration remains frozen. Any Warehouse costmap or
recovery change should be isolated as a new experiment and then revalidated
against Baylands, Santorini, and sandHeightMap rather than folded into this
baseline.

## Evidence

- Baseline partial run: `result/20261024_155733_520303`
- Rejected cached-cloud runs: `result/20261024_164655_281149`,
  `result/20261024_165620_077448`, and `result/20261024_170133_865371`
- Final run 1: `result/20261024_171945_313140`
- Final run 2: `result/20261024_172845_475586`
- Final run 3: `result/20261024_173444_474681`
- Warehouse generalization run: `result/20261024_193516_077449`
- Santorini generalization run: `result/20261024_193627_486774`
- sandHeightMap generalization run: `result/20261024_194808_055082`
