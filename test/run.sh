#!/usr/bin/env bash
# Compile and run all native unit tests, then the WASM smoke test.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EIGEN="${EIGEN_INCLUDE_DIR:-/opt/homebrew/include/eigen3}"
CXX="clang++ -std=c++17 -O2 -I $ROOT/src -I $EIGEN"
SRC="$ROOT/src"
# All engine sources (tests link the subset they need; linking all is simplest).
ALL="$SRC/slam_engine.cpp $SRC/orb.cpp $SRC/two_view.cpp $SRC/pnp.cpp $SRC/ba.cpp $SRC/map.cpp $SRC/scale.cpp $SRC/imu_preint.cpp $SRC/vi_init.cpp $SRC/vi_optimizer.cpp $SRC/vi_window.cpp"
fail=0

run() { echo; echo "== $1 =="; shift; "$@" || fail=1; }

run "test_native (M1: FAST)"            sh -c "$CXX '$ROOT/test/test_native.cpp' $ALL -o /tmp/t_native && /tmp/t_native"
run "test_two_view (M2: geometry)"      sh -c "$CXX '$ROOT/test/test_two_view.cpp' $ALL -o /tmp/t_tv && /tmp/t_tv"
run "test_orb (M2: descriptors+match)"  sh -c "$CXX '$ROOT/test/test_orb.cpp' $ALL -o /tmp/t_orb && /tmp/t_orb"
run "test_pnp (M3: PnP)"                sh -c "$CXX '$ROOT/test/test_pnp.cpp' $ALL -o /tmp/t_pnp && /tmp/t_pnp"
run "test_ba (M4: bundle adjustment)"   sh -c "$CXX '$ROOT/test/test_ba.cpp' $ALL -o /tmp/t_ba && /tmp/t_ba"
run "test_map (M3+M4: map + BA)"        sh -c "$CXX '$ROOT/test/test_map.cpp' $ALL -o /tmp/t_map && /tmp/t_map"
run "test_reloc (M6: relocalization)"   sh -c "$CXX '$ROOT/test/test_reloc.cpp' $ALL -o /tmp/t_reloc && /tmp/t_reloc"
run "test_explore (M7: sliding map)"    sh -c "$CXX '$ROOT/test/test_explore.cpp' $ALL -o /tmp/t_expl && /tmp/t_expl"
run "test_scale (M5: metric scale)"     sh -c "$CXX '$ROOT/test/test_scale.cpp' $ALL -o /tmp/t_scale && /tmp/t_scale"
run "test_imu_preint (M12.1)"           sh -c "$CXX '$ROOT/test/test_imu_preint.cpp' $ALL -o /tmp/t_imu && /tmp/t_imu"
run "test_vi_init (M12.2)"              sh -c "$CXX '$ROOT/test/test_vi_init.cpp' $ALL -o /tmp/t_vi && /tmp/t_vi"
run "test_vi_opt (M12.3)"               sh -c "$CXX '$ROOT/test/test_vi_opt.cpp' $ALL -o /tmp/t_viopt && /tmp/t_viopt"
run "test_vi_window (M12.4 live init)"  sh -c "$CXX '$ROOT/test/test_vi_window.cpp' $ALL -o /tmp/t_viwin && /tmp/t_viwin"
run "test_track_gyro (M13 gyro prior)"  sh -c "$CXX '$ROOT/test/test_track_gyro.cpp' $ALL -o /tmp/t_trkgyro && /tmp/t_trkgyro"
run "test_wasm (compiled artifact)"     node "$ROOT/test/test_wasm.mjs"

echo
[ "$fail" -eq 0 ] && echo "ALL TESTS PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
