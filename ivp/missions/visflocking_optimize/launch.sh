#!/bin/bash

# --- Read Command Line Arguments ---
A0_VAL=${1:-1.0} # Default wert 1.0, falls nichts übergeben wird
A1_VAL=${2:-0.09}
B0_VAL=${3:-1.0}
B1_VAL=${4:-0.09}
GAM_VAL=${5:-0.1}
# Degrees. 360=100% FOV, 270=75%, 180=50%, 90=25% (Supp. Fig. 3/4 columns).
# Must stay in sync between meta_vehicle.bhv (BHV_VisFlocking) and
# meta_shoreside.moos (pSimVisionServer) -- both get this same value below.
FOV_VAL=${6:-360.0}
# Tags every row uFlockEvaluator writes this run (see meta_shoreside.moos).
# Defaults to the parameter combo itself so a run is identifiable even if
# the caller doesn't pass one explicitly.
RUN_ID_VAL=${7:-fov_${FOV_VAL}_a0_${A0_VAL}_a1_${A1_VAL}_b0_${B0_VAL}_b1_${B1_VAL}_gam_${GAM_VAL}}
# Optional: ";"-separated list of "x,y,heading" triples, one per vehicle in
# alpha_001..alpha_0<VEHICLE_COUNT> order (set by optimize.py's --random).
# Empty (the default) keeps the original fixed staggered-line start below.
START_POSES_VAL=${8:-}
# Which concurrent sweep worker this run belongs to (optimize.py's --jobs N
# assigns each worker a fixed slot 0..N-1 for its whole lifetime). Slot 0
# (the default, for a lone ./launch.sh or --jobs 1) reproduces the historical
# port numbers/paths exactly. Every generated/aliased/ported thing below is
# derived from this one number so N slots never collide with each other.
SLOT_VAL=${9:-0}

# Retina resolution, derived from the FOV so the angular resolution is always
# 1.125 deg/bin (Supp. Table 1's Nret = 320 over a full 360 deg circle). The
# vision range follows from the minimum-blob-width rule at that bin size, so a
# fixed resolution across FOV columns would silently change how far the agents
# can see (see meta_shoreside.moos's pSimVisionServer block).
RES_VAL=$(python3 -c "print(int(round(320 * $FOV_VAL / 360.0)))")

# Verification logging: LOGGING=yes ./launch.sh ... launches pLogger on the
# shoreside (whole-fleet NODE_REPORTs, for plot_trajectories.py) and on each
# vehicle (DEBUG_*/DESIRED_*/NAV_*). Off by default -- sweeps don't need it.
LOGGING_VAL=${LOGGING:-no}

# Hull dynamics: "baseline" (default) makes the boats near-holonomic so the
# vision model itself is what the run measures; "boat" restores the rudder-
# steered hull (turn-rate clip, speed/thrust-coupled turning, deceleration
# limit, stiffer heading PID) to measure what the vehicle costs the collective
# behaviour. DYNAMICS=boat ./launch.sh ...
DYNAMICS_VAL=${DYNAMICS:-baseline}

# --- Per-slot port block ---
# Shoreside MOOSDB/pShare ports used to be hardcoded literals (9000/9200) in
# meta_shoreside.moos, which is exactly why two overlapping runs couldn't
# share a machine. Every slot now gets its own 100-wide block: slot 0 is
# 9000/9200 (unchanged from before), slot 1 is 9100/9300, etc. -- comfortably
# wide for VEHICLE_COUNT=10 vehicles offset within a slot.
SHORE_PORT=$((9000 + 100 * SLOT_VAL))
SHORE_PSHARE_PORT=$((9200 + 100 * SLOT_VAL))

# Number of vehicles (override for small diagnostic runs, e.g.
# VEHICLE_COUNT=2 ./launch.sh ... with a two-element START_POSES list)
VEHICLE_COUNT=${VEHICLE_COUNT:-10}

# --- Per-slot working directory ---
# targ_shoreside.moos/targ_<VNAME>.moos/.bhv/plug_pshare_outputs.moos used to
# be generated in the mission directory itself with fixed names, so two
# concurrent runs would stomp each other's files; flock_evaluation.csv
# (opened relative + append-mode by uFlockEvaluator) would interleave rows
# from both runs. Isolating each slot in its own directory and cd-ing into it
# before generating/launching anything fixes both for free: every subsequent
# relative path (targ_*, plug_pshare_outputs.moos, flock_evaluation.csv,
# MOOSLog_*) resolves inside this slot's own directory, and pAntler's
# children inherit that cwd.
MISSION_DIR="$(pwd)"
SLOT_DIR="$MISSION_DIR/.slots/slot${SLOT_VAL}"

# --- Kill only this slot's previous processes before starting ---
# Every process below is launched with $SLOT_DIR's own *absolute* targ_*.moos
# path as its mission-file argument (pAntler forwards that argv straight
# through to each child it spawns, unchanged -- see Antler.cpp's
# DoNixOSLaunch), so every process in this slot has $SLOT_DIR somewhere in
# its command line, and no *other* slot's does. `pkill -f "$SLOT_DIR/"`
# matches exactly this slot's MOOS apps by their real command line, killed
# directly via signal -- no network/multicast involved, and no reliance on
# pAntler's own "~alias" mechanism.
#
# (An earlier version of this instead gave every process a
# "~<App>..._s<slot>" pAntler alias and killed by matching that. Dropped
# because most MOOS-IvP apps' main.cpp (pHelmIvP, uSimMarineV23, pShare,
# uTimerScript, etc. -- everything except uFlockEvaluator) treats that same
# alias as argv[2] and feeds it into CMOOSApp::Run() as the app's own
# GetAppName(), which OnStartUp() then uses to find its *own*
# "ProcessConfig = <AppName>" block -- so aliasing silently broke every
# aliased app's ability to find its own config block (confirmed directly:
# uTimerScript's DEPLOY_ALL event never fired while aliased, because it
# could no longer find its "ProcessConfig = uTimerScript" block under the
# renamed identity; pHelmIvP/uSimMarineV23 would have silently lost their
# behavior file / physics config the same way). The slot directory in the
# mission-file path achieves the same per-slot kill scoping without
# touching any app's identity or config lookup at all.)
#
# pAntler itself doesn't need to be killed directly: it blocks in a wait
# loop until every process *it* spawned has exited (Antler.cpp's Spawn()),
# so once the pkill above kills its children, pAntler notices and exits on
# its own within its ~100ms poll interval.
echo "Cleaning up slot ${SLOT_VAL}'s previous processes (if any)..."
pkill -9 -f "$SLOT_DIR/" 2>/dev/null
sleep 1
# ----------------------------------------------------

mkdir -p "$SLOT_DIR"
cd "$SLOT_DIR"

echo "Generating dynamic pShare routes for shoreside..."
> plug_pshare_outputs.moos # Clear or create the file

# Loop to generate the pShare outputs for N vehicles
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  PORT=$((SHORE_PSHARE_PORT + $i))
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  VNAME_UPPER=$(echo $VNAME | tr '[:lower:]' '[:upper:]')
  echo "  // Routes for Vehicle $i (Port $PORT)" >> plug_pshare_outputs.moos
  echo "  Output = src_name=DEPLOY_ALL, dest_name=DEPLOY, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "  Output = src_name=RETURN_ALL, dest_name=RETURN, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "  Output = src_name=MOOS_MANUAL_OVERRIDE_ALL, dest_name=MOOS_MANUAL_OVERRIDE, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "  Output = src_name=APPCAST_REQ, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "" >> plug_pshare_outputs.moos

  echo "  Output = src_name=NODE_REPORT_ALL, dest_name=NODE_REPORT, route=localhost:$PORT" >> plug_pshare_outputs.moos

  echo "  Output = src_name=VPF_$VNAME_UPPER, dest_name=VPF, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "" >> plug_pshare_outputs.moos
done

# Generate the final shoreside file (nsplug will automatically absorb the #include file)
nsplug "$MISSION_DIR/meta_shoreside.moos" targ_shoreside.moos -f RUN_ID="$RUN_ID_VAL" FOV_VAL="$FOV_VAL" RES_VAL="$RES_VAL" LOGGING="$LOGGING_VAL" SHORE_PORT="$SHORE_PORT" SHORE_PSHARE_PORT="$SHORE_PSHARE_PORT"

echo "Assembling MOOS and BHV files for vehicles..."

# Loop to generate .moos and .bhv files for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  # Pad the vehicle number with leading zeros (e.g., 001, 002, ...)
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  MOOS_PORT=$((SHORE_PORT + $i))
  PSHARE_PORT=$((SHORE_PSHARE_PORT + $i))
  POLAR_PLOT_STR="0,100: 90,100: 180,100"

  # Fixed staggered-line start (default), or this vehicle's triple pulled
  # out of START_POSES_VAL (optimize.py's --random) if one was supplied.
  if [ -n "$START_POSES_VAL" ]; then
    IFS=';' read -ra START_POSES_ARR <<< "$START_POSES_VAL"
    IFS=',' read -r POSE_X POSE_Y POSE_H <<< "${START_POSES_ARR[$((i - 1))]}"
    START_POS_STR="x=$POSE_X,y=$POSE_Y,heading=$POSE_H"
  else
    START_POS_STR="x=$((($i*4) - 10)),y=$((6 +($i%3))),heading=-$(($i*5))"
  fi

  # Generate .moos and .bhv files
  nsplug "$MISSION_DIR/meta_vehicle.moos" "targ_${VNAME}.moos" -f VNAME="$VNAME" MOOS_PORT="$MOOS_PORT" PSHARE_PORT="$PSHARE_PORT" SHORE_PSHARE_PORT="$SHORE_PSHARE_PORT" POLAR_PLOT="$POLAR_PLOT_STR" START_POS="$START_POS_STR" LOGGING="$LOGGING_VAL" DYNAMICS="$DYNAMICS_VAL"

  # Inject the A0, A1, B0, B1, GAM and FOV variables into the behavior file generation
  nsplug "$MISSION_DIR/meta_vehicle.bhv" "targ_${VNAME}.bhv" -f VNAME="$VNAME" POLAR_PLOT="$POLAR_PLOT_STR" RETURN_POS="0,-20" A0_VAL="$A0_VAL" A1_VAL="$A1_VAL" B0_VAL="$B0_VAL" B1_VAL="$B1_VAL" GAM_VAL="$GAM_VAL" FOV_VAL="$FOV_VAL"
done

echo "Launching Simulation (slot ${SLOT_VAL}, dir $SLOT_DIR)..."
# Make sure to launch the newly generated targ_shoreside.moos, not the template!
# The *absolute* path (rather than the bare relative filename) is what makes
# this slot's processes identifiable for the pkill-based cleanup above --
# see the comment there.
pAntler "$SLOT_DIR/targ_shoreside.moos" >& /dev/null &
sleep 0.4

# Loop to launch pAntler for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  pAntler "$SLOT_DIR/targ_${VNAME}.moos" >& /dev/null &
  sleep 0.4
done

echo "Simulation running (no GUI -- deploy is automatic via uTimerScript's DEPLOY_ALL event)."
