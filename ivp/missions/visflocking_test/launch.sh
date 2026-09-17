#!/bin/bash

# --- Parse --random / --seed flags out of the argument list (they can
# appear anywhere, e.g. trailing after run_id) before the positional
# params below are assigned. Mirrors visflocking_optimize/optimize.py's
# --random/--seed: paper-style random start poses (positions uniform in
# the central third of the arena, headings uniform in [-180,180)) instead
# of the fixed staggered-line start; --seed makes the draw reproducible.
RANDOM_START=0
SEED_VAL=""
POSITIONAL_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --random)
      RANDOM_START=1
      shift
      ;;
    --seed)
      SEED_VAL="$2"
      shift 2
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift
      ;;
  esac
done
set -- "${POSITIONAL_ARGS[@]}"

# --- Vision-flocking parameters: edit the defaults below directly, or
# override any of them from the command line (in this order):
#   ./launch.sh [a0] [a1] [b0] [b1] [gam] [fov_deg] [run_id] [--random] [--seed N]
A0_VAL=${1:-0.1}    # social acceleration/deceleration strength
A1_VAL=${2:-0.09}   # equilibrium front-back distance
B0_VAL=${3:-3.0}    # social turning strength
B1_VAL=${4:-0.2}   # equilibrium left-right distance
GAM_VAL=${5:-0.8}   # self-propulsion preference factor
# Degrees. 360=100% FOV, 270=75%, 180=50%, 90=25% (Supp. Fig. 3/4 columns).
# Must stay in sync between meta_vehicle.bhv (BHV_VisFlocking) and
# meta_shoreside.moos (pSimVisionServer) -- both get this same value below.
FOV_VAL=${6:-120.0}
# Tags every row uFlockEvaluator writes this run (see meta_shoreside.moos).
RUN_ID_VAL=${7:-manual_test}
# Must match VEHICLE_COUNT below and meta_vehicle.moos's
# wormhole_safety_period (the torus's full wrap period) -- see
# visflocking_optimize/optimize.py's ARENA_WIDTH.
ARENA_WIDTH=164.0

# --- Kill previous processes before starting ---
# NOTE: the MOOS app name registered by uFlockEvaluator's binary is
# "pFlockEvaluator" (see ProcessConfig block), but the actual OS process
# (and thus what killall must match) is "uFlockEvaluator" -- the exe name.
echo "Cleaning up old processes..."
killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uFlockEvaluator uTimerScript
sleep 1
# ----------------------------------------------------

# Number of vehicles
VEHICLE_COUNT=10

# ";"-separated list of "x,y,heading" triples, one per vehicle, used below
# instead of the fixed staggered-line start when --random was passed.
# Positions uniform in a square of side ARENA_WIDTH/3 centered on the
# arena origin (the paper's "central third of the arena"), headings
# uniform in [-180,180) -- same distribution as optimize.py's
# random_start_poses(). Seeded via awk's srand() when --seed is given, so
# a run is reproducible; otherwise awk seeds from the current time/pid.
START_POSES_VAL=""
if [ "$RANDOM_START" -eq 1 ]; then
  START_POSES_VAL=$(awk -v n="$VEHICLE_COUNT" -v arena_width="$ARENA_WIDTH" -v seed="$SEED_VAL" '
    BEGIN {
      if (seed != "") srand(seed); else srand();
      half_side = arena_width / 3.0 / 2.0;
      out = "";
      for (i = 0; i < n; i++) {
        x = -half_side + rand() * 2 * half_side;
        y = -half_side + rand() * 2 * half_side;
        h = -180.0 + rand() * 360.0;
        if (out != "") out = out ";";
        out = out x "," y "," h;
      }
      print out;
    }')
fi

echo "Generating dynamic pShare routes for shoreside..."
> plug_pshare_outputs.moos # Clear or create the file

# Loop to generate the pShare outputs for N vehicles
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  PORT=$((9200 + $i))
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="sim_$VEHICLE_NUM"
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
nsplug meta_shoreside.moos targ_shoreside.moos -f RUN_ID="$RUN_ID_VAL" FOV_VAL="$FOV_VAL"

echo "Assembling MOOS and BHV files for vehicles..."

# Loop to generate .moos and .bhv files for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  # Pad the vehicle number with leading zeros (e.g., 001, 002, ...)
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="sim_$VEHICLE_NUM"
  MOOS_PORT=$((9000 + $i))
  PSHARE_PORT=$((9200 + $i))
  #POLAR_PLOT_STR="0,100: 90,100: 180,100"
  POLAR_PLOT_STR="0,0: 30,40: 45,80: 90,90: 135,100: 180,60"

  # Fixed staggered-line start (default), or this vehicle's triple pulled
  # out of START_POSES_VAL (--random) if one was supplied.
  if [ -n "$START_POSES_VAL" ]; then
    IFS=';' read -ra START_POSES_ARR <<< "$START_POSES_VAL"
    IFS=',' read -r POSE_X POSE_Y POSE_H <<< "${START_POSES_ARR[$((i - 1))]}"
    START_POS_STR="x=$POSE_X,y=$POSE_Y,heading=$POSE_H"
  else
    START_POS_STR="x=$((($i*4) - 10)),y=$((6 +($i%3))),heading=-$(($i*5))"
  fi

  # Generate .moos and .bhv files
  nsplug meta_vehicle.moos "targ_${VNAME}.moos" -f VNAME="$VNAME" MOOS_PORT="$MOOS_PORT" PSHARE_PORT="$PSHARE_PORT" POLAR_PLOT="$POLAR_PLOT_STR" START_POS="$START_POS_STR"

  # Inject the A0, A1, B0, B1, GAM and FOV variables into the behavior file generation
  nsplug meta_vehicle.bhv "targ_${VNAME}.bhv" -f VNAME="$VNAME" POLAR_PLOT="$POLAR_PLOT_STR" RETURN_POS="0,-20" A0_VAL="$A0_VAL" A1_VAL="$A1_VAL" B0_VAL="$B0_VAL" B1_VAL="$B1_VAL" GAM_VAL="$GAM_VAL" FOV_VAL="$FOV_VAL"
done

echo "Launching Simulation..."
# Make sure to launch the newly generated targ_shoreside.moos, not the template!
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.6

# Loop to launch pAntler for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="sim_$VEHICLE_NUM"
  pAntler "targ_${VNAME}.moos" >& /dev/null &
  sleep 0.6
done

echo "Simulation running. Parameters: ${A0_VAL} ${A1_VAL} ${B0_VAL} ${B1_VAL} ${GAM_VAL}"
