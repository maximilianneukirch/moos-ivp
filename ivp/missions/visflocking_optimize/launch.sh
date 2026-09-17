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

# --- Kill previous processes before starting ---
# NOTE: the MOOS app name registered by uFlockEvaluator's binary is
# "pFlockEvaluator" (see ProcessConfig block), but the actual OS process
# (and thus what killall must match) is "uFlockEvaluator" -- the exe name.
# Killing "pFlockEvaluator" here was a no-op and left stale evaluator/MOOSDB
# processes running across sweep iterations, corrupting later runs.
echo "Cleaning up old processes..."
killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uTimerScript uFlockEvaluator
sleep 1
# ----------------------------------------------------

# Number of vehicles (override for small diagnostic runs, e.g.
# VEHICLE_COUNT=2 ./launch.sh ... with a two-element START_POSES list)
VEHICLE_COUNT=${VEHICLE_COUNT:-10}

echo "Generating dynamic pShare routes for shoreside..."
> plug_pshare_outputs.moos # Clear or create the file

# Loop to generate the pShare outputs for N vehicles
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  PORT=$((9200 + $i))
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
nsplug meta_shoreside.moos targ_shoreside.moos -f RUN_ID="$RUN_ID_VAL" FOV_VAL="$FOV_VAL" RES_VAL="$RES_VAL" LOGGING="$LOGGING_VAL"

echo "Assembling MOOS and BHV files for vehicles..."

# Loop to generate .moos and .bhv files for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  # Pad the vehicle number with leading zeros (e.g., 001, 002, ...)
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  MOOS_PORT=$((9000 + $i))
  PSHARE_PORT=$((9200 + $i))
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
  nsplug meta_vehicle.moos "targ_${VNAME}.moos" -f VNAME="$VNAME" MOOS_PORT="$MOOS_PORT" PSHARE_PORT="$PSHARE_PORT" POLAR_PLOT="$POLAR_PLOT_STR" START_POS="$START_POS_STR" LOGGING="$LOGGING_VAL" DYNAMICS="$DYNAMICS_VAL"
  
  # Inject the A0, A1, B0, B1, GAM and FOV variables into the behavior file generation
  nsplug meta_vehicle.bhv "targ_${VNAME}.bhv" -f VNAME="$VNAME" POLAR_PLOT="$POLAR_PLOT_STR" RETURN_POS="0,-20" A0_VAL="$A0_VAL" A1_VAL="$A1_VAL" B0_VAL="$B0_VAL" B1_VAL="$B1_VAL" GAM_VAL="$GAM_VAL" FOV_VAL="$FOV_VAL"
done

echo "Launching Simulation..."
# Make sure to launch the newly generated targ_shoreside.moos, not the template!
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.4

# Loop to launch pAntler for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  pAntler "targ_${VNAME}.moos" >& /dev/null &
  sleep 0.4
done

echo "Simulation running. Hit [Deploy] in the pMarineViewer window to start."