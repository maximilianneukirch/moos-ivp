#!/bin/bash

# Increase OS limits for massive multi-vehicle simulations
ulimit -n 4096  # Increases the max number of open files/sockets
ulimit -u 8192  # Increases the max number of user processes

# Number of vehicles (change this to your desired count)
VEHICLE_COUNT=50

# Waypoint routes (repeating as needed)
WPT_ROUTES=(
  "60,-40:180,-40:180,50:120,80:60,50"
  "180,-40:180,50:120,80:60,50:60,-40"
  "180,50:120,80:60,50:60,-40:180,-40"
)

echo "Assembling MOOS and BHV files..."

# Compile the shoreside directly (no need for nsplug since there are no macros in it anymore)
cp meta_shoreside.moos targ_shoreside.moos

# Generate vehicle moos/bhv files dynamically
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  MOOS_PORT=$((9000 + $i))
  PSHARE_PORT=$((9200 + $i))
  WPT_ROUTE=${WPT_ROUTES[$(( ($i-1) % ${#WPT_ROUTES[@]} ))]}

  nsplug meta_vehicle.moos "targ_${VNAME}.moos" -f VNAME="$VNAME" MOOS_PORT="$MOOS_PORT" PSHARE_PORT="$PSHARE_PORT" START_POS="x=$(($i*20)),y=-20,heading=180"
  nsplug meta_vehicle.bhv "targ_${VNAME}.bhv" -f WPT_ROUTE="$WPT_ROUTE" RETURN_POS="$(($i*20)),-20"
done

echo "Launching Simulation..."
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.5

for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  pAntler "targ_${VNAME}.moos" >& /dev/null &
  sleep 1.0
done

echo "Simulation running. Hit [Deploy] in the pMarineViewer window to start."