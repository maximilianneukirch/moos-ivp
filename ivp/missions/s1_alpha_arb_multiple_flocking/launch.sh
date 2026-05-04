#!/bin/bash

# Number of vehicles (change this to your desired count)
VEHICLE_COUNT=5

# Waypoint routes (repeating as needed)
WPT_ROUTES=(
  "60,-40:180,-40:180,50:120,80:60,50"
  "180,-40:180,50:120,80:60,50:60,-40"
  "180,50:120,80:60,50:60,-40:180,-40"
)

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

  echo "  Output = src_name=VPF_$VNAME_UPPER, dest_name=VPF, route=localhost:$PORT" >> plug_pshare_outputs.moos
  echo "" >> plug_pshare_outputs.moos
done

# Generate the final shoreside file (nsplug will automatically absorb the #include file)
nsplug meta_shoreside.moos targ_shoreside.moos -f

echo "Assembling MOOS and BHV files for vehicles..."

# Loop to generate .moos and .bhv files for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  # Pad the vehicle number with leading zeros (e.g., 001, 002, ...)
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  MOOS_PORT=$((9000 + $i))
  PSHARE_PORT=$((9200 + $i))

  # Select waypoint route (cycle through the array)
  WPT_ROUTE=${WPT_ROUTES[$(( ($i-1) % ${#WPT_ROUTES[@]} ))]}

  # Generate .moos and .bhv files
  nsplug meta_vehicle.moos "targ_${VNAME}.moos" -f VNAME="$VNAME" MOOS_PORT="$MOOS_PORT" PSHARE_PORT="$PSHARE_PORT" START_POS="x=$(($i*20)),y=-20,heading=180"
  nsplug meta_vehicle.bhv "targ_${VNAME}.bhv" -f VNAME="$VNAME" WPT_ROUTE="$WPT_ROUTE" RETURN_POS="0,-20"
done

echo "Launching Simulation..."
# Make sure to launch the newly generated targ_shoreside.moos, not the template!
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.5

# Loop to launch pAntler for each vehicle
for ((i=1; i<=$VEHICLE_COUNT; i++)); do
  VEHICLE_NUM=$(printf "%03d" $i)
  VNAME="alpha_$VEHICLE_NUM"
  pAntler "targ_${VNAME}.moos" >& /dev/null &
  sleep 0.5
done

echo "Simulation running. Hit [Deploy] in the pMarineViewer window to start."