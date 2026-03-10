#!/bin/bash
echo "Assembling MOOS and BHV files..."

# Vehicle 1: Alpha (Original Order)
nsplug meta_vehicle.moos targ_alpha.moos -f VNAME="alpha" MOOS_PORT="9001" PSHARE_PORT="9201" START_POS="x=0,y=-20,heading=180"
nsplug meta_vehicle.bhv targ_alpha.bhv -f WPT_ROUTE="60,-40:180,-40:180,50:120,80:60,50" RETURN_POS="0,-20"

# Vehicle 2: Bravo (Shifted Order)
nsplug meta_vehicle.moos targ_bravo.moos -f VNAME="bravo" MOOS_PORT="9002" PSHARE_PORT="9202" START_POS="x=20,y=-20,heading=180"
nsplug meta_vehicle.bhv targ_bravo.bhv -f WPT_ROUTE="180,-40:180,50:120,80:60,50:60,-40" RETURN_POS="20,-20"

# Vehicle 3: Charlie (Shifted Order)
nsplug meta_vehicle.moos targ_charlie.moos -f VNAME="charlie" MOOS_PORT="9003" PSHARE_PORT="9203" START_POS="x=40,y=-20,heading=180"
nsplug meta_vehicle.bhv targ_charlie.bhv -f WPT_ROUTE="180,50:120,80:60,50:60,-40:180,-40" RETURN_POS="40,-20"

echo "Launching Simulation..."
pAntler shoreside.moos >& /dev/null &
sleep 0.5
pAntler targ_alpha.moos &
sleep 0.5
pAntler targ_bravo.moos &
sleep 0.5
pAntler targ_charlie.moos >& /dev/null &

echo "Simulation running. Hit [Deploy] in the pMarineViewer window to start."
