#!/bin/bash
# -------------------------------------------------------
# FILE: launch.sh
# -------------------------------------------------------
TIME_WARP=1
SHORE_LISTEN="9300"

# 1. Generate the Shoreside configuration
echo "Assembling Shoreside..."
nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
       SNAME="shoreside" SHARE_LISTEN=$SHORE_LISTEN SPORT="9000"

# 2. Generate the 10 Vehicle configurations
for i in $(seq 1 10); do
    VNAME="sailor$i"
    VPORT=$((9000 + $i))       # MOOSDB port (9001, 9002, etc.)
    LPORT=$((9300 + $i))       # pShare listen port
    
    # Stagger their starting positions slightly so they don't spawn on top of each other
    START_POS="x=$((i*10 - 50)),y=-20,heading=180,speed=0"
    
    echo "Assembling $VNAME..."
    nsplug meta_vehicle.moos targ_$VNAME.moos -f WARP=$TIME_WARP \
           VNAME=$VNAME VPORT=$VPORT SHARE_LISTEN=$LPORT \
           SHORE_LISTEN=$SHORE_LISTEN START_POS=$START_POS
           
    nsplug meta_vehicle.bhv targ_$VNAME.bhv -f VNAME=$VNAME
done

# 3. Launch everything using pAntler
echo "Launching Shoreside..."
pAntler targ_shoreside.moos >& /dev/null &
sleep 0.25

for i in $(seq 1 10); do
    VNAME="sailor$i"
    echo "Launching $VNAME..."
    pAntler targ_$VNAME.moos >& /dev/null &
    sleep 0.25
done

echo "Done launching. Watch pMarineViewer!"
