#!/bin/bash
set -e

TASMOTA_HOST=${TASMOTA_HOST:-tasmota-hkv-test}
TASMOTA_PORT=${TASMOTA_PORT:-80}

if [ "$1" = "--setup-template" ]; then
    echo "Configuring Tasmota GPIO template (DS18x20 on GPIO4, I2C on GPIO8/9) and disabling native INA219 polling (I2CDriver14 0)..."
    # GPIO4  = DS18x20 (1312)
    # GPIO8  = I2C SDA (640)
    # GPIO9  = I2C SCL (608)
    # Motor pins (5, 6, 7, 14, 15, 16, 17, 18, 21) remain None (0) for Berry gpio control
    curl -s "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" \
        --data-urlencode 'cmnd=Backlog I2CDriver14 0; Template {"NAME":"ATB-Flow-HKV","GPIO":[0,0,0,0,1312,0,0,0,640,608,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"FLAG":0,"BASE":1}; Module 0'
    echo
    echo "Waiting 5s for Tasmota to reboot after template change..."
    sleep 5
fi

echo "Deleting existing Berry script files on $TASMOTA_HOST:$TASMOTA_PORT"
for file in *.be; do
    [ -f "$file" ] || continue
    curl -s -o /dev/null "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsd?delete=/$file"
    curl -s -o /dev/null "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsd?delete=/${file%.be}.bec"
done
curl -s "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" --data-urlencode 'cmnd=brrestart'; echo

echo "Uploading Berry script files"
for file in *.be; do
    [ -f "$file" ] || continue
    echo "  -> $file"
    curl -s -o /dev/null -F "file=@./$file" "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsu"
done

if [ -z "$GITHUB_ACTIONS" ]; then
    echo "Restarting Tasmota device..."
    curl -s "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" --data-urlencode 'cmnd=restart 1'; echo
fi
