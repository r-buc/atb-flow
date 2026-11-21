#!/bin/bash
set -ex

TASMOTA_HOST=${TASMOTA_HOST:-tasmota-hkv-test}
TASMOTA_PORT=${TASMOTA_PORT:-80}
REFERER_HEADER="Referer: http://$TASMOTA_HOST/"

if [ "$1" = "--setup-template" ]; then
    echo "Configuring Tasmota GPIO template (DS18x20 on GPIO4, I2C on GPIO8/9), disabling native INA219 polling (I2CDriver14 0), and enabling BLE MI32 (SetOption115 1)..."
    # GPIO4  = DS18x20 (1312)
    # GPIO8  = I2C SDA (640)
    # GPIO9  = I2C SCL (608)
    # SetOption115 1 = Enable ESP32 MI32 BLE driver (passive BTHome v2 / ATC / PVVX room sensors)
    # Note: MI32Period is non-persistent in Tasmota and is set to 60s on boot by sensors.be
    # Motor pins (5, 6, 7, 14, 15, 16, 17, 18, 21) remain None (0) for Berry gpio control
    curl -sS -G -H "$REFERER_HEADER" "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" \
        --data-urlencode 'cmnd=Backlog I2CDriver14 0; SetOption115 1; Template {"NAME":"ATB-Flow-HKV","GPIO":[0,0,0,0,1312,0,0,0,640,608,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"FLAG":0,"BASE":1}; Module 0'
    echo
    echo "Waiting 5s for Tasmota to reboot after template change..."
    sleep 5
fi

echo "Deleting existing Berry script files on $TASMOTA_HOST:$TASMOTA_PORT"
for file in *.be; do
    [ -f "$file" ] || continue
    curl -sS -o /dev/null -H "$REFERER_HEADER" "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsd?delete=/$file"
    curl -sS -o /dev/null -H "$REFERER_HEADER" "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsd?delete=/${file%.be}.bec"
done
curl -sS -G -H "$REFERER_HEADER" "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" --data-urlencode 'cmnd=brrestart'; echo

echo "Uploading Berry script files"
for file in *.be; do
    [ -f "$file" ] || continue
    fsize=$(stat -c%s "$file")
    echo "  -> $file ($fsize bytes)"
    curl -sS -o /dev/null -H "Expect:" -H "$REFERER_HEADER" -F "ufsu=@./$file" "http://$TASMOTA_HOST:$TASMOTA_PORT/ufsu?fsz=$fsize"
    sleep 1
done

if [ -z "$GITHUB_ACTIONS" ]; then
    echo "Restarting Tasmota device..."
    curl -sS -G -H "$REFERER_HEADER" "http://$TASMOTA_HOST:$TASMOTA_PORT/cm" --data-urlencode 'cmnd=restart 1'; echo
fi
