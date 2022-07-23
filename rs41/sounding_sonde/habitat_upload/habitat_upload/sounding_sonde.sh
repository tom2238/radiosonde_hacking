#!/bin/sh
# Config
LATITUDE=48.9090167
LONGTITUDE=17.0385985
CALLSIGN="TOM2238"
RADIO="RTL-SDR"
ANTENNA="GP"
COMMENT="Test sonde RS41"
STATION_RATIO=300

sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 62 -b 4800 -P 3 | ./habitat_upload -L "$LATITUDE" -l "$LONGTITUDE" -C "$CALLSIGN" -r "$RADIO" -a "$ANTENNA" -c "$COMMENT" -P $STATION_RATIO

