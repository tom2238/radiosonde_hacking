# Habitat uploader for sounding sonde

Sonde -> RTL-SDR -> sox -> nrz_decoder -> habitat_upload -> tracker.habhub.org 
* Thanks: https://github.com/PiInTheSky/lora-gateway
* Thanks: https://revspace.nl/TTNHABBridge

```
Habitat uploader
Usage: ./habitat_upload -L lat -l lon -C callsign -r radio -a antenna -c comment -P ratio| -h
  -L <latitude>   Latitude position of station, 48.478645
  -l <longtitude> Longtitude position of station, -2.568855
  -C <callsign>   Station callsign
  -r <radio>      Station radio name
  -a <antenna>    Station antenna type
  -c <comment>    Comment section
  -P <ratio>      Packet to position ratio (send station position every N packets)
  -h            Show this help
                Build: 13:52:46 Jul 23 2022, GCC 5.3.0
```

## Habitat upload
* 
```sox -t pulseaudio default -t wav - 2>/dev/null | ./decoder -i - -L 62 -b 4800 -P 3 | ./habitat_upload -L "$LATITUDE" -l "$LONGTITUDE" -C "$CALLSIGN" -r "$RADIO" -a "$ANTENNA" -c "$COMMENT" -P $STATION_RATIO
```

