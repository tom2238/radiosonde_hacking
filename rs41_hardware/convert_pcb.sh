#!/bin/sh
# Crop image, quality 20

for f in $1/*.jpg; do
  echo "Processing $f file..."
  fname=`basename $f`
  # take action on each file. $f store current file name
  #ffmpeg -i "$f" -vf "crop=4500:1400:150:150" -q:v 20 "$fname"
  #ffmpeg -i "$f" -vf "crop=4500:1400:50:120" -q:v 20 "$fname"
done
