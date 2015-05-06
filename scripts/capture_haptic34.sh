#!/usr/bin/env sh


## Choice one, one ffmpeg process
ffmpeg -y -thread_queue_size 16536 \
       -i rtsp://stream:knot4ewe@192.168.4.23/ \
       -i rtsp://stream:knot4ewe@192.168.4.24/ \
       -vcodec copy -map 0:0 haptic3.mp4 \
       -vcodec copy -map 1:0 haptic4.mp4

# Choice two, two ffmpeg processes
#ffmpeg -y -thread_message_queue_size 1024 \
#       -i rtsp://stream:knot4ewe@10.0.76.11/big  \
#       -vcodec copy -map 0:0 haptic3.mp4    & 
#
#ffmpeg -y -thread_message_queue_size 1024 \
#       -i rtsp://stream:knot4ewe@10.0.76.2/big  \
#       -vcodec copy -map 0:0 haptic4.mp4    & 
#
#wait
