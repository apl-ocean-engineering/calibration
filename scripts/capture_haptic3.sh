ffmpeg -y -thread_queue_size 8192 \
       -i rtsp://stream:knot4ewe@192.168.4.23/ -codec copy $1
