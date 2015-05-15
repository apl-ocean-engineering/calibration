ffmpeg -y -thread_queue_size 8192 \
       -i rtsp://stream:knot4ewe@10.95.76.233/ -codec copy $1
