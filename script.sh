
tools/video_calibration -d ../data --fix-skew -c haptic4 -b april_poster_2in -m angular --calibration-db calibration_reference_haptic4_all_two.kch all ../data/datasets/calibration_reference/haptic4.mp4

tools/calibration_reproject --reference-db ../data/cache/E9C9740A25558DEB11E29044813121269F328324.kch --calibration-db calibration_reference_haptic4.kch --results-db calibration_reference_haptic4_results.kch 

tools/calibration_db_dump --results-db calibration_reference_haptic4_results.kch  > calibration_reference_against_reference.txt



