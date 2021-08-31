# !/bin/bash
# a = 0
# until [ ! $a -lt $1 ]
# do
# 	a=`expr $a + 1`
#     # echo "running model/$a.ply"
#     ./bin/main_weldseam model/$a.ply 10000 4 2 0.1 30 0.1 0.5 0.5 1.5
# done

# ./bin/main_weldseam model/1.ply 0 -5 -30 0.3 1 1 180 0.5 1.5 # before belt grind
# ./bin/main_weldseam model/5.ply 0 -28.8 55.2 0.25 1 1 180 0.5 1.5
# ./bin/main_weldseam model/7.ply 0 9.6 22.2 0.15 1 1 180 0.5 1.5 # after belt grind
# ./bin/main_weldseam model/9.ply 0 13.6 -27.8 0.15 1 1 180 0.5 1.5
./bin/main_weldseam model/10.ply 0 0 -16.1 0.15 1 1 180 0.5 1.5