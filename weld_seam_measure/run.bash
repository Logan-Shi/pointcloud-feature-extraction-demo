# !/bin/bash
# a = 0
# until [ ! $a -lt $1 ]
# do
# 	a=`expr $a + 1`
#     # echo "running model/$a.ply"
#     ./bin/main_weldseam model/$a.ply 10000 4 2 0.1 30 0.1 0.5 0.5 1.5
# done

# ./bin/main_weldseam model/1.ply 0 -5 -30 0.3 60 1 180 0.5 1.5
./bin/main_weldseam model/5.ply 1 -5 -30 0.3 60 1 180 0.5 1.5