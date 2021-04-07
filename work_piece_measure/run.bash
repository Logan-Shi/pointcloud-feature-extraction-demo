# !/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    echo "running model/$a.ply"
    ./bin/main_workpiece model/$a.ply 10000 4 2 0.04 200 0.04 0.5 0.5 1.5
done

# ./bin/main_workpiece model/point_$1.ply $4 $2 $3 -500 $6 $5 0.5 0.5 1.5