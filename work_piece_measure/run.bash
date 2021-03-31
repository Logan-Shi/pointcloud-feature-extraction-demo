# !/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    echo "running model/$a.ply"
    ./bin/main_workpiece model/point_$a.ply 1000 4 $2 -500 40 0.03 0.5 0.5 1.5
done

# ./bin/main_workpiece model/point_$1.ply $4 $2 $3 -500 $6 $5 0.5 0.5 1.5