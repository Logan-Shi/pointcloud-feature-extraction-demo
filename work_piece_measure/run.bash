# !/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    # echo "running model/$a.ply"
    ./bin/main_workpiece model/$a.ply 1000 4 2 0.1 30 0.1 0.5 0.5 1.5
done

# ./bin/main_workpiece model/1.ply 1000 15 2 0.05 30 0.07 0.5 0.5 1.5