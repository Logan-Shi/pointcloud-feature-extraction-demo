#!/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    # echo "running model/$a.ply for $2 iterations"
    ./bin/target_ball_measure model/6.ply 100 0.03 1
    # ./bin/target_ball_measure model/$a.ply 100 0.001 1
    # ./bin/target_ball_measure model/$a.ply 1000 0.0001 0
done