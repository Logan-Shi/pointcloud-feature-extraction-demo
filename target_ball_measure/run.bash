#!/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    echo "running model/$a.ply for $2 iterations"
    ./bin/target_ball_measure model/$a.ply $2
done