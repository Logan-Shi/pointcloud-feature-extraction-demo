#!/bin/bash
a = 0
until [ ! $a -lt $1 ]
do
	a=`expr $a + 1`
    echo "running model/$a.ply"
    ./bin/work_piece_measure model/$a.ply 1000 $2 2 -500 500 $3 0.5 0.5 1.5
done