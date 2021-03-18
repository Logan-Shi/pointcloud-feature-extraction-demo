#!/bin/bash
for num in {1..8..1}
do
echo "running $num"
./bin/interactive_icp model/test.ply model/$num.ply 1
done