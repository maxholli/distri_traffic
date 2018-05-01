#!/bin/bash

for i in {1..300}
do
    echo $i
    python runner.py $i >> "output/summary_dumb.txt" 2>/dev/null
done
