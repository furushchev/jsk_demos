#!/bin/bash

gnuplot -e "
set terminal png size 1920,1080 enhanced font 'Helvetica,20'
set output 'base.png'
plot [2.9:2.97] 'base.csv' index 0 w p ps 8 pt 20 lt 1 title 'SUCCESS', 'base.csv' index 1 w p ps 8 pt 20 lt 3 title 'FAILURE'
"
