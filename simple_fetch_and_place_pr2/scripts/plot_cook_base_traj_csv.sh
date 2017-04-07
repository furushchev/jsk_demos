#!/bin/bash

gnuplot -e "
set terminal png size 1920,1080 enhanced font 'Helvetica,20';
set output 'cook_base_traj.png';
plot
'cook_base_traj.csv' index 0 w p ps 2.8 pt 20 lt 1,
'cook_base_traj.csv' index 1 w p ps 2.8 pt 20 lt 2,
'cook_base_traj.csv' index 2 w p ps 2.8 pt 20 lt 3,
'cook_base_traj.csv' index 3 w p ps 2.8 pt 20 lt 4,
'cook_base_traj.csv' index 4 w p ps 2.8 pt 20 lt 5,
'cook_base_traj.csv' index 5 w p ps 2.8 pt 20 lt 6,
'cook_base_traj.csv' index 6 w p ps 2.8 pt 20 lt 7,
'cook_base_traj.csv' index 7 w p ps 2.8 pt 20 lt 8;
"
