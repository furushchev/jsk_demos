#!/bin/bash

gnuplot -e "
set terminal png size 1920,1080 enhanced font 'Helvetica,20';
set output 'pick_cup_arm.png';
set xlabel 'x position from robot base';
set ylabel 'y position from robot base';
set zlabel 'z position from robot base' rotate by 90;
set zrange [0.5:1.1];
set grid;
splot
'pick_cup_arm.csv' index 1 w p ps 2.8 pt 20 lt 2,
'pick_cup_arm.csv' index 2 w p ps 2.8 pt 20 lt 3,
'pick_cup_arm.csv' index 3 w p ps 2.8 pt 20 lt 4,
'pick_cup_arm.csv' index 4 w p ps 2.8 pt 20 lt 5,
'pick_cup_arm.csv' index 5 w p ps 2.8 pt 20 lt 6,
'pick_cup_arm.csv' index 6 w p ps 2.8 pt 20 lt 7,
'pick_cup_arm.csv' index 7 w p ps 2.8 pt 20 lt 8,
'pick_cup_arm.csv' index 8 w p ps 2.8 pt 20 lt 9,
'pick_cup_arm.csv' index 9 w p ps 2.8 pt 20 lt 10,
'pick_cup_arm.csv' index 10 w p ps 2.8 pt 20 lt 11,
'pick_cup_arm.csv' index 11 w p ps 2.8 pt 20 lt 12,
'pick_cup_arm.csv' index 12 w p ps 2.8 pt 20 lt 13;
"

# 'pick_cup_arm.csv' index 0 w p ps 2.8 pt 20 lt 1,
