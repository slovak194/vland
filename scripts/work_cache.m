clc
clear

fan.BatS = 12;
fan.BatV = 44.4;
fan.I = 102;

bat.S = 6;
bat.C = 35;
bat.Capacity = 5000;
bat.I = bat.Capacity * bat.C / 1000;



operation_time = 60*(bat.Capacity/1000)/fan.I