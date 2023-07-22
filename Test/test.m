clear all;
close all;
clc;

syms R w1 w2 w3 u n r

B = [0, -1, R; 
    cos(pi/6), sin(pi/6), R;
    -cos(pi/6), sin(pi/6), R]

Bi = inv(B)
Bi3 = Bi*3

rot = [cos(u), -sin(u), 0;
    sin(u), cos(u), 0;
    0,0,1]

world = rot * Bi3 * [w1;w2;w3]

S = simplify(world)

