clc;
clear;
close all;

free_joints = [4 7];

A = [1:7; 8:14; 15:21; 22:28; 29:35; 36:42; 43:49];

num_dofs = sum(free_joints);

A_new = A(free_joints, free_joints)

B = 8:14;

B_new = B(free_joints)