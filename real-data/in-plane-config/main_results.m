%% RESULTS FROM GYROSTABILIZED TABLE
% Author: Laura Train
% Date of the last update June 16 2021
%
% Compare the data taken from a gyrostabilized table excited by a known
% motion in four different architectures

%% Include paths

clear; clc; close all;

addpath architecture-1/
addpath architecture-2/
addpath architecture-3/
addpath architecture-4/

load('arch1_mov1_navCM.mat');
load('arch1_mov2_navCM.mat');
load('arch1_mov3_navCM.mat');

load('arch2_mov1_navCM.mat');
load('arch2_mov2_navCM.mat');
load('arch2_mov3_navCM.mat');


load('arch3_mov1_navCM.mat');
load('arch3_mov2_navCM.mat');
load('arch3_mov3_navCM.mat');


load('arch4_mov1_navCM.mat');
load('arch4_mov2_navCM.mat');
load('arch4_mov3_navCM.mat');


load('mov1_time.mat');
load('mov2_time.mat');
load('mov3_time.mat');


%% Motion 1

% simulate real motion
mov1.imuMAIN.t = arch1_mov1_navCM.t;

mov1.roll_true = deg2rad(20)*sin(pi/1.9*mov1.imuMAIN.t);
mov1.pitch_true = deg2rad(10)*cos(pi/1.9*mov1.imuMAIN.t);
mov1.yaw_true = -deg2rad(15)*sin(pi/3.7*mov1.imuMAIN.t);

% LQR
arch1_mov1_navCM.Jroll = LQR(arch1_mov1_navCM.t, mov1.roll_true, deg2rad(arch1_mov1_navCM.roll));
arch1_mov1_navCM.Jpitch = LQR(arch1_mov1_navCM.t, mov1.pitch_true, deg2rad(arch1_mov1_navCM.pitch));
arch1_mov1_navCM.Jyaw = LQR(arch1_mov1_navCM.t, mov1.yaw_true, deg2rad(arch1_mov1_navCM.yaw));
arch1_mov1_navCM.Jtotal = arch1_mov1_navCM.Jroll + arch1_mov1_navCM.Jpitch + arch1_mov1_navCM.Jyaw;

arch2_mov1_navCM.Jroll = LQR(arch2_mov1_navCM.t, mov1.roll_true, deg2rad(arch2_mov1_navCM.roll));
arch2_mov1_navCM.Jpitch = LQR(arch2_mov1_navCM.t, mov1.pitch_true, deg2rad(arch2_mov1_navCM.pitch));
arch2_mov1_navCM.Jyaw = LQR(arch2_mov1_navCM.t, mov1.yaw_true, deg2rad(arch2_mov1_navCM.yaw));
arch2_mov1_navCM.Jtotal = arch2_mov1_navCM.Jroll + arch2_mov1_navCM.Jpitch + arch2_mov1_navCM.Jyaw;

arch3_mov1_navCM.Jroll = LQR(arch3_mov1_navCM.t, mov1.roll_true, deg2rad(arch3_mov1_navCM.roll));
arch3_mov1_navCM.Jpitch = LQR(arch3_mov1_navCM.t, mov1.pitch_true, deg2rad(arch3_mov1_navCM.pitch));
arch3_mov1_navCM.Jyaw = LQR(arch3_mov1_navCM.t, mov1.yaw_true, deg2rad(arch3_mov1_navCM.yaw));
arch3_mov1_navCM.Jtotal = arch3_mov1_navCM.Jroll + arch3_mov1_navCM.Jpitch + arch3_mov1_navCM.Jyaw;

arch4_mov1_navCM.Jroll = LQR(arch4_mov1_navCM.t, mov1.roll_true, deg2rad(arch4_mov1_navCM.roll));
arch4_mov1_navCM.Jpitch = LQR(arch4_mov1_navCM.t, mov1.pitch_true, deg2rad(arch4_mov1_navCM.pitch));
arch4_mov1_navCM.Jyaw = LQR(arch4_mov1_navCM.t, mov1.yaw_true, deg2rad(arch4_mov1_navCM.yaw));
arch4_mov1_navCM.Jtotal = arch4_mov1_navCM.Jroll + arch4_mov1_navCM.Jpitch + arch4_mov1_navCM.Jyaw;

%% Motion 2

% simulate real motion
mov2.imuMAIN.t = arch1_mov2_navCM.t;

start_pos_mov2 = 51795;
end_pos_mov2 = 71444;
end_first_motion2 = 57254 - start_pos_mov2 ;
end_second_motion2 = 66025 - start_pos_mov2;
end_third_motion2 = end_pos_mov2 - start_pos_mov2 + 1;

mov2.roll_true(1:end_first_motion2) = deg2rad(0.1)*ones(length(mov2.imuMAIN.t(1:end_first_motion2)),1);
mov2.pitch_true(1:end_first_motion2) = deg2rad(0.21)*ones(length(mov2.imuMAIN.t(1:end_first_motion2)),1);
mov2.yaw_true(1:end_first_motion2) = deg2rad(50)*mov2.imuMAIN.t(1:end_first_motion2 );

mov2.roll_true(end_first_motion2:end_second_motion2) = deg2rad(0.48)*ones(length(mov2.imuMAIN.t(end_first_motion2:end_second_motion2)),1);
mov2.pitch_true(end_first_motion2:end_second_motion2) = deg2rad(0.59)*ones(length(mov2.imuMAIN.t(end_first_motion2:end_second_motion2)),1);
mov2.yaw_true(end_first_motion2:end_second_motion2) = deg2rad(100)*(mov2.imuMAIN.t(end_first_motion2:end_second_motion2) - mov2.imuMAIN.t(end_first_motion2)) + mov2.yaw_true(end_first_motion2 - 1);

mov2.roll_true(end_second_motion2:end_third_motion2) = deg2rad(1.09)*ones(length(mov2.imuMAIN.t(end_second_motion2:end_third_motion2)),1);
mov2.pitch_true(end_second_motion2:end_third_motion2) = deg2rad(1.15)*ones(length(mov2.imuMAIN.t(end_second_motion2:end_third_motion2)),1);
mov2.yaw_true(end_second_motion2:end_third_motion2) = deg2rad(150)*(mov2.imuMAIN.t(end_second_motion2:end_third_motion2) - mov2.imuMAIN.t(end_second_motion2)) + mov2.yaw_true(end_second_motion2 - 1);

% LQR
arch1_mov2_navCM.Jroll = LQR(arch1_mov2_navCM.t, mov2.roll_true', deg2rad(arch1_mov2_navCM.roll));
arch1_mov2_navCM.Jpitch = LQR(arch1_mov2_navCM.t, mov2.pitch_true', deg2rad(arch1_mov2_navCM.pitch));
arch1_mov2_navCM.Jyaw = LQR(arch1_mov2_navCM.t, mov2.yaw_true', deg2rad(arch1_mov2_navCM.yaw));
arch1_mov2_navCM.Jtotal = arch1_mov2_navCM.Jroll + arch1_mov2_navCM.Jpitch + arch1_mov2_navCM.Jyaw;

arch2_mov2_navCM.Jroll = LQR(arch2_mov2_navCM.t, mov2.roll_true', deg2rad(arch2_mov2_navCM.roll));
arch2_mov2_navCM.Jpitch = LQR(arch2_mov2_navCM.t, mov2.pitch_true', deg2rad(arch2_mov2_navCM.pitch));
arch2_mov2_navCM.Jyaw = LQR(arch2_mov2_navCM.t, mov2.yaw_true', deg2rad(arch2_mov2_navCM.yaw));
arch2_mov2_navCM.Jtotal = arch2_mov2_navCM.Jroll + arch2_mov2_navCM.Jpitch + arch2_mov2_navCM.Jyaw;

arch3_mov2_navCM.Jroll = LQR(arch3_mov2_navCM.t, mov2.roll_true', deg2rad(arch3_mov2_navCM.roll));
arch3_mov2_navCM.Jpitch = LQR(arch3_mov2_navCM.t, mov2.pitch_true', deg2rad(arch3_mov2_navCM.pitch));
arch3_mov2_navCM.Jyaw = LQR(arch3_mov2_navCM.t, mov2.yaw_true', deg2rad(arch3_mov2_navCM.yaw));
arch3_mov2_navCM.Jtotal = arch3_mov2_navCM.Jroll + arch3_mov2_navCM.Jpitch + arch3_mov2_navCM.Jyaw;

arch4_mov2_navCM.Jroll = LQR(arch4_mov2_navCM.t, mov2.roll_true', deg2rad(arch4_mov2_navCM.roll));
arch4_mov2_navCM.Jpitch = LQR(arch4_mov2_navCM.t, mov2.pitch_true', deg2rad(arch4_mov2_navCM.pitch));
arch4_mov2_navCM.Jyaw = LQR(arch4_mov2_navCM.t, mov2.yaw_true', deg2rad(arch4_mov2_navCM.yaw));
arch4_mov2_navCM.Jtotal = arch4_mov2_navCM.Jroll + arch4_mov2_navCM.Jpitch + arch4_mov2_navCM.Jyaw;


%% Motion 3

% simulate real motion
mov3.imuMAIN.t = arch1_mov3_navCM.t;
mov3.imuMAIN.ini_align = deg2rad([20, 2.2, 200]);

mov3.roll_true = deg2rad(0.8)*cos(pi/0.75*mov3.imuMAIN.t);
mov3.pitch_true = -deg2rad(0.8)*cos(pi/0.75*mov3.imuMAIN.t);
mov3.yaw_true = -deg2rad(20)*cos(pi/0.75*mov3.imuMAIN.t) + deg2rad(20);

% LQR
arch1_mov3_navCM.Jroll = LQR(arch1_mov3_navCM.t, mov3.roll_true, deg2rad(arch1_mov3_navCM.roll));
arch1_mov3_navCM.Jpitch = LQR(arch1_mov3_navCM.t, mov3.pitch_true, deg2rad(arch1_mov3_navCM.pitch));
arch1_mov3_navCM.Jyaw = LQR(arch1_mov3_navCM.t, mov3.yaw_true, deg2rad(arch1_mov3_navCM.yaw));
arch1_mov3_navCM.Jtotal = arch1_mov3_navCM.Jroll + arch1_mov3_navCM.Jpitch + arch1_mov3_navCM.Jyaw;

arch2_mov3_navCM.Jroll = LQR(arch2_mov3_navCM.t, mov3.roll_true, deg2rad(arch2_mov3_navCM.roll));
arch2_mov3_navCM.Jpitch = LQR(arch2_mov3_navCM.t, mov3.pitch_true, deg2rad(arch2_mov3_navCM.pitch));
arch2_mov3_navCM.Jyaw = LQR(arch2_mov3_navCM.t, mov3.yaw_true, deg2rad(arch2_mov3_navCM.yaw));
arch2_mov3_navCM.Jtotal = arch2_mov3_navCM.Jroll + arch2_mov3_navCM.Jpitch + arch2_mov3_navCM.Jyaw;

arch3_mov3_navCM.Jroll = LQR(arch3_mov3_navCM.t, mov3.roll_true, deg2rad(arch3_mov3_navCM.roll));
arch3_mov3_navCM.Jpitch = LQR(arch3_mov3_navCM.t, mov3.pitch_true, deg2rad(arch3_mov3_navCM.pitch));
arch3_mov3_navCM.Jyaw = LQR(arch3_mov3_navCM.t, mov3.yaw_true, deg2rad(arch3_mov3_navCM.yaw));
arch3_mov3_navCM.Jtotal = arch3_mov3_navCM.Jroll + arch3_mov3_navCM.Jpitch + arch3_mov3_navCM.Jyaw;

arch4_mov3_navCM.Jroll = LQR(arch4_mov3_navCM.t, mov3.roll_true, deg2rad(arch4_mov3_navCM.roll));
arch4_mov3_navCM.Jpitch = LQR(arch4_mov3_navCM.t, mov3.pitch_true, deg2rad(arch4_mov3_navCM.pitch));
arch4_mov3_navCM.Jyaw = LQR(arch4_mov3_navCM.t, mov3.yaw_true, deg2rad(arch4_mov3_navCM.yaw));
arch4_mov3_navCM.Jtotal = arch4_mov3_navCM.Jroll + arch4_mov3_navCM.Jpitch + arch4_mov3_navCM.Jyaw;

