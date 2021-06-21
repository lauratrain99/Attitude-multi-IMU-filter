%% RESULTS FROM GYROSTABILIZED TABLE
% Author: Laura Train
% Date of the last update June 17 2021
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
load('arch1_mov4_navCM.mat');
load('arch1_mov5_navCM.mat');

load('arch2_mov1_navCM.mat');
load('arch2_mov2_navCM.mat');
load('arch2_mov3_navCM.mat');
load('arch2_mov4_navCM.mat');
load('arch2_mov5_navCM.mat');

load('arch3_mov1_navCM.mat');
load('arch3_mov2_navCM.mat');
load('arch3_mov3_navCM.mat');
load('arch3_mov4_navCM.mat');
load('arch3_mov5_navCM.mat');

load('arch4_mov1_navCM.mat');
load('arch4_mov2_navCM.mat');
load('arch4_mov3_navCM.mat');
load('arch4_mov4_navCM.mat');
load('arch4_mov5_navCM.mat');

load('mov1_time.mat');
load('mov2_time.mat');
load('mov3_time.mat');
load('mov4_time.mat');
load('mov5_time.mat');

%% Motion 1
% simulate real motion
mov1.imuMAIN.t = arch1_mov1_navCM.t;

mov1.roll_true = deg2rad(20)*cos(pi/2.3*mov1.imuMAIN.t);
mov1.pitch_true = deg2rad(2.3)*cos(pi/2.3*mov1.imuMAIN.t);
mov1.yaw_true = deg2rad(0.25)*cos(pi/1.15*mov1.imuMAIN.t) + deg2rad(3);

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

mov2.roll_true = -deg2rad(2.3)*cos(pi/2.3*mov2.imuMAIN.t);
mov2.pitch_true = deg2rad(20)*cos(pi/2.3*mov2.imuMAIN.t);
mov2.yaw_true = -deg2rad(0.5)*cos(pi/1.15*mov2.imuMAIN.t) + deg2rad(5);

arch1_mov2_navCM.Jroll = LQR(arch1_mov2_navCM.t, mov2.roll_true, deg2rad(arch1_mov2_navCM.roll));
arch1_mov2_navCM.Jpitch = LQR(arch1_mov2_navCM.t, mov2.pitch_true, deg2rad(arch1_mov2_navCM.pitch));
arch1_mov2_navCM.Jyaw = LQR(arch1_mov2_navCM.t, mov2.yaw_true, deg2rad(arch1_mov2_navCM.yaw));
arch1_mov2_navCM.Jtotal = arch1_mov2_navCM.Jroll + arch1_mov2_navCM.Jpitch + arch1_mov2_navCM.Jyaw;

arch2_mov2_navCM.Jroll = LQR(arch2_mov2_navCM.t, mov2.roll_true, deg2rad(arch2_mov2_navCM.roll));
arch2_mov2_navCM.Jpitch = LQR(arch2_mov2_navCM.t, mov2.pitch_true, deg2rad(arch2_mov2_navCM.pitch));
arch2_mov2_navCM.Jyaw = LQR(arch2_mov2_navCM.t, mov2.yaw_true, deg2rad(arch2_mov2_navCM.yaw));
arch2_mov2_navCM.Jtotal = arch2_mov2_navCM.Jroll + arch2_mov2_navCM.Jpitch + arch2_mov2_navCM.Jyaw;

arch3_mov2_navCM.Jroll = LQR(arch3_mov2_navCM.t, mov2.roll_true, deg2rad(arch3_mov2_navCM.roll));
arch3_mov2_navCM.Jpitch = LQR(arch3_mov2_navCM.t, mov2.pitch_true, deg2rad(arch3_mov2_navCM.pitch));
arch3_mov2_navCM.Jyaw = LQR(arch3_mov2_navCM.t, mov2.yaw_true, deg2rad(arch3_mov2_navCM.yaw));
arch3_mov2_navCM.Jtotal = arch3_mov2_navCM.Jroll + arch3_mov2_navCM.Jpitch + arch3_mov2_navCM.Jyaw;

arch4_mov2_navCM.Jroll = LQR(arch4_mov2_navCM.t, mov2.roll_true, deg2rad(arch4_mov2_navCM.roll));
arch4_mov2_navCM.Jpitch = LQR(arch4_mov2_navCM.t, mov2.pitch_true, deg2rad(arch4_mov2_navCM.pitch));
arch4_mov2_navCM.Jyaw = LQR(arch4_mov2_navCM.t, mov2.yaw_true, deg2rad(arch4_mov2_navCM.yaw));
arch4_mov2_navCM.Jtotal = arch4_mov2_navCM.Jroll + arch4_mov2_navCM.Jpitch + arch4_mov2_navCM.Jyaw;

%% Motion 3
% simulate real motion
mov3.imuMAIN.t = arch1_mov3_navCM.t;

mov3.roll_true = -deg2rad(0.1)*cos(pi/2.3*mov3.imuMAIN.t);
mov3.pitch_true = deg2rad(0.1)*cos(pi/1.15*mov3.imuMAIN.t);
mov3.yaw_true = deg2rad(20)*cos(pi/2.3*mov3.imuMAIN.t);

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

%% Motion 4
% simulate real motion
mov4.imuMAIN.t = arch1_mov4_navCM.t;

mov4.roll_true = deg2rad(20)*cos(pi/2.3*mov4.imuMAIN.t )  ;
mov4.pitch_true = deg2rad(6)*cos(pi/2.3*mov4.imuMAIN.t - 2.3/2);
mov4.yaw_true = deg2rad(5)*cos(pi/2.3*mov4.imuMAIN.t) + deg2rad(15);

arch1_mov4_navCM.Jroll = LQR(arch1_mov4_navCM.t, mov4.roll_true, deg2rad(arch1_mov4_navCM.roll));
arch1_mov4_navCM.Jpitch = LQR(arch1_mov4_navCM.t, mov4.pitch_true, deg2rad(arch1_mov4_navCM.pitch));
arch1_mov4_navCM.Jyaw = LQR(arch1_mov4_navCM.t, mov4.yaw_true, deg2rad(arch1_mov4_navCM.yaw));
arch1_mov4_navCM.Jtotal = arch1_mov4_navCM.Jroll + arch1_mov4_navCM.Jpitch + arch1_mov4_navCM.Jyaw;

arch2_mov4_navCM.Jroll = LQR(arch2_mov4_navCM.t, mov4.roll_true, deg2rad(arch2_mov4_navCM.roll));
arch2_mov4_navCM.Jpitch = LQR(arch2_mov4_navCM.t, mov4.pitch_true, deg2rad(arch2_mov4_navCM.pitch));
arch2_mov4_navCM.Jyaw = LQR(arch2_mov4_navCM.t, mov4.yaw_true, deg2rad(arch2_mov4_navCM.yaw));
arch2_mov4_navCM.Jtotal = arch2_mov4_navCM.Jroll + arch2_mov4_navCM.Jpitch + arch2_mov4_navCM.Jyaw;

arch3_mov4_navCM.Jroll = LQR(arch3_mov4_navCM.t, mov4.roll_true, deg2rad(arch3_mov4_navCM.roll));
arch3_mov4_navCM.Jpitch = LQR(arch3_mov4_navCM.t, mov4.pitch_true, deg2rad(arch3_mov4_navCM.pitch));
arch3_mov4_navCM.Jyaw = LQR(arch3_mov4_navCM.t, mov4.yaw_true, deg2rad(arch3_mov4_navCM.yaw));
arch3_mov4_navCM.Jtotal = arch3_mov4_navCM.Jroll + arch3_mov4_navCM.Jpitch + arch3_mov4_navCM.Jyaw;

arch4_mov4_navCM.Jroll = LQR(arch4_mov4_navCM.t, mov4.roll_true, deg2rad(arch4_mov4_navCM.roll));
arch4_mov4_navCM.Jpitch = LQR(arch4_mov4_navCM.t, mov4.pitch_true, deg2rad(arch4_mov4_navCM.pitch));
arch4_mov4_navCM.Jyaw = LQR(arch4_mov4_navCM.t, mov4.yaw_true, deg2rad(arch4_mov4_navCM.yaw));
arch4_mov4_navCM.Jtotal = arch4_mov4_navCM.Jroll + arch4_mov4_navCM.Jpitch + arch4_mov4_navCM.Jyaw;

%% Motion 5
% simulate real motion
start_pos_mov5 = 16405;
end_pos_mov5 = 18695;

mov5.imuMAIN.t = arch1_mov5_navCM.t;

end_first_motion5 = 17227 - start_pos_mov5 ;
end_second_motion5 = 18274 - start_pos_mov5;
end_third_motion5 = end_pos_mov5 - start_pos_mov5 + 1;

mov5.roll_true(1:end_first_motion5) = deg2rad(0.25)*ones(length(mov5.imuMAIN.t(1:end_first_motion5)),1);
mov5.pitch_true(1:end_first_motion5) = deg2rad(0.2)*ones(length(mov5.imuMAIN.t(1:end_first_motion5)),1);
mov5.yaw_true(1:end_first_motion5) = deg2rad(50)*mov5.imuMAIN.t(1:end_first_motion5 );

mov5.roll_true(end_first_motion5:end_second_motion5) = deg2rad(0.5)*ones(length(mov5.imuMAIN.t(end_first_motion5:end_second_motion5)),1);
mov5.pitch_true(end_first_motion5:end_second_motion5) = deg2rad(0.2)*ones(length(mov5.imuMAIN.t(end_first_motion5:end_second_motion5)),1);
mov5.yaw_true(end_first_motion5:end_second_motion5) = deg2rad(100)*(mov5.imuMAIN.t(end_first_motion5:end_second_motion5) - mov5.imuMAIN.t(end_first_motion5)) + mov5.yaw_true(end_first_motion5 - 1);

mov5.roll_true(end_second_motion5:end_third_motion5) = deg2rad(1.2)*ones(length(mov5.imuMAIN.t(end_second_motion5:end_third_motion5)),1);
mov5.pitch_true(end_second_motion5:end_third_motion5) = -deg2rad(0.3)*ones(length(mov5.imuMAIN.t(end_second_motion5:end_third_motion5)),1);
mov5.yaw_true(end_second_motion5:end_third_motion5) = deg2rad(200)*(mov5.imuMAIN.t(end_second_motion5:end_third_motion5) - mov5.imuMAIN.t(end_second_motion5)) + mov5.yaw_true(end_second_motion5 - 1);

arch1_mov5_navCM.Jroll = LQR(arch1_mov5_navCM.t, mov5.roll_true', deg2rad(arch1_mov5_navCM.roll));
arch1_mov5_navCM.Jpitch = LQR(arch1_mov5_navCM.t, mov5.pitch_true', deg2rad(arch1_mov5_navCM.pitch));
arch1_mov5_navCM.Jyaw = LQR(arch1_mov5_navCM.t, mov5.yaw_true', deg2rad(arch1_mov5_navCM.yaw));
arch1_mov5_navCM.Jtotal = arch1_mov5_navCM.Jroll + arch1_mov5_navCM.Jpitch + arch1_mov5_navCM.Jyaw;

arch2_mov5_navCM.Jroll = LQR(arch2_mov5_navCM.t, mov5.roll_true', deg2rad(arch2_mov5_navCM.roll));
arch2_mov5_navCM.Jpitch = LQR(arch2_mov5_navCM.t, mov5.pitch_true', deg2rad(arch2_mov5_navCM.pitch));
arch2_mov5_navCM.Jyaw = LQR(arch2_mov5_navCM.t, mov5.yaw_true', deg2rad(arch2_mov5_navCM.yaw));
arch2_mov5_navCM.Jtotal = arch2_mov5_navCM.Jroll + arch2_mov5_navCM.Jpitch + arch2_mov5_navCM.Jyaw;

arch3_mov5_navCM.Jroll = LQR(arch3_mov5_navCM.t, mov5.roll_true', deg2rad(arch3_mov5_navCM.roll));
arch3_mov5_navCM.Jpitch = LQR(arch3_mov5_navCM.t, mov5.pitch_true', deg2rad(arch3_mov5_navCM.pitch));
arch3_mov5_navCM.Jyaw = LQR(arch3_mov5_navCM.t, mov5.yaw_true', deg2rad(arch3_mov5_navCM.yaw));
arch3_mov5_navCM.Jtotal = arch3_mov5_navCM.Jroll + arch3_mov5_navCM.Jpitch + arch3_mov5_navCM.Jyaw;

arch4_mov5_navCM.Jroll = LQR(arch4_mov5_navCM.t, mov5.roll_true', deg2rad(arch4_mov5_navCM.roll));
arch4_mov5_navCM.Jpitch = LQR(arch4_mov5_navCM.t, mov5.pitch_true', deg2rad(arch4_mov5_navCM.pitch));
arch4_mov5_navCM.Jyaw = LQR(arch4_mov5_navCM.t, mov5.yaw_true', deg2rad(arch4_mov5_navCM.yaw));
arch4_mov5_navCM.Jtotal = arch4_mov5_navCM.Jroll + arch4_mov5_navCM.Jpitch + arch4_mov5_navCM.Jyaw;
