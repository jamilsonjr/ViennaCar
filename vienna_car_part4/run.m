clc
clear all
close all

g=9.81;

%% motor dq
%--------------------------------------------------------------------------
% Motor Parameters
%--------------------------------------------------------------------------
% base values
Vb = 76;
fb = 76;
wb = 2*pi*fb;
Pp = 2;
Ib = 160;
Imb = 120/sqrt(2);
% Equivalent Circuit Parameters
pp=2;
J=0.01;
Rs = 8.56e-3;
ls = 0.06292e-3;
LM = 1.0122e-3;
Rr = 2*5.10e-3;
lr = 0.06709e-3;
% stator and rotor self inductances for dq model
LS = ls + LM;
LR = lr + LM;
% dq model Matrices
LMatrix = [LS 0 LM 0; 0 LS 0 LM; LM 0 LR 0; 0 LM 0 LR];
RMatrix = [Rs 0 0 0; 0 Rs 0 0; 0 0 Rr 0; 0 0 0 Rr];
RLInv = RMatrix/(LMatrix);

%% sim
sim('Motor_dq_2017b.slx')
%% plot
figure(1)
plot(speed.Time,speed.Data*60/(2*pi))
hold on
plot(ref_speed.Time,ref_speed.Data)
title('Speed [rpm]')


figure(2)
plot(Torque.Time,Torque.Data)
title('Torque [Nm]')


figure(3)
plot(stator_currents.Time,stator_currents.Data)
title('stator currents [A]')


figure(4)
plot(rotor_currents.Time,rotor_currents.Data)
title('rotor currents [A]')


figure(5)
P_dq=Torque.Data.*speed.Data;


subplot(1,2,1)
plot(Torque.Time,P_dq/1000)
title('Active power [kW]')


E_dq=cumtrapz(Torque.Time,P_dq);

subplot(1,2,2)
plot(Torque.Time,E_dq)
title('Active Energy [J]')

%%
figure()
plot(my_I123.Time, my_I123.Data);
%%
figure()
plot(stator_currents.Time,stator_currents.Data)
title('stator currents [A]')
%%
close all
figure()
plot(rotor_currents.Time,rotor_currents.Data)
title('rotor currents [A]')
%%
figure()
plot(my_Ir123.Time, my_Ir123.Data);
