clc
clear all
close all

g=9.81;

%% motor dq
%--------------------------------------------------------------------------
% Motor Parameters
%--------------------------------------------------------------------------
% rated values
Vb = 76;
fb = 76;
wb = 2*pi*fb;
Pp = 2;
Ib = 160;
Imb = 120/sqrt(2); % no load current

% Equivalent Circuit Parameters
pp=2;
Rs = 8.56e-3;
ls = 0.06292e-3;
LM = 1.0122e-3;
Rr = 2*5.10e-3;
lr = 0.06709e-3;

%% ----- COMPLETE THE FOLLOWING -----
% stator and rotor self inductances for dq model
LS = ls + LM;
LR = lr + LM;
% dq model Matrices
LMatrix = [LS 0 LM 0; 0 LS 0 LM; LM 0 LR 0; 0 LM 0 LR];
RMatrix = [Rs 0 0 0; 0 Rs 0 0; 0 0 Rr 0; 0 0 0 Rr];
RLInv = RMatrix/(LMatrix);

%--------------------------------------------------------------------------
% FOC
%--------------------------------------------------------------------------
ratedFluxPeak = sqrt(2)*Imb*LM;
maxTorque = 65;
maxID = sqrt(2)*Imb;
maxIQ = sqrt(2*Ib^2 - Imb^2);

%% sim
% select the matlab version you want!
sim('test_motor_dq_2017b.slx')
% sim('test_motor_dq_2016b.slx')
% sim('test_motor_dq_2015b.slx')
% sim('test_motor_dq_2015a.slx')
% sim('test_motor_dq_2014a.slx')
%% plot
figure(1)
plot(speed_dq_model.Time,speed_dq_model.Data*60/(2*pi))
hold on
plot(speed_eq_circuit_model.Time,speed_eq_circuit_model.Data*60/(2*pi))
plot(ref_speed.Time,ref_speed.Data)
title('Speed [rpm]')
legend('dq-model','Equivalent circuit model','reference speed')

figure(2)
plot(Torque_dq_model.Time,Torque_dq_model.Data)
hold on
plot(Torque_eq_circuit_model.Time,Torque_eq_circuit_model.Data)
title('Torque [Nm]')
legend('dq-model','Equivalent circuit model')

figure(3)
plot(stator_currents_dq_model.Time,stator_currents_dq_model.Data)
hold on
plot(stator_currents_eq_circuit_model.Time,stator_currents_eq_circuit_model.Data)
title('currents [A]')
legend('dq-model','Equivalent circuit model')

figure(4)
P_dq=Torque_dq_model.Data.*speed_dq_model.Data;
P_eq=Torque_eq_circuit_model.Data.*speed_eq_circuit_model.Data;

subplot(1,2,1)
plot(Torque_dq_model.Time,P_dq/1000)
hold on
plot(Torque_eq_circuit_model.Time,P_eq/1000)
title('Active power [kW]')
legend('dq-model','Equivalent circuit model')

E_dq=cumtrapz(Torque_dq_model.Time,P_dq);
E_eq=cumtrapz(Torque_eq_circuit_model.Time,P_eq);

subplot(1,2,2)
plot(Torque_dq_model.Time,E_dq)
hold on
plot(Torque_eq_circuit_model.Time,E_eq)
title('Active Energy [J]')
legend('dq-model','Equivalent circuit model')
%%
% close all 
% 
% figure()
% plot(stator_currents_dq_model_junior.Time,stator_currents_dq_model_junior.Data)
% title('currents in dq [A]')
% legend('dq-model')
% 
% figure()
% plot(stator_currents_dq_model_junior_d.Time,stator_currents_dq_model_junior_d.Data)
% title('currents in d [A]')
% legend('dq-model')
% 
% figure()
% plot(stator_currents_dq_model_junior_q.Time,stator_currents_dq_model_junior_q.Data)
% title('currents in q [A]')
% legend('dq-model')

save('sim_professor_part4');