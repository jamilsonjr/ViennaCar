%% Vienna Car part2
%% Induction Motor Model Characteristics 
% For the first task with the already known constants it's possible to calculate the values for the remaining incognites of the indution motor equivelent circuit.
%%
% Starting with the stator current:
%% 
% $$I_s = { V_S \over  Z_S + ({R_R \over S} + j\omega L_R) //( j \omega
% L_{Ms})}$$
%%
% With respects to the $V_{Ms}$ we obtain the following expression with KVL:
%%
% $$ V_{Ms} = V_S - I_S * Z_S $$
%%
% Then we procede to calculate $I_R$ with KCL:
%%
% $$ I_R = I_S - \frac{ V_{Ms} }{ \frac{R_R}{S} + j*\omega * L_R } $$
%%
% Knowing now the Rotor current we can find the Phase Power on each phase with:
%%
% $$ P = I_R^2 * \frac{ R_R * (S-1) }{ S } $$
%%
% The Mechanical Power is the sum of the Power in the three fases:
%%
% $$ P_{Mec} = 3 * P $$
%%
% Finally, the Torque is given by:
%%
% $$ T = \frac{ P_{Mec} }{ \Omega_S } $$
% 
%%
clear
clc
close all
frequency = 0:0.76:76;
stator_voltage = 0:0.76:76;
slip = 0:0.01:1;
slip = fliplr(slip);
%
torque_sim = sim('induction_motor');
figure();
plot(frequency, abs(torque_sim.torque), 'LineWidth', 1);
set( gca, 'FontSize', 11);
grid on;
title('Torque');
xlabel('Frequancy $[Hz]$','Interpreter', 'latex');
ylabel('Torque [$Nm$]','Interpreter', 'latex');
