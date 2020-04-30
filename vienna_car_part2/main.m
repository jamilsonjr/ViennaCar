%% Vienna Car part2
%% Induction Motor Model Characteristics 
% For the first task with the already known constants it's possible to calculate the values for the remaining incognites of the indution motor equivelent circuit.
%%
% Starting with the stator current:
%% 
% $I_s = { V_S \over  Z_S + ({R_R \over S} + j\omega L_R) // ( j \omega L_{Ms})}$
%%
% With respects to the $V_M$ we obtain the following expression with KVL:
% \begin{equation}
%    V_{Ms} = V_S - I_S * Z_S
% \end{equation}
%
% -> Then we procede to calculate %I_R% with KCL:
%
% \begin{equation}
%    I_R = I_S - \frac{ V_{Ms} }{ \frac{R_R}{S} + j*\omega * L_R }
% \end{equation}
%
% -> Knowing now the Rotor current we can find the Phase Power on each phase with:
%
% \begin{equation}
%    P = I_R^2 * \frac{ R_R * (S-1) }{ S }
% \end{equation}
%
% -> The Mechanical Power is the sum of the Power in the three fases:
%
% \begin{equation}
%    P_{Mec} = 3 * P
% \end{equation}
%
% -> Finally, the Torque is given by:
%
% \begin{equation}
%    T = \frac{ P_{Mec} }{ \Omega_S }
% \end{equation}
% 