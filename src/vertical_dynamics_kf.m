%% =========================================================================
% PROJEKT: Zustandsschätzung der Fahrzeug-Vertikaldynamik (7-DOF Modell)
% TUTORIUM: Zustands- und Parameterschätzung am Beispiel der KFZ-Dynamik
%
% BESCHREIBUNG: 
% Dieses Skript führt eine Simulation der Vertikaldynamik eines Fahrzeugs 
% mit 7 Freiheitsgraden durch. Es vergleicht die simulierten "wahren" Werte 
% mit den Schätzwerten eines Kalman-Filters (KF) unter Berücksichtigung 
% von System- und Messrauschen.
%
% ENTHALTENE SCHRITTE:
% 1. Definition der Systemparameter (Masse, Steifigkeit, Dämpfung)
% 2. Generierung der Straßenanregung
% 3. Konfiguration des Kalman-Filters (Q- und R-Matrizen)
% 4. Iterative Simulation und statistische Auswertung (MSE)
% 5. Visualisierung der Schätzgüte
% =========================================================================

clc; clear; close all;

%% *Systemparameter vorgeben*
% Massen- und Trägheitsmomente
m1 = 50;        % Ungefederte Masse Rad 1 [kg]
m2 = 50;        % Ungefederte Masse Rad 2 [kg]
m3 = 50;        % Ungefederte Masse Rad 3 [kg]
m4 = 50;        % Ungefederte Masse Rad 4 [kg]      
m5 = 1200;      % Gefederte Masse (Aufbau) [kg]
Jphi = 500;     % Trägheitsmoment um die Längsachse (Wanken) [kgm^2]
Jtheta = 2000;  % Trägheitsmoment um die Querachse (Nicken) [kgm^2]

% Federsteifigkeiten
kR = 100000;    % Reifensteifigkeit [N/m]
kFv = 2500;     % Federsteifigkeit vorne [N/m]
kFh = 2200;     % Federsteifigkeit hinten [N/m]

% Dämpfungskonstanten
dR = 250;       % Reifendämpfung [Ns/m]
dFv = 1500;     % Dämpfung vorne [Ns/m]
dFh = 2000;     % Dämpfung hinten [Ns/m]

% Geometrische Abmessungen
b = 1.4;        % Halbe Fahrzeugbreite [m]
lv = 1.9;       % Abstand Schwerpunkt zu Vorderachse [m]
lh = 1.8;       % Abstand Schwerpunkt zu Hinterachse [m]

% Anfangsbedingungen (Positionen und Geschwindigkeiten)
z1_0 = 0;       dz1_0 = 0;
z2_0 = 0;       dz2_0 = 0;
z3_0 = 0;       dz3_0 = 0;
z4_0 = 0;       dz4_0 = 0;
z5_0 = 0;       dz5_0 = 0;
Phi_0 = 0;      dPhi_0 = 0;
Theta_0 = 0;    dTheta_0 = 0;

%% *Dauer der Simulation, Abtastrate*
T_sim = 10;     % Gesamtsimulationszeit [s]
T_abt = 0.01;   % Abtastzeitschritt [s]
% t = nan(T_sim/T_abt,1);
% t(:,1) = T_abt:T_abt:T_sim;
t = (T_abt:T_abt:T_sim)';   % Zeitvektor [s]

%% *Kraftanregung erzeugen (Straßenprofil)*
udach = 0.05;   % Amplitude der Anregung [m]
fE = 5;         % Erregerfrequenz [Hz]
Omega = 2*pi*fE;

% Vertikale Auslenkung der vier Räder
u1 = udach * cos(Omega*t);
u2 = 3*udach * cos(Omega*t);
u3 = -2*udach * sin(Omega*t);
u4 = -udach * sin(Omega*t);

% Vertikale Geschwindigkeiten der Anregung
du1 = -Omega * udach * sin(Omega*t);
du2 = -3*Omega * udach * sin(Omega*t);
du3 = -2*Omega * udach * cos(Omega*t);
du4 = -Omega * udach * cos(Omega*t);

%% *Rauschen des Systems vorgeben*
% Standardabweichungen für Prozessrauschen
sigma_z         =  1.5;
sigma_dz        =  1.1;
sigma_phi       =  1.2;
sigma_dphi      =  1.6;
sigma_theta     =  1.7;
sigma_dtheta    =  1.3;

% Standardabweichungen für Messrauschen
sigma_mz        =  0.1;
sigma_mphi      =  0.2;
sigma_mtheta    =  0.1;

%% *Einstellungen des Kalman Filters*
% Kovarianzmatrix des Prozessrauschens Q
Q = diag([sigma_z^2,sigma_z^2,sigma_z^2,sigma_z^2,sigma_z^2,sigma_phi^2,sigma_theta^2,...
   sigma_dz^2,sigma_dz^2,sigma_dz^2,sigma_dz^2,sigma_dz^2,sigma_dphi^2,sigma_dtheta^2 ])*T_abt^2;

% Kovarianzmatrix des Messrauschens R
R = diag([sigma_mz^2,sigma_mz^2,sigma_mz^2,sigma_mz^2,sigma_mz^2,sigma_mphi^2,sigma_mtheta^2]);

% Initiale Fehlerkovarianz und Zustandsvektor
P_0 = Q;
x_0 = [z1_0;z2_0;z3_0;z4_0;z5_0;Phi_0;Theta_0;dz1_0;dz2_0;dz3_0;dz4_0;dz5_0;dPhi_0;dTheta_0];

%% *Simulation und Bewertung*
N = 10; % Anzahl der Simulationsläufe für die statistische Auswertung
 for i = 1:N
    seed=randi(1000,3,1);  % Zufallssaat für stochastische Prozesse in Simulink
    sim('vertical_dynamics_vehicle.slx',T_sim);
    
    % Berechnung der Schätzgüte mittels Mean Squared Error (MSE)
    % Vergleich: Messung/Simulation vs. Realität und Schätzung vs. Realität
    fit(1).z5(i)    = goodnessOfFit(z5_real,z5_sim,'MSE');
    fit(2).z5(i)    = goodnessOfFit(z5_real,z5_est,'MSE');
    fit(1).phi(i)   = goodnessOfFit(Phi_real,Phi_sim,'MSE');
    fit(2).phi(i)   = goodnessOfFit(Phi_real,Phi_est,'MSE');
    fit(1).theta(i) = goodnessOfFit(Theta_real,Theta_sim,'MSE');
    fit(2).theta(i) = goodnessOfFit(Theta_real,Theta_est,'MSE');
    fit(1).dz5(i)   = goodnessOfFit(dz5_real,dz5_sim,'MSE');
    fit(2).dz5(i)   = goodnessOfFit(dz5_real,dz5_est,'MSE');
    fit(1).dphi(i)  = goodnessOfFit(dPhi_real,dPhi_sim,'MSE');
    fit(2).dphi(i)  = goodnessOfFit(dPhi_real,dPhi_est,'MSE');
    fit(1).dtheta(i)= goodnessOfFit(dTheta_real,dTheta_sim,'MSE');
    fit(2).dtheta(i)= goodnessOfFit(dTheta_real,dTheta_est,'MSE');
    
    % Gesamte Fehler für Position, Wanken und Nicken
    fit(1).gesZ(i)  = goodnessOfFit([z5_real, dz5_real],[z5_sim, dz5_sim],'MSE');
    fit(2).gesZ(i)  = goodnessOfFit([z5_real, dz5_real],[z5_est, dz5_est],'MSE');
    fit(1).gesP(i)  = goodnessOfFit([Phi_real, dPhi_real],[Phi_sim, dPhi_sim],'MSE');
    fit(2).gesP(i)  = goodnessOfFit([Phi_real, dPhi_real],[Phi_est, dPhi_est],'MSE');
    fit(1).gesT(i)  = goodnessOfFit([Theta_real, dTheta_real],[Theta_sim, dTheta_sim],'MSE');
    fit(2).gesT(i)  = goodnessOfFit([Theta_real, dTheta_real],[Theta_est, dTheta_est],'MSE');
 end

% Mittelwertbildung der Fehler über alle Simulationsläufe (reine Simulation)
sim.z5      = mean(fit(1).z5);
sim.dz5     = mean(fit(1).dz5);
sim.gesZ    = mean(fit(1).gesZ);
sim.Phi     = mean(fit(1).phi);
sim.dPhi    = mean(fit(1).dphi);
sim.gesP    = mean(fit(1).gesP);
sim.Theta   = mean(fit(1).theta);
sim.dTheta  = mean(fit(1).dtheta);
sim.gesT    = mean(fit(1).gesT);

% Mittelwertbildung der Fehler über alle Simulationsläufe (Kalman-Filter)
kf.z5       = mean(fit(2).z5);
kf.dz5      = mean(fit(2).dz5);
kf.gesZ     = mean(fit(2).gesZ);
kf.Phi      = mean(fit(2).phi);
kf.dPhi     = mean(fit(2).dphi);
kf.gesP     = mean(fit(2).gesP);
kf.Theta    = mean(fit(2).theta);
kf.dTheta   = mean(fit(2).dtheta);
kf.gesT     = mean(fit(2).gesT);

%% *Durchschnittliche Fehler plotten*
names = fieldnames(sim);
for i = 1:length(names)
    bardata(1,i) = sim.(names{i});
    bardata(2,i) = kf.(names{i});    
end
cdata = categorical({'Sim','KF'});
cdata = reordercats(cdata, {'Sim','KF'});

% Balkendiagramm zum Vergleich der Fehler
bar(cdata, bardata)
legend({'z5','dz5','gesZ5','phi','dphi','gesPhi','theta','dtheta','gesTheta'},...
       'Location','northwest','NumColumns',3)
ylabel('Durchschnittliche Fehler (MSE)')
grid on;

%% *Ausgabe der Ergebnisse im Command Window* 
disp('--- Durchschnittliche Fehler der Simulation ---')
disp(sim)
disp('--- Durchschnittliche Fehler des Kalman-Filters ---')
disp(kf)