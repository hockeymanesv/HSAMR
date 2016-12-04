% Skript zur Darstellung von Wurzelortskurven
% Autor: Dipl. Ing. Matthias Schäfer

clear all; close all; clc;
set(0, 'DefaultLineLineWidth',1);
Tn=1; Tv=1;

P=tf([1],[1 0 0]); % hier Übertragungsfunktion der Regelstrecke eintragen
K=[ tf(1,1);%P-Regler
    tf([Tv 1],1);%PD-Regler
    tf([1 1/Tn],[1 0]);%PI
    tf([Tv 1 1/Tn],[1 0]);%PID
    ];

    figure('Name','Wurzelortskurven für linearisiertes Modell Geradeausfahrt','NumberTitle','off');
    subplot('Position',[0.05 0.6 0.3 0.4]),rlocus(P*K(1));
    title('P-Regler');
    
    subplot('Position',[0.5 0.6 0.3 0.4]),rlocus(P*K(2));
    title('PD-Regler');
    
    subplot('Position',[0.05 0.1 0.3 0.4]),rlocus(P*K(3));
    title('PI-Regler');
    
    subplot('Position',[0.5 0.1 0.3 0.4]),rlocus(P*K(4));
    title('PID-Regler');
    

print -depsc WurzelOrtskurvenHighGain.eps
