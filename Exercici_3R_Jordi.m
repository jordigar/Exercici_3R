% Cridem la funcio que sera qui cridara les altres funcions i
% plotejara el moviment
ploteja()

% Definim les funcions que necessitem i que ja hem utilitzat en els exercicis
% anteriors

function J = jacobian(J1,J2,J3)

s1 = [J1(2); -J1(1); 1];
s2 = [J2(2); -J2(1); 1];
s3 = [J3(2); -J3(1); 1];
J = [s1 s2 s3]  

end

function G = inversekin(J, T)

G = inv(J) * T

end

function [a1, a2, a3] = inversepos (P, alfa)

% Valors de l'exercici 3R
l1 = 0.62;
l2 = 0.57;
g1 = 0.1; 
g2 = 0.2;
g3 = 0.3;

x = P(1) - g3;
y = P(2) + g2;

d_inv = 2*l1*l2;
f_inv = (x - g1*cos(alfa))^2 + (y - g1*sin(alfa))^2 - l1^2 - l2^2;

a2 = acos(f_inv/d_inv);

A = l1 + l2*cos(a2);
B = l2*sin(a2);
E = x - g1*cos(alfa);
F = y - g1*sin(alfa);

AA = [-B, A; A, B]; 
EE = [E; F]; 
X=AA\EE;
cos1=X(2);

a1 = - acos(cos1);

a3 = alfa - a1 - a2;

end

% Funció que fa les crides i ploteja el moviment
function ploteja()
% Valors de l'exercici 3R
Start = [0.9; -0.2];
Goal = [0.9; -0.7];
alfa = 0; 
v = -0.1; % definim la velocitat
T = [0; v; 0]; % nomes tenim velocitat en y
t = 0.01; % definim el temps

l1 = 0.62;  
l2 = 0.57 ; 
g1 = 0.1; 
g2 = 0.2; 
g3 = 0.3; 

% Calculem el total d'iteracions i inicialitzem les matrius
n = round((Goal(2)-Start(2))/(v*t));

j1 = []; % joint 1
j2 = []; % joint 2
j3 = []; % joint 3
a1 = []; % angle joint 1
a2 = []; % angle joint 2
a3 = []; % angle joint 3 
G = []; % velocitats de rotacio de les joints
P = []; % pose end effector

% Cridem la funcio inversepos() per saber la posicio dels joints
[a1(1), a2(1), a3(1)] = inversepos (Start, alfa);
 
j1(:,1) = [0; 0];             
j2(:,1) = j1(:, 1) + l1*[cos(a1(1)); sin(a1(1))];
j3(:,1) = j2(:, 1) + l2*[cos(a1(1)+a2(1)); sin(a1(1)+a2(1))];
P(:,1) = j3(:, 1) + [g1+g3;-g2];

% Primer plotejem la posicio inicial del robot 3R
figure(1)
hold on
title('Moviment 3R')
xlabel('x')
ylabel('y')

% els links son rectes
plot([j1(1,1),j2(1,1)],[j1(2,1),j2(2,1)],'-k')
plot([j2(1,1),j3(1,1)],[j2(2,1),j3(2,1)],'-k')
plot([j3(1,1),j3(1,1)+g1],[j3(2,1),j3(2,1)],'-k')
plot([j3(1,1)+g1,j3(1,1)+g1],[j3(2,1),j3(2,1)-g2],'-k')
plot([j3(1,1)+g1,j3(1,1)+g1+g3],[j3(2,1)-g2,j3(2,1)-g2],'-k')

% les joints son cercles
plot(j1(1,1),j1(2,1),'ok','MarkerFaceColor','k')
plot(j2(1,1),j2(2,1),'ok','MarkerFaceColor','k')
plot(j3(1,1),j3(2,1),'ok','MarkerFaceColor','k')

axis([-0.2 1 -1 0.2])
grid on
hold off
 
% Fem totes les iteracion i plotejem
for k=1:n
    % Cridem la funcio del jacobia i la inversekin per saber les velositats
    % de les joints
    J = jacobian( j1(:,end), j2(:,end), j3(:,end));
    G(:,end+1) = inversekin(J, T);

    % Sabent les velocitats trobem els angles i les posicions
    a1(end+1) = a1(end) + t*G(1,end);
    a2(end+1) = a2(end) + t*G(2,end);
    a3(end+1) = a3(end) + t*G(3,end);
    j1(:,end+1) = j1(:,end); 
    j2(:,end+1) = j1(:,end) + l1*[cos(a1(end)); sin(a1(end))];
    j3(:,end+1) = j2(:,end) + l2*[cos(a1(end)+a2(end)); sin(a1(end)+a2(end))];
    
    P(:,end+1) = j3(:, end) + [g1+g3;-g2];

    % Tornem a fer el plot de la posicio
    figure(1)
    axis([-0.2 1 -1 0.2])

    % Els links son rectes
    plot([j1(1,end),j2(1,end)],[j1(2,end),j2(2,end)],'-k')
    hold on
    title('Moviment 3R')
    xlabel('x')
    ylabel('y')
    plot([j2(1,end),j3(1,end)],[j2(2,end),j3(2,end)],'-k')
    plot([j3(1,end),j3(1,end)+g1],[j3(2,end),j3(2,end)],'-k')
    plot([j3(1,end)+g1,j3(1,end)+g1],[j3(2,end),j3(2,end)-g2],'-k')
    plot([j3(1,end)+g1,j3(1,end)+g1+g3],[j3(2,end)-g2,j3(2,end)-g2],'-k')

    % Les joints son cercles
    plot(j1(1,end),j1(2,end),'ok','MarkerFaceColor','k')
    plot(j2(1,end),j2(2,end),'ok','MarkerFaceColor','k')
    plot(j3(1,end),j3(2,end),'ok','MarkerFaceColor','k')
    
    axis([-0.2 1 -1 0.2])
    grid on
    hold off

end

% Finalment plotejem les velocitats de les joints
time = linspace(t,n*t,n);
figure(2)
hold on
grid on
title(['Velocitats joints'])
xlabel(['temps'])
ylabel(['velocitat'])
axis([0 n*t -1.5 1.5])
plot(time, G(1,:),'k')
plot(time, G(2,:),'b')
plot(time, G(3,:),'r')
hold off
legend('joint_1','joint_2','joint_3')

end