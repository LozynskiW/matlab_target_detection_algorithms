clc
clear all
%{
A - macierz stanu
B - macierz wejœcia(sterownia)
C - macierz wyjœcia
P - macierz kowariancji
K - macierz wzmocnienia Kalmana
Q - macierz kowariancji procesu
R - macierz kowariancji pomiaru
v - szum pomiaru
w - szum procesu

x_est_priori - estymowana wartoœæ x
x_est_post
%}

%Dane wstêpne
dt=1; %czas próbkowania
S_frame=10; %klatka startowa
u=5; %przyœpieszenie
%Q= [CM_idx(S_frame,1); CM_idx(S_frame,2); 0; 0];
%Q_estimate=Q;
Accel_noise_mag=10;
tkn_x=1; %szum pomiaru po³o¿enia w x
tkn_y=1; %szum pomiaru po³o¿enia w y
Ez=[tkn_x 0; 
    0 tkn_y]; %macierz 
Ex=[dt^4/4 0 dt^3/2 0;
    0 dt^4/4 0 dt^3/2;
    dt^3/2 0 dt^2 0;
    0 dt^3/2 0 dt^2].*Accel_noise_mag^2;
P=Ex; %pocz¹tkowa za³o¿ona wartoœæ macierzy kowariancji b³êdu
T=0.04;
A=[1 T 0 0;
   0 1 0 0;
   0 0 1 T;
   0 0 0 1]; %macierz wi¹¿¹ca stan poprzedni z aktualnym
B=[T 0;
   0 0;
   0 T;
   0 0];

C=[1 0 1 0];


Bx=0.1;

R=[Bx^2 0 0 0;
   0 Bx^2 0 0;
   0 0 0 0;
   0 0 0 0]; %macierz kowariancji szumów pomiarowych
Q=[1 0 0 0;
   0 0 0 0;
   0 0 1 0;
   0 0 0 0]; %macierz pomiarów
model=ss(A,[B B],C,0,-1);
Q=1;
R=1;
[kalmf,L,P,M]=kalman(model,Q,R);

function [residual,xhatOut]=kalman(meas,deltat)
%initialization
persistent P
persistent xhat
if isempty (P)
    xhat = [0.001; 0.01; 0.001; 400];
    P=zeros(4);
end

% 1.Compute Phi, Q and R
Phi = [1 deltat 0 0; 0 1 0 0; 0 0 1 deltat; 0 0 0 1];
Q = diag({0 0.005 0 0.005});
R = diag({300^2 0.001^2});

% 2.Propagate the covariance matrix:
P=phi*P*Phi'Q;
% 3. Propagate the track estimate:
xhat = Phi*xhat;
% 4 a). Compute observation estimates:
Rangehat = sqrt(xhat(1)^2*xhat(3)^2);
Bearinghat = atan2(xhat(3),xhat(1));

% 4 b). Compute observation vector y and linearized measurement matrix M:
yhat = [Rangehat;
        Bearinghat];
M = [cos(Bearinghat)           0 sin(Bearinghat)          0
     -sin(Bearinghat)/Rangehat 0 cos(Bearinghat)/Rangehat 0];
% 4 c). Compute residual (Estimation Error)
residual = meas - yhat;

% 5. Compute Kalman Gain:
W = P*M'*inv(M*P*M'+R); 

% 6.Update estimate
xhat = xhat + W*residual;

% 7. Update Covariance Matrix
P = (eye(4)-W*M)*P*(eye(4)-W*M)' + W*R*W';

xhatOut = xhat;
end

