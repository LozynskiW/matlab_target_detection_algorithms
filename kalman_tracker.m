function [Xk,Yk,x_corr,Pk_corr]=kalman_tracker(x,y,vx,vy,T,licznik,x_previous,Pk_previous)
%{
F-macierz dynamiki systemu (wi¹¿¹ca stan poprzedni z aktualnym)
B-wymuszenie stanu (sterowanie)
H-macierz wi¹¿¹ca pomiar ze stanem (wyjœcie filtru)

x_estim - prognozowane wartoœci stanu a priori
x_corr - estymowany stan a posteriori uwzglêdniaj¹cy pomiar
x_previous - optymalne oszacowanie wartoœci a posteriori wykonane w
poprzednim kroku

Pk_estim - prognozowana macierz kowariancji a priori
Pk_corr - macierz kowariancji a posteriori
Pk_previous - optymalne oszacowanie wartoœci a posteriori wykonane w
poprzednim kroku

Kk - wzmocnienie Kalmana
zk - pomiar
I - macierz jednostkowa
Q/q-wariancja procesu
R/r-wariancja zak³óceñ
S - pozycja
V - prêdkoœæ
Tp - okres próbkowania
k-czas
%}
%Zmienne
%X=[x x_dot y y_dot].'; %wektor stanu
%T=10*T;
F=[1 T 0 0;
   0 1 0 0;
   0 0 1 T;
   0 0 0 1]; %macierz dynamiki systemu
H=[1 0 0 0;
   0 0 0 0;
   0 0 1 0;
   0 0 0 0];
B=[T 0;
   0 0;
   0 T;
   0 0]; %macierz wejœciowych danych deterministycznych
Bx=0.1;
u=[vx vy].';
R=[Bx^2 0 0 0;
   0 Bx^2 0 0;
   0 0 0 0;
   0 0 0 0]; %macierz kowariancji szumów pomiarowych
Q=[1 0 0 0;
   0 0 0 0;
   0 0 1 0;
   0 0 0 0]; %macierz pomiarów

if x*y==0
    Pk_previous=Q;
    x_previous=[0 0 0 0].';
end
    zk=[x vx y vy].';
    %Przewidywanie
x_estim=F*x_previous+B*u; %przewidywanie stanu w przysz³oœci
Pk_estim=F*Pk_previous*F.'+Q; %przewidywanie macierzy kowariancji w przysz³oœci

%Wyniki
Xk=x_estim(1,1);
Yk=x_estim(3,1);

%Korekcja
Kk=Pk_estim*(H.').*inv(H*Pk_estim*(H.')+R+eye(4)*1e-3); %wzmocnienie filtru kalmana
x_corr=x_estim+Kk*(zk-H*x_estim); %aktualizacja przewidywañ wzglêdem pomiarów
Pk_corr=(1-Kk*H).*Pk_estim; %aktualizacja macierzy kowariancji b³êdu

end