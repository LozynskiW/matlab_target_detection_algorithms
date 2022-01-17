clc
clear all

%% Zmienne

%{
Kl - nakazane przyœpieszenie wzd³u¿ne
K_teta - nakazane przyœpieszenie wokó³ osi poprzecznej (pochylenie)
K_fi - nakazane przyœpieszenie wokó³ osi pionowej (odchylenie)

al - przyœpieszenie wzd³u¿ne samolotu
a_teta - przyœpieszenie wokó³ osi poprzecznej (pochylenie)
a_fi - przyœpieszenie wokó³ osi pionowej (odchylenie)

tau - sta³a czasowa
v - prêdkoœæ celu
teta - k¹t pochylenia
fi - k¹t odchylenia
gamma - kat przechylenia (obrót wokó³ osi pod³u¿nej

x,y,z - wspó³rzêdne œrodka masy samolotu

K = [ x y z teta fi gamma] - macierz stanu
%}

%% Wyprowadzenie równañ do symulacji
%{
syms al(t) afi(t) ateta(t) Kl Kteta Kfi tau vl(t) teta(t) fi(t) gamma(t) x(t) y(t) z(t) g vl0 teta0 fi0 x0 y0 z0 al0 ateta0 afi0 tetawyl fiwyl vlwyl xwyl

% V
ode=diff(al(t),t)+(1/tau)*al(t)==(1/tau)*Kl;
cond = al(0) == al0;
al=dsolve(ode,cond)

ode2=diff(vl(t),t)==al;
cond2 = vl(0)==vl0;
v=dsolve(ode2,cond2)

%teta
ode3=diff(ateta(t),t)+(1/tau)*ateta(t)==(1/tau)*Kteta;
cond3 = ateta(0) == ateta0;
ateta=dsolve(ode3,cond3)

%{
ode4=diff(teta(t),t)==ateta/v;
cond4 = teta(0)==teta0;
teta1=dsolve(ode4,cond4)
teta2=int((ateta0 - Kteta + Kteta*exp(xwyl/tau))/(Kl*tau - al0*tau + vl0*exp(xwyl/tau) - Kl*tau*exp(xwyl/tau) + Kl*xwyl*exp(xwyl/tau) + al0*tau*exp(xwyl/tau)));
teta=teta0+teta2
%}
%fi
ode5=diff(afi(t),t)+(1/tau)*afi(t)==(1/tau)*Kfi;
cond5 = afi(0)==afi0;
afi=dsolve(ode5,cond5)

ode6=diff(fi(t),t)==afi/v;
cond6 = fi(0)==fi0;
fi1=dsolve(ode6,cond6);
fi2=int((afi0 - Kfi + Kfi*exp(xwyl/tau))/(Kl*tau - al0*tau + vl0*exp(xwyl/tau) - Kl*tau*exp(xwyl/tau) + Kl*xwyl*exp(xwyl/tau) + al0*tau*exp(xwyl/tau)));
fi=fi0+fi2
%gamma
gamma(t)=atan(afi/g)

%X
ode7=diff(x(t),t)==v*cos(tetawyl)*sin(fiwyl);
cond7 = x(0)==x0;
x=dsolve(ode7,cond7)

%Y
ode8=diff(y(t),t)==v*sin(tetawyl);
cond8 = y(0)==y0;
y=dsolve(ode8,cond8)

ode9=diff(z(t),t)==v*cos(tetawyl)*cos(fiwyl);
cond9 = z(0)==z0;
z=dsolve(ode9,cond9)

ode4=diff(teta(t),t)==ateta/v;
cond4 = teta(0)==teta0;
teta=dsolve(ode4,cond4)
%}

%% Symulacja
fps=50;
tau=1/fps;
Kl=50;
Kfi=20;
Kteta=30;
al0=20;
ateta0=10;
afi0=10;

vl0=300;
teta0=0;
fi0=30;

accel_l=0.5;
accel_teta=0.5;
accel_fi=0.5;

vl_max=400;
vteta_max=300;
vfi_max=300;


x0=15000;
y0=5000;
z0=5000;

g=9.81;
M=[];

dt=0.1;
tmax=60;
time=0:dt:tmax;

v=[];
A=[];
K=al0;
Kt=ateta0;
Kf=afi0;
Klm=[];
Ktetam=[];
Kfim=[];

v_l_kontrol=0;
v_teta_kontrol=0;
v_fi_kontrol=0;

vteta0=0;
vfi0=0;

%{
for t=0:dt:tmax
    v_teta_kontrol=t*Kt+v_teta_kontrol;
    if v_teta_kontrol>vteta_max*0.9
        Kt=0;
        Ktetam=[Ktetam Kt];
    else
        Ktetam=[Ktetam Kt];    
    end
end

for t=0:dt:tmax
    v_fi_kontrol=t*Kf+v_fi_kontrol;
    if v_fi_kontrol>vfi_max*0.9
        Kf=0;
        Kfi=[Kfim Kf];
    else
        Kfim=[Kfim Kf];    
    end
end
%}

licznik=1;
for t=0:dt:tmax
   
al = Kl - exp(-t/tau)*(Kl - al0);
vl = vl0 + Kl*t - Kl*tau + al0*tau + tau*exp(-t/tau)*(Kl - al0);
ateta = Kteta - exp(-t/tau)*(Kteta - ateta0);
%teta = teta0 + int((ateta0 - Kteta + Kteta*exp(x/tau))/(Kl*tau - al0*tau + vl0*exp(x/tau) - Kl*tau*exp(x/tau) + Kl*x*exp(x/tau) + al0*tau*exp(x/tau)), x, 0, t, 'IgnoreSpecialCases', true, 'IgnoreAnalyticConstraints', true);
afi = Kfi - exp(-t/tau)*(Kfi - afi0);
%fi = fi0 + int((afi0 - Kfi + Kfi*exp(xwyl/tau))/(Kl*tau - al0*tau + vl0*exp(xwyl/tau) - Kl*tau*exp(xwyl/tau) + Kl*xwyl*exp(xwyl/tau) + al0*tau*exp(xwyl/tau)), xwyl);
 
teta= teta0 - (tau*(Kteta - ateta0))/vl + (Kteta*t)/vl + (tau*exp(-t/tau)*(Kteta - ateta0))/vl;
teta=teta*pi/180;
fi=fi0 - (tau*(Kfi - afi0))/vl + (Kfi*t)/vl + (tau*exp(-t/tau)*(Kfi - afi0))/vl;
fi=fi*pi/180;
gamma=atan((Kfi - exp(-t/tau)*(Kfi - afi0))/g);
x=x0 - exp(-t/tau)*(Kl*tau^2*cos(teta)*sin(fi) - al0*tau^2*cos(teta)*sin(fi)) + t*(vl0*cos(teta)*sin(fi) - Kl*tau*cos(teta)*sin(fi) + al0*tau*cos(teta)*sin(fi)) + (Kl*t^2*cos(teta)*sin(fi))/2 + Kl*tau^2*cos(teta)*sin(fi) - al0*tau^2*cos(teta)*sin(fi);
y=y0 + t*(vl0*sin(teta) - Kl*tau*sin(teta) + al0*tau*sin(teta)) - exp(-t/tau)*(Kl*tau^2*sin(teta) - al0*tau^2*sin(teta)) + (Kl*t^2*sin(teta))/2 + Kl*tau^2*sin(teta) - al0*tau^2*sin(teta);
z=z0 - exp(-t/tau)*(Kl*tau^2*cos(fi)*cos(teta) - al0*tau^2*cos(fi)*cos(teta)) + t*(vl0*cos(fi)*cos(teta) - Kl*tau*cos(fi)*cos(teta) + al0*tau*cos(fi)*cos(teta)) + (Kl*t^2*cos(fi)*cos(teta))/2 + Kl*tau^2*cos(fi)*cos(teta) - al0*tau^2*cos(fi)*cos(teta);
M=[M; x y z teta fi gamma];


A=[A; al ateta afi];
v=[v vl];
licznik=licznik+1;
if t==0
    v_l_kontrol=vl0;
    v_teta_kontrol=vteta0;
    v_fi_kontrol=vfi0;
end
v_l_kontrol=dt*al+v_l_kontrol;
    if v_l_kontrol>vl_max*0.9
        Kl=0;
        Klm=[Klm Kl];
    else
        Klm=[Klm Kl];    
    end   
    
    v_teta_kontrol=dt*ateta+v_teta_kontrol;
    if v_teta_kontrol>vteta_max*0.9
        Kteta=0;
        Ktetam=[Ktetam Kteta];
    else
        Ktetam=[Ktetam Kteta];    
    end
    
    v_fi_kontrol=dt*afi+v_fi_kontrol;
    if v_fi_kontrol>vfi_max*0.9
        Kfi=0;
        Kfim=[Kfim Kfi];
    else
        Kfim=[Kfim Kfi];    
    end
end

figure(1)
subplot(3,1,1)
plot(time,M(:,1))
title('x')
subplot(3,1,2)
plot(time,M(:,2))
title('y')
subplot(3,1,3)
plot(time,M(:,3))
title('z')

figure(2)
plot3(M(:,1),M(:,2),M(:,3))
grid on
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')

figure(3)
subplot(4,1,1)
plot(time,v)
title('v')
subplot(4,1,2)
plot(time,A(:,1))
title('al')
subplot(4,1,3)
plot(time,A(:,2))
title('ateta')
subplot(4,1,4)
plot(time,A(:,3))
title('afi')

%{
Skrypt do ustawiania translacji
                                   X   Y   Z
bpy.ops.transform.translate(value=(0, -0, -0),orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)),orient_matrix_type='GLOBAL', constraint_axis=(True, False, False),mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH',proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)

Skrypt do ustawiania rotacji
bpy.ops.transform.rotate(value=-0.986386, orient_axis='Y', orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(False, True, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)

Skrypt do ustawiania klatki
bpy.context.scene.frame_current = 3

Skrypt do ustawienia obiekty aktywnego

ob = bpy.context.scene.objects["Samolot"]       # Get the object
bpy.ops.object.select_all(action='DESELECT') # Deselect all objects
bpy.context.view_layer.objects.active = ob   # Make the cube the active object 
ob.select_set(True)                          # Select the cube
%}
%Teta - wokó³ osi Y
%Fi - wokó³ osi Z
%Gamma - wokó³ osi X
% fileID = fopen('exp.txt','w');
skrypt = fopen('symulacja_2.txt','w');
translacja="bpy.ops.transform.translate(value=(%4.2f,%4.2f,%4.2f),orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)),orient_matrix_type='GLOBAL', constraint_axis=(True, False, False),mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH',proportional_size=1, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaX="bpy.ops.transform.rotate(value=%4.2f, orient_axis='X', orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaY="bpy.ops.transform.rotate(value=%4.2f, orient_axis='Y', orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(False, True, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaZ="bpy.ops.transform.rotate(value=%4.2f, orient_axis='Z', orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL', constraint_axis=(False, False, True), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=1, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
fprintf(skrypt,translacja,M(1,1),M(1,2),M(1,3));
fprintf(skrypt,rotacjaX,M(1,6));
fprintf(skrypt,rotacjaY,M(1,4));
fprintf(skrypt,rotacjaZ,M(1,5));
fclose(skrypt);
