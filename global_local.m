clc
clear all

teta=0;
gamma=0;
fi=0;
Rx=[1 0 0;
    0 cosd(teta) -sind(teta);
    0 sind(teta) cosd(teta)];
Ry=[cosd(gamma) 0 sind(gamma);
    0 1 0;
    -sind(gamma) 0 cosd(gamma)];
Rz=[cosd(fi) -sind(fi) 0;
    sind(fi) cosd(fi) 0;
    0 0 1];
R0=Rx*Ry*Rz


teta=-45;
gamma=0;
fi=0;
Rx=[1 0 0;
    0 cosd(teta) -sind(teta);
    0 sind(teta) cosd(teta)];
Ry=[cosd(gamma) 0 sind(gamma);
    0 1 0;
    -sind(gamma) 0 cosd(gamma)];
Rz=[cosd(fi) -sind(fi) 0;
    sind(fi) cosd(fi) 0;
    0 0 1];
R1=Rx*Ry*Rz
R1=R0*R1
R1=R1'

teta=0;
gamma=60;
fi=0;
Rx=[1 0 0;
    0 cosd(teta) -sind(teta);
    0 sind(teta) cosd(teta)];
Ry=[cosd(gamma) 0 sind(gamma);
    0 1 0;
    -sind(gamma) 0 cosd(gamma)];
Rz=[cosd(fi) -sind(fi) 0;
    sind(fi) cosd(fi) 0;
    0 0 1];
R2=Rx*Ry*Rz
R2=R1*R2
R2=R2'

teta=0;
gamma=0;
fi=30;
Rx=[1 0 0;
    0 cosd(teta) -sind(teta);
    0 sind(teta) cosd(teta)];
Ry=[cosd(gamma) 0 sind(gamma);
    0 1 0;
    -sind(gamma) 0 cosd(gamma)];
Rz=[cosd(fi) -sind(fi) 0;
    sind(fi) cosd(fi) 0;
    0 0 1];
R3=Rx*Ry*Rz
R3=R2*R3
R3=R3'

