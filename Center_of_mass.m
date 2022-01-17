clc


%% Program

nagranie = VideoReader('syt_boj_0_16.mkv');
filtr = VideoWriter('COM_tracking.mp4');
filtr.FrameRate = nagranie.FrameRate;
vidSzer = nagranie.Width;
vidWys = nagranie.Height;
Tp=1/nagranie.FrameRate;
Xc=[];
Yc=[];
zakres=70;
wynik=zeros(vidWys,vidSzer);
mask = zeros(size(wynik));
mask(25:end-25,25:end-25) = 1;
X_true=[];
Y_true=[];
%G³ówny program, realizuje funkcjê pamiêci i wczytywania z niej danych
kalmanFilter = []; 
isTrackInitialized = false;
trackedLocation=[];
open(filtr)
while hasFrame(nagranie)
      %Akwizycja
      klatka_akwizycja = rgb2gray(readFrame(nagranie));
      klatka_segmentacja = edge(klatka_akwizycja,'Prewitt');
      %klatka_segmentacja =  bwmorph(klatka_segmentacja,'clean');
      klatka_segmentacja =  bwmorph(klatka_segmentacja,'bridge');
      klatka_segmentacja = activecontour(klatka_segmentacja, mask, 100, 'edge');
      klatka_segmentacja=uint8(255*klatka_segmentacja); 
      klatka_segmentacja = medfilt2(klatka_segmentacja);
      [Xc,Yc]=center_of_mass(klatka_segmentacja);     
          if Xc+Yc==0
              Xc=[];
              Yc=[];
          end
      detectedLocation=[Xc,Yc];
      isObjectDetected = size(detectedLocation, 1) > 0;
      
      if Xc+Yc>0
          wynik=uint8(insertShape(klatka_akwizycja,'Circle',[Xc Yc 40],'Color','black','LineWidth',5));
      else
          wynik=klatka_akwizycja;
      end
          X_true=[X_true Xc];
          Y_true=[Y_true Yc];
      %Filtr Kalmana
       if ~isTrackInitialized
           if isObjectDetected
             kalmanFilter = configureKalmanFilter('ConstantAcceleration',...
                      detectedLocation(1,:), [1 1 1]*1e5, [25, 10, 10], 25);
             isTrackInitialized = true;
           end
           label = ''; circle = zeros(0,3);
       else
           if isObjectDetected
             predict(kalmanFilter);
             trackedLocation = correct(kalmanFilter, detectedLocation(1,:));
             label = 'Corrected';
                 
           else
             trackedLocation = predict(kalmanFilter);
             label = 'Predicted';
           end
           circle = [trackedLocation, 5];
           circle(1,1)=floor(circle(1,1));
           circle(1,2)=floor(circle(1,2));
           wynik=uint8(insertShape(wynik,'Circle',circle,'Color','white','LineWidth',5));
       end
      writeVideo(filtr,wynik)
      montage({klatka_akwizycja,klatka_segmentacja,wynik})    
      
end
close(filtr)

   
   %% Funkcje
%funkcja wyliczaj¹ca œrodek masy
function [X,Y] = center_of_mass(klatka)
Mass=0;  
n=size(klatka);
wys=n(1,1);
szer=n(1,2);

MOX=zeros(1,szer);  
MOY=zeros(1,wys);
My=0;
Mx=0;

%Dla osi Y
for y=1:wys
    My_i=0;
    for x=1:szer
        if klatka(y,x)==255
            Mass=Mass+1;
            My_i=My_i+1;
        end 
    end
    MOY(1,y)=My_i*y;
end
My=sum(MOY);

%Dla osi X
for x=1:szer
    Mx_i=0;
    for y=1:wys
        if klatka(y,x)==255
            Mx_i=Mx_i+1;
        end 
    end
    MOX(1,x)=Mx_i*x;
end
Mx=sum(MOX);
if Mass>0
X=round(Mx/Mass);
Y=round(My/Mass);
else
X=0;
Y=0;   
end
end
