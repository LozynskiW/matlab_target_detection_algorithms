clc
%% Zmienne
BinWidth=8; %szerokoœæ zakresów przestrzeni barw
epsilon=0.35; %dok³adnoœæ algorytmu
se = offsetstrel('ball',20,20);

%% Trenowanie detektora

%load('target1.mat');

%Select the bounding boxes for stop signs from the table.
positiveInstances = gTruth(:,1:2);

%Add the image folder to the MATLAB path.
imDir = fullfile('C:\Users\Wojciech £o¿yñski\Desktop\WAT\inzynierka\algorytmy\nauka algorytmu\pozytywne');
addpath(imDir);

%Specify the folder for negative images.
negativeFolder = fullfile('C:\Users\Wojciech £o¿yñski\Desktop\WAT\inzynierka\algorytmy\nauka algorytmu\negatywne');

%Create an imageDatastore object containing negative images.
negativeImages = imageDatastore(negativeFolder);

%Train a cascade object detector called 'stopSignDetector.xml' using HOG features. NOTE: The command can take several minutes to run.
trainCascadeObjectDetector('AerialTargetDetector.xml',positiveInstances,negativeFolder,'FalseAlarmRate',0.2,'NumCascadeStages',5);

%% Algorytm - detekcja celu
%Zmienne do testowania algorytmu
D_y=[];

% Create a cascade detector object.
TargetDetector = vision.CascadeObjectDetector('AerialTargetDetector.xml');

nagranie = VideoReader('sytuacja_bojowa_1_2d.mkv');
filtr = VideoWriter('Mean_shift_tracking.mp4');
filtr.FrameRate = nagranie.FrameRate;
vidSzer = nagranie.Width;
vidWys = nagranie.Height;
Tp=1/nagranie.FrameRate;

% Okreslenie celu
%videoFileReader = vision.VideoFileReader('sytuacja_bojowa_1_2d.mkv');
%feature      = imread('target_ms.jpg');
klatka = readFrame(nagranie);
search_window = step(TargetDetector, klatka);

%Zmiejszanie wymiarów okna obszaru poszukiwañ celu
search_window(1,1)=search_window(1,1)+floor(0.25*search_window(1,3));
search_window(1,2)=search_window(1,2)+floor(0.15*search_window(1,3));
search_window(1,3)=floor(0.5*search_window(1,3));
search_window(1,4)=search_window(1,3);

klatka=rgb2gray(klatka);
klatka=uint8(klatka);
% Segmentacja 
target_eroded = imerode(klatka,se);
iteracje=0;
kolejny=0;

figure,imshowpair(uint8(insertShape(klatka,'Rectangle',search_window,'Color','black','LineWidth',5)),uint8(insertShape(target_eroded,'Rectangle',search_window,'Color','black','LineWidth',1)),'montage')

%Wydzielenie obszaru przedstawiaj¹cego cel
target=uint8(zeros(search_window(1,4),search_window(1,3)));
for y=search_window(1,2):1:search_window(1,2)+search_window(1,4)
    for x=search_window(1,1):1:search_window(1,1)+search_window(1,3)
        target(y-search_window(1,2)+1,x-search_window(1,1)+1)=target_eroded(y,x);
    end
end

figure
target_histogram=histogram(target,'BinWidth',BinWidth,'BinLimits',[0,255],'Normalization','pdf');
target_histogram_data=target_histogram.Data;


%Oszacowanie wartoœci funkcji gêstoœci prawdopodobieñstwa celu
n=1;
Target_data=zeros(1,size(target_histogram_data,1)+size(target_histogram_data,2));
for i=1:1:size(target_histogram_data,1)
    for j=1:1:size(target_histogram_data,2)
        Target_data(1,n)=target_histogram_data(i,j);
        n=n+1;
    end
end

[target_kde,x]=ksdensity(Target_data,0:1:255,'Kernel','epanechnikov','width',BinWidth);
figure,plot(x,target_kde)

target_candidate_x=search_window(1,1);
target_candidate_y=search_window(1,2);



%% Algorytm - œledzenie celu

%-------------------Akwizycja----------------------------------------------
open(filtr)
while hasFrame(nagranie)
        Xc=target_candidate_x+search_window(1,3)/2;
        Yc=target_candidate_y+search_window(1,4)/2;
        %Akwizycja
        wynik=uint8(insertShape(klatka,'Circle',[Xc Yc 40],'Color','black','LineWidth',5));
        writeVideo(filtr,wynik)
        imshowpair(wynik,uint8(insertShape(target_eroded,'Rectangle',search_window,'Color','black','LineWidth',1)),'montage')

klatka = rgb2gray(readFrame(nagranie));
klatka=uint8(klatka);
%Segmentacja
target_eroded = imerode(klatka,se);
d_y=1;  

%------------------Algorytm Mean-Shift-------------------------------------
%Algorytm Mean Shift
while 1

%Korekta obszaru poszukiwañ w Y
if search_window(1,2)+search_window(1,4)>size(klatka,1)
    search_window(1,2)=floor(size(klatka,1)-1-search_window(1,4));
end
if search_window(1,2)<1
    search_window(1,2)=1;
end
%Korekta obszaru poszukiwañ w X
if search_window(1,1)+search_window(1,3)>size(klatka,2)
    search_window(1,1)=floor(size(klatka,2)-1-search_window(1,3));
end
if search_window(1,1)<1
    search_window(1,1)=1;
end
%Wybranie badanego obszaru
  target_candidate=uint8(zeros(search_window(1,4),search_window(1,3)));
    for y=1:1:search_window(1,4)
        for x=1:1:search_window(1,3)
            target_candidate(y,x)=target_eroded(y+search_window(1,2),x+search_window(1,1));
        end
    end
%Oszacowanie funkcji gêstoœci prawdopodobieñstwa potencjalnego celu
      target_candidate_histogram=histogram(target_candidate,'BinWidth',1,'BinLimits',[0,255],'Normalization','pdf');
      target_candidate_histogram_data=target_candidate_histogram.Data;
     
    n=1;
    Target_candidate_data=zeros(1,size(target_candidate_histogram_data,1)+size(target_candidate_histogram_data,2));
    for i=1:1:size(target_candidate_histogram_data,1)
        for j=1:1:size(target_candidate_histogram_data,2)
            Target_candidate_data(1,n)=target_candidate_histogram_data(i,j);
            n=n+1;
        end
    end
[target_candidate_kde,bins]=ksdensity(Target_candidate_data,0:1:255,'Kernel','epanechnikov','width',BinWidth); %pu

for t=1:1:length(target_candidate_kde)
    if target_candidate_kde==0
        target_candidate_kde(1,t)=0.001;
    end
end

%Obliczanie wartoœci wag dla poszczególnych wartoœci przestrzeni
%barw
        w_y=zeros(1,256);
        shift_x=zeros(1,256);
        shift_y=zeros(1,256);
        for gray_scale_bin=0:1:255
            w_y(1,gray_scale_bin+1)=sqrt(target_kde(1,gray_scale_bin+1)/target_candidate_kde(1,gray_scale_bin+1));
        end
        
%Obliczanie przesuniêcia
dzielnik=0;
    for bin=0:1:255
        for yn=search_window(1,2):1:search_window(1,2)+search_window(1,4)
            for xn=search_window(1,1):1:search_window(1,1)+search_window(1,3)
                if klatka(yn,xn)==bin
                    shift_x(1,bin+1)=w_y(1,bin+1)*xn+shift_x(1,bin+1);
                    shift_y(1,bin+1)=w_y(1,bin+1)*yn+shift_y(1,bin+1);
                    dzielnik=w_y(1,bin+1)+dzielnik;
                end
            end
        end
    end
    if dzielnik==0 %
        mean_shift_x=0;
        mean_shift_y=0;
        search_window=[];
        search_window = step(TargetDetector, klatka);
        
        search_window(1,1)=search_window(1,1)+floor(0.1*search_window(1,3));
        search_window(1,2)=search_window(1,2)+floor(0.05*search_window(1,3));
        search_window(1,3)=floor(0.7*search_window(1,3));
        search_window(1,4)=search_window(1,3);
        
    else
        mean_shift_x=floor(sum(shift_x)/dzielnik);%-search_window(1,1);
        mean_shift_y=floor(sum(shift_y)/dzielnik);%-search_window(1,2);
    end 
        target_candidate_x=mean_shift_x;%+search_window(1,1);
        target_candidate_y=mean_shift_y;%+search_window(1,2);
        
        if target_candidate_x>0
            if target_candidate_x<size(klatka,2)
            search_window(1,1)=target_candidate_x;
            search_window(1,2)=target_candidate_y;
            end
        else
            search_window=[];
            search_window = step(TargetDetector, klatka);
            
            search_window(1,1)=search_window(1,1)+floor(0.1*search_window(1,3));
            search_window(1,2)=search_window(1,2)+floor(0.05*search_window(1,3));
            search_window(1,3)=floor(0.7*search_window(1,3));
            search_window(1,4)=search_window(1,3);
            
            d_y=1;
            break
        end        

%Obliczanie wsp Bhat
        bhat_coef=zeros(1,256);
        for bin=0:1:255
            bhat_coef(1,bin+1)=sqrt(target_candidate_kde(1,bin+1)*target_kde(1,bin+1));
        end
        
        d_y=sqrt(1-sum(bhat_coef));
        if d_y<epsilon
            break
        end
        if size(D_y)>1
            if d_y==D_y(1,iteracje-1)
                search_window=[];
                search_window = step(TargetDetector, klatka);
                search_window(1,1)=search_window(1,1)+floor(0.1*search_window(1,3));
                search_window(1,2)=search_window(1,2)+floor(0.05*search_window(1,3));
                search_window(1,3)=floor(0.7*search_window(1,3));
                search_window(1,4)=search_window(1,3);
                d_y=1;
                break
            end
        end

        D_y=[D_y d_y];
        iteracje=iteracje+1;
end
%------------------Algorytm Mean-Shift-------------------------------------
kolejny=kolejny+1;
end
%-------------------Akwizycja----------------------------------------------
close(filtr)

%To do:
%Zmniejszyæ wymiar okien poszukuj¹cych cel
%Zmniejszyæ wymiar okna ustalaj¹cego cel
%Poprawiæ warunek wskazywania fa³szywego celu
