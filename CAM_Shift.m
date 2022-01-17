clc
clear all

%% Program

nagranie = VideoReader('sytuacja_bojowa_1_2d.mkv');
filtr = VideoWriter('MeanShift_tracking.mp4');
filtr.FrameRate = nagranie.FrameRate;
vidSzer = nagranie.Width;
vidWys = nagranie.Height;
Tp=1/nagranie.FrameRate;
licznik=1;

wymiar_okienko=81;
okienko=ones(wymiar_okienko,wymiar_okienko);

wynik=zeros(vidWys,vidSzer);
[X_start,Y_start]=Detection_start(wynik,wymiar_okienko);
%G³ówny program, realizuje funkcjê pamiêci i wczytywania z niej danych
open(filtr)
while hasFrame(nagranie)
          klatka = rgb2gray(readFrame(nagranie));
          obraz2 = segmentation(klatka);
          obraz2=uint8(obraz2);
          switch licznik
              case 1 % Detekcja celu
X_detection=[];
Y_detection=[];
    for i=1:1:length(Y_start)
        Y_mean=Y_start(1,i);
        for j=1:1:length(X_start)
            
            x_lim=(size(okienko,2)-1)/2;
            y_lim=(size(okienko,1)-1)/2;
            

            x_mean=X_start(1,j);
            y_mean=Y_mean;

            delta=5;
            target=0;
            radius=5;

%Rozpoczêcie dzia³ania algorytmu
            while target==0
            %Korekcja obszaru poszukiwañ ze wzglêdu na po³o¿enie œredniej
                if x_mean+x_lim>size(obraz2,2)
                x_lim=size(obraz2,2)-x_mean;
                end

                while x_mean-x_lim<0
                    x_lim=x_lim-1;
                end

                if y_mean+y_lim>size(obraz2,1)
                    y_lim=size(obraz2,1)-y_mean;
                end

                while y_mean-y_lim<0
                    y_lim=y_lim-1;
                end  

            x_mean_prev=x_mean;
            y_mean_prev=y_mean;
            
        Xsum=zeros(1,2*x_lim);
        Ysum=zeros(1,2*y_lim);
            %Przeszukiwanie obszaru
            for ii=y_mean-y_lim+1:1:y_mean+y_lim
                for jj=x_mean-x_lim+1:1:x_mean+x_lim
                    x=jj*obraz2(ii,jj);
                    y=ii*obraz2(ii,jj);
                    Xsum(1,jj)=x;
                    Ysum(1,ii)=y;
                end
            end

            x_mean=floor(mean(Xsum)/255);
            y_mean=floor(mean(Ysum)/255);
            shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;

            if shift<delta
                target=1;
                Xc=x_mean;
                Yc=y_mean;
                %Sprawdzenie, czy obszar przebywania kandydata na cel jest
                %po³o¿ony w pobli¿u innego
                if isempty(X_detection)==0
                    for c=1:1:length(X_detection)
                        is_target_candidate_x=JestwZakresie(X_detection(1,c),Xc+radius,Xc-radius);
                        is_target_candidate_y=JestwZakresie(Y_detection(1,c),Yc+radius,Yc-radius); 
                        if is_target_candidate_x+is_target_candidate_y==2 
                            target_candidate_x=mean([X_detection(1,c),Xc]);
                            Xc=target_candidate_x;
                            X_detection(1,c)=target_candidate_x;
                            target_candidate_y=mean([Y_detection(1,c),Yc]);
                            Yc=target_candidate_y;
                            Y_detection(1,c)=target_candidate_y;                            
                        end
                    end
                end

                X_detection=[X_detection Xc];
                Y_detection=[Y_detection Yc];
            end

            end

%------------------------------------------------------------------------            

        end
    end
    Xmean=mode(X_detection);
    Ymean=mode(Y_detection);
    
                  %[Xmean,Ymean]=detection(obraz2,wymiar_okienko,X_start,Y_start);
                  wynik=uint8(insertShape(klatka,'Circle',[Xmean Ymean 40],'Color','black','LineWidth',5));
                  if Xmean+Ymean>0
                      licznik=2;
                  end
              case 2
                  [Xmean,Ymean]=meanshift(obraz2,wymiar_okienko,Xmean,Ymean);
                  wynik=uint8(insertShape(klatka,'Circle',[Xmean Ymean 40],'Color','black','LineWidth',5));
                  if Xmean+Ymean==0
                      licznik = 1;
                  end      
              otherwise
                  disp('B³¹d dzia³ania algorytmu, wezwij in¿yniera')
          end  
          writeVideo(filtr,uint8(wynik))
          montage({klatka,obraz2,uint8(wynik)})
end
close(filtr)


%% Funkcje

%Funkcja do przeszukiwania ca³ego obrazu
function [Xc,Yc]=detection(klatka,kernel,X,Y)
X_detection=[];
Y_detection=[];
    for i=1:1:length(Y)
        Y_mean=Y(1,i);
        for j=1:1:length(X)
            
            x_lim=(size(kernel,2)-1)/2;
            y_lim=(size(kernel,1)-1)/2;
            

            x_mean=X(1,j);
            y_mean=Y_mean;

            delta=5;
            target=0;
            radius=5;

%Rozpoczêcie dzia³ania algorytmu
            while target==0
            %Korekcja obszaru poszukiwañ ze wzglêdu na po³o¿enie œredniej
                if x_mean+x_lim>size(klatka,2)
                x_lim=size(klatka,2)-x_mean;
                end

                while x_mean-x_lim<0
                    x_lim=x_lim-1;
                end

                if y_mean+y_lim>size(klatka,1)
                    y_lim=size(klatka,1)-y_mean;
                end

                while y_mean-y_lim<0
                    y_lim=y_lim-1;
                end  
                
            Xsum=[];
            Ysum=[];

            x_mean_prev=x_mean;
            y_mean_prev=y_mean;
            
            %Przeszukiwanie obszaru
            for ii=y_mean-y_lim+1:1:y_mean+y_lim
                for jj=x_mean-x_lim+1:1:x_mean+x_lim
                    x=jj*klatka(ii,jj);
                    y=ii*klatka(ii,jj);
                    Xsum=[Xsum x];
                    Ysum=[Ysum y];
                end
            end

            x_mean=floor(mean(X)/255);
            y_mean=floor(mean(Y)/255);
            shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;

            if shift<delta
                target=1;
                Xc=x_mean;
                Yc=y_mean;
                %Sprawdzenie, czy obszar przebywania kandydata na cel jest
                %po³o¿ony w pobli¿u innego
                if isempty(X_detection)==0
                    for i=1:1:length(X_detection)
                        is_target_candidate_x=JestwZakresie(X_detection(1,i),Xc+radius,Xc-radius);
                        is_target_candidate_y=JestwZakresie(Y_detection(1,i),Yc+radius,Yc-radius); 
                        if is_target_candidate_x+is_target_candidate_y==2 
                            target_candidate_x=mean(X_detection(1,i),Xc);
                            Xc=target_candidate_x;
                            X_detection(1,i)=target_candidate_x;
                            target_candidate_y=mean(Y_detection(1,i),Yc);
                            Yc=target_candidate_y;
                            Y_detection(1,i)=target_candidate_y;                            
                        end
                    end
                end

                X_detection=[X_detection Xc];
                Y_detection=[Y_detection Yc];
            end

            end

%------------------------------------------------------------------------            

        end
    end
    Xc=mode(X_detection);
    Yc=mode(Y_detection);
end

%Funkcja do przydzielania wspó³rzêdnych pocz¹tkowych
function [X,Y]=Detection_start(klatka,kernel)
wys=size(klatka,1);
szer=size(klatka,2);
lim=(kernel+1)/2;
X=[];
Y=[];

ny=floor(wys/kernel);
nx=floor(szer/kernel);

for y=lim:kernel:ny*kernel
    for x=lim:kernel:nx*kernel
        X=[X x];
    end
end
for x=lim:kernel:nx*kernel
    for y=lim:kernel:ny*kernel
        Y=[Y y];
    end
end
end

%Funkcja do wyszukiwania œredniej ze wskazanych wspó³rzêdnych
function [Xc,Yc]=meanshift(klatka,kernel,x_mean,y_mean)
x_lim=(size(kernel,2)-1)/2;
y_lim=(size(kernel,1)-1)/2;

delta=2;
target=0;


while target==0
    
    if x_mean+x_lim>size(klatka,2)
    x_lim=size(klatka,2)-x_mean;
    end

    while x_mean-x_lim<0
        x_lim=x_lim-1;
    end

    if y_mean+y_lim>size(klatka,1)
        y_lim=size(klatka,1)-y_mean;
    end

    while y_mean-y_lim<0
        y_lim=y_lim-1;
    end  
        Xsum=[];
        Ysum=[];    
        x_mean_prev=x_mean;
        y_mean_prev=y_mean;
        mean_intensity=0;
    for i=y_mean-y_lim+1:1:y_mean+y_lim
        for j=x_mean-x_lim+1:1:x_mean+x_lim
            if klatka(i,j)==255
                Xsum=[Xsum j];
                Ysum=[Ysum i];
                mean_intensity=klatka(i,j)+mean_intensity;
            end
        end
    end

        x_mean=floor(mean(Xsum));
        y_mean=floor(mean(Ysum));

    if mean_intensity==0
        target=1;
        x_mean=0;
        y_mean=0;
    end

shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;

    if shift<delta
        target=1;
        Xc=x_mean;
        Yc=y_mean;
    end

end

end

%Funkcja do badania zakresu
function [tak_czy_nie]=JestwZakresie(liczba,max,min)
if liczba<max
    if liczba>min
        tak_czy_nie=1;
    else
        tak_czy_nie=0;
    end
else
    tak_czy_nie=0;
end
        
end
