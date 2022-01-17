clc
clear all

%% Program

nagranie = VideoReader('syt_boj_31.mkv');
filtr = VideoWriter('MeanShift_tracking.mp4');
filtr.FrameRate = nagranie.FrameRate;
vidSzer = nagranie.Width;
vidWys = nagranie.Height;
Tp=1/nagranie.FrameRate;
licznik=1;
Wynik_x=[];
Wynik_y=[];
wymiar_okienko=81;
okienko=ones(wymiar_okienko,wymiar_okienko);

wynik=zeros(vidWys,vidSzer);
[X_start,Y_start]=Detection_start(wynik,wymiar_okienko);
%G³ówny program, realizuje funkcjê pamiêci i wczytywania z niej danych
open(filtr)
while hasFrame(nagranie)
      klatka_akwizycja = rgb2gray(readFrame(nagranie));
      klatka_segmentacja = edge(klatka_akwizycja,'Prewitt');
      klatka_segmentacja =  bwmorph(klatka_segmentacja,'clean');
      klatka_segmentacja =  bwmorph(klatka_segmentacja,'bridge');
      klatka_segmentacja=uint8(255*klatka_segmentacja); 
      klatka_segmentacja = medfilt2(klatka_segmentacja);
      obraz2=klatka_segmentacja;
          switch licznik
              case 1 % Detekcja celu
                wymiar_kernel=81;
                kernel=ones(wymiar_kernel,wymiar_kernel);
                koniec=0;
                    for i=1:1:length(Y_start)
                        for j=1:1:length(X_start)
                            x_mean=X_start(1,j);
                            y_mean=Y_start(1,i);

            
                                x_lim=(size(kernel,2)-1)/2;
                                y_lim=(size(kernel,1)-1)/2;

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

                                X=[];
                                Y=[];

                                delta=2;
                                target=0;
                                stop=0;

                                while target==0
                                    
                                x_lim=(size(kernel,2)-1)/2;
                                y_lim=(size(kernel,1)-1)/2;

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
                                mean_intensity=0;
                                for ii=y_mean-y_lim+1:1:y_mean+y_lim
                                    for jj=x_mean-x_lim+1:1:x_mean+x_lim
                                        if obraz2(ii,jj)==255
                                            X=[X jj];
                                            Y=[Y ii];
                                        end
                                        mean_intensity=obraz2(ii,jj)+mean_intensity;
                                    end
                                end

                                x_mean=floor(mean(X));
                                y_mean=floor(mean(Y));

                        if mean_intensity==0
                            target=1;
                            stop=1;
                            x_mean=0;
                            y_mean=0;
                        else
                            shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;
                             if shift<delta
                                target=1;
                            end                           
                        end
                                Xc=x_mean;
                                Yc=y_mean;

                                end

                                if stop==0
                                    if Xc>0 && Yc>0
                                        koniec=1;
                                    end
                                end
                                if koniec==1
                                   break;
                                end
                            end
                             if koniec==1
                                break;
                             end
                        end
wynik=uint8(insertShape(uint8(klatka_akwizycja),'Circle',[Xc Yc 50],'Color','red','LineWidth',5));
                  %{
                  [Xmean,Ymean]=detection(obraz2,wymiar_okienko,X_start,Y_start);
                  wynik=uint8(insertShape(klatka,'Circle',[Xmean Ymean 40],'Color','black','LineWidth',5));
                  %}
                  if x_mean+y_mean>0
                      Xmean=x_mean;
                      Ymean=y_mean;
                      licznik=2;
                  end
              case 2
                  %[Xmean,Ymean,stop]=meanshift(obraz2,wymiar_okienko,Xmean,Ymean);
                  
                        x_lim=(size(kernel,2)-1)/2;
                        y_lim=(size(kernel,1)-1)/2;

                        delta=2;
                        target=0;
                        stop=0;

                        while target==0

                            if x_mean+x_lim>size(klatka_akwizycja,2)
                            x_lim=size(klatka_akwizycja,2)-x_mean;
                            end

                            while x_mean-x_lim<0
                                x_lim=x_lim-1;
                            end

                            if y_mean+y_lim>size(klatka_akwizycja,1)
                                y_lim=size(klatka_akwizycja,1)-y_mean;
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
                                if klatka_segmentacja(i,j)==255
                                    Xsum=[Xsum j];
                                    Ysum=[Ysum i];
                                end
                                mean_intensity=klatka_segmentacja(i,j)+mean_intensity;
                            end
                        end

                        x_mean=floor(mean(Xsum));
                        y_mean=floor(mean(Ysum));

                        if mean_intensity==0
                            target=1;
                            stop=1;
                            x_mean=0;
                            y_mean=0;
                            licznik=1;
                        else
                            shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;
                             if shift<delta
                                target=1;
                                Xc=x_mean;
                                Yc=y_mean;
                            end                           
                        end

                        end
                  
                  wynik=uint8(insertShape(uint8(klatka_akwizycja),'Circle',[Xc Yc 40],'Color','black','LineWidth',5));   
              otherwise
                  disp('B³¹d dzia³ania algorytmu, wezwij in¿yniera')
          end  
          writeVideo(filtr,uint8(wynik))
          montage({klatka_akwizycja,obraz2,uint8(wynik)})
          Wynik_x=[Wynik_x Xc];
          Wynik_y=[Wynik_y Yc];         
end
close(filtr)


%% Funkcje


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
function [Xc,Yc,stop]=meanshift(klatka,kernel,x_mean,y_mean)
x_lim=(size(kernel,2)-1)/2;
y_lim=(size(kernel,1)-1)/2;

delta=2;
target=0;
stop=0;

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
    stop=1;
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

%Funkcja do przeszukiwania ca³ego obrazu
function [Xc,Yc]=detection(klatka,kernel,X,Y)
koniec=0;
    for i=1:1:length(Y)
        y_mean=Y(1,i);
        for j=1:1:length(X)
            
            x_lim=(size(kernel,2)-1)/2;
            y_lim=(size(kernel,1)-1)/2;
            x_mean=X(1,j);

            delta=2;
            target=0;
            stop=0;

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
            for ii=y_mean-y_lim+1:1:y_mean+y_lim
                for jj=x_mean-x_lim+1:1:x_mean+x_lim
                    if klatka(ii,jj)==255
                        Xsum=[Xsum jj];
                        Ysum=[Ysum ii];
                        mean_intensity=klatka(ii,jj)+mean_intensity;
                    end
                end
            end

            x_mean=floor(mean(X));
            y_mean=floor(mean(Y));

            if mean_intensity==0
                target=1;
                stop=1;
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

%------------------------------------------------------------------------            

        end
    end
end
