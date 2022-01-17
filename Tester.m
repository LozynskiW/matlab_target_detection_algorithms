clc
test=[];
for i=0:1:3
disp('Wybierz sytuacjê bojow¹ do testowania: ')
disp('0. Sytuacja bojowa nr 0 - nieruchoma kamera, ruchomy cel, sta³e oœwietlenie')
disp('1. Sytuacja bojowa nr 1 - zmienne oœwietlenie')
disp('2. Sytuacja bojowa nr 2 - obiekt przes³oniêty w czasie lotu')
disp('3. Sytuacja bojowa nr 3 - obiekt lec¹cy w kierunku s³oñca')
%animacja=input('Wybierz sytuacjê bojow¹ do testów: ');
animacja=i;
%% Trenowanie detektora

if i==0
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
end

switch animacja
    case 0
        syt_boj = VideoReader('syt_boj_0_16.mkv');
        syt_boj_wzor = VideoReader('syt_boj_0_wzor_16.mkv');
        nazwa='syt_boj_0_16.mkv';
        wyniki_com='COM_0.png';
        wyniki_of='OF_0.png';
        wyniki_ms='MS_0.png';
        
        x_wyniki_com='COM_0X.png';
        y_wyniki_com='COM_0Y.png';
        x_wyniki_of='OF_0X.png';
        y_wyniki_of='OF_0Y.png';
        x_wyniki_ms='MS_0X.png';
        y_wyniki_ms='MS_0Y.png';
        
        hist_com='COM_0_hist.png';
        hist_of='OF_0_hist.png';
        hist_ms='MS_0_hist.png'; 
        
        x_hist_com='COM_0X_hist.png';
        y_hist_com='COM_0Y_hist.png';
        x_hist_of='OF_0X_hist.png';
        y_hist_of='OF_0Y_hist.png';
        x_hist_ms='MS_0X_hist.png';
        y_hist_ms='MS_0Y_hist.png';
        
        fileID = fopen('C:\\syt_boj_0.txt','r');
        title_com='Wyniki algorytmu Center of Mass podczas sytuacji bojowej nr 0';
        title_of='Wyniki algorytmu Optical Flow podczas sytuacji bojowej nr 0';
        title_ms='Wyniki algorytmu Mean Shift podczas sytuacji bojowej nr 0';
    case 1
        syt_boj = VideoReader('syt_boj_1.mkv');
        syt_boj_wzor = VideoReader('syt_boj_1_wzor.mkv');
        nazwa='syt_boj_1.mkv';
        wyniki_com='COM_1.png';
        wyniki_of='OF_1.png';
        wyniki_ms='MS_1.png'; 
        
        x_wyniki_com='COM_1X.png';
        y_wyniki_com='COM_1Y.png';
        x_wyniki_of='OF_1X.png';
        y_wyniki_of='OF_1Y.png';
        x_wyniki_ms='MS_1X.png';
        y_wyniki_ms='MS_1Y.png';
        
        hist_com='COM_1_hist.png';
        hist_of='OF_1_hist.png';
        hist_ms='MS_1_hist.png'; 
        
        x_hist_com='COM_1X_hist.png';
        y_hist_com='COM_1Y_hist.png';
        x_hist_of='OF_1X_hist.png';
        y_hist_of='OF_1Y_hist.png';
        x_hist_ms='MS_1X_hist.png';
        y_hist_ms='MS_1Y_hist.png';
        
        fileID = fopen('C:\\syt_boj_1.txt','r');
        title_com='Wyniki algorytmu Center of Mass podczas sytuacji bojowej nr 1';
        title_of='Wyniki algorytmu Optical Flow podczas sytuacji bojowej nr 1';
        title_ms='Wyniki algorytmu Mean Shift podczas sytuacji bojowej nr 1';        
    case 2
        syt_boj = VideoReader('syt_boj_2.mkv');
        syt_boj_wzor = VideoReader('syt_boj_2_wzor.mkv');
        nazwa='syt_boj_2.mkv';
        wyniki_com='COM_2.png';
        wyniki_of='OF_2.png';
        wyniki_ms='MS_2.png'; 
        
        x_wyniki_com='COM_2X.png';
        y_wyniki_com='COM_2Y.png';
        x_wyniki_of='OF_2X.png';
        y_wyniki_of='OF_2Y.png';
        x_wyniki_ms='MS_2X.png';
        y_wyniki_ms='MS_2Y.png';
        
        hist_com='COM_2_hist.png';
        hist_of='OF_2_hist.png';
        hist_ms='MS_2_hist.png'; 
        
        x_hist_com='COM_2X_hist.png';
        y_hist_com='COM_2Y_hist.png';
        x_hist_of='OF_2X_hist.png';
        y_hist_of='OF_2Y_hist.png';
        x_hist_ms='MS_2X_hist.png';
        y_hist_ms='MS_2Y_hist.png';   
        
        fileID = fopen('C:\\syt_boj_2.txt','r');
        title_com='Wyniki algorytmu Center of Mass podczas sytuacji bojowej nr 2';
        title_of='Wyniki algorytmu Optical Flow podczas sytuacji bojowej nr 2';
        title_ms='Wyniki algorytmu Mean Shift podczas sytuacji bojowej nr 2';        
    case 3
        syt_boj = VideoReader('syt_boj_31.mkv');
        syt_boj_wzor = VideoReader('syt_boj_3_wzor.mkv');
        nazwa='syt_boj_3.mkv';
        wyniki_com='COM_3.png';
        wyniki_of='OF_3.png';
        wyniki_ms='MS_3.png';
        
        x_wyniki_com='COM_3X.png';
        y_wyniki_com='COM_3Y.png';
        x_wyniki_of='OF_3X.png';
        y_wyniki_of='OF_3Y.png';
        x_wyniki_ms='MS_3X.png';
        y_wyniki_ms='MS_3Y.png';
        
        hist_com='COM_3_hist.png';
        hist_of='OF_3_hist.png';
        hist_ms='MS_3_hist.png'; 
        
        x_hist_com='COM_3X_hist.png';
        y_hist_com='COM_3Y_hist.png';
        x_hist_of='OF_3X_hist.png';
        y_hist_of='OF_3Y_hist.png';
        x_hist_ms='MS_3X_hist.png';
        y_hist_ms='MS_3Y_hist.png';
        
        fileID = fopen('C:\\syt_boj_3.txt','r');
        title_com='Wyniki algorytmu Center of Mass podczas sytuacji bojowej nr 3';
        title_of='Wyniki algorytmu Optical Flow podczas sytuacji bojowej nr 3';
        title_ms='Wyniki algorytmu Mean Shift podczas sytuacji bojowej nr 3';        
    otherwise
        disp('Brak animacji do wybranej sytuacji bojowej')
end
%Zmienne do testowania

X_true=[];
Y_true=[];
while hasFrame(syt_boj_wzor)
          klatka_akwizycja = rgb2gray(readFrame(syt_boj_wzor));
          klatka_segmentacja = edge(klatka_akwizycja,'Prewitt');
          %klatka_segmentacja =  bwmorph(klatka_segmentacja,'clean');
          klatka_segmentacja =  bwmorph(klatka_segmentacja,'bridge');       
          klatka_segmentacja=uint8(255*klatka_segmentacja); 
          [Xt,Yt]=center_of_mass(klatka_segmentacja);
          X_true=[X_true Xt];
          Y_true=[Y_true Yt];

end
close

%Zmienne do wyników
frames=zeros(1,length(X_true));
%Center of Mass
X_com=zeros(1,length(X_true));
Y_com=zeros(1,length(Y_true));

X_com_kalman=zeros(1,length(X_true));
Y_com_kalman=zeros(1,length(Y_true));
%Optical Flow
X_of=zeros(1,length(X_true));
Y_of=zeros(1,length(Y_true));

X_of_kalman=zeros(1,length(X_true));
Y_of_kalman=zeros(1,length(Y_true));
%Mean Shift
X_ms=zeros(1,length(X_true));
Y_ms=zeros(1,length(Y_true));

X_ms_kalman=zeros(1,length(X_true));
Y_ms_kalman=zeros(1,length(Y_true));

%% Center Of Mass

vidSzer = syt_boj.Width;
vidWys = syt_boj.Height;
Xc=[];
Yc=[];
zakres=70;
n=1;
wynik=zeros(vidWys,vidSzer);
mask = zeros(size(wynik));
mask(25:end-25,25:end-25) = 1;

%G³ówny program, realizuje funkcjê pamiêci i wczytywania z niej danych
kalmanFilter = []; 
isTrackInitialized = false;
trackedLocation=[];
while hasFrame(syt_boj)
      %Akwizycja
      klatka_akwizycja = rgb2gray(readFrame(syt_boj));
      klatka_segmentacja = edge(klatka_akwizycja,'Prewitt');
      klatka_segmentacja =  bwmorph(klatka_segmentacja,'clean');
      klatka_segmentacja =  bwmorph(klatka_segmentacja,'bridge');
      klatka_segmentacja = uint8(255*klatka_segmentacja);
      klatka_segmentacja = medfilt2(klatka_segmentacja);
      [Xc,Yc]=center_of_mass(klatka_segmentacja);        
          if Xc+Yc==0
              Xc=0;
              Yc=0;
          end
    
      if Xc+Yc>0
          wynik=uint8(insertShape(klatka_akwizycja,'Circle',[Xc Yc 40],'Color','black','LineWidth',5));
      else
          wynik=klatka_akwizycja;
      end
       %Zebranie wyników
            X_com(1,n)=Xc;
            Y_com(1,n)=Yc;
        frames(1,n)=n;
          n=n+1;
end

%% Optical Flow

nagranie = syt_boj;

kalmanFilter = []; 
isTrackInitialized = false;
trackedLocation=[];
n=1;

% Create a cascade detector object.
TargetDetector = vision.CascadeObjectDetector('AerialTargetDetector.xml');

% Read a video frame and run the face detector.
videoFileReader = vision.VideoFileReader(nazwa);
videoFrame      = step(videoFileReader);
bbox            = step(TargetDetector, videoFrame);

if size(bbox,2)>1
    points=[];
else
% Draw the returned bounding box around the detected target.
videoFrame = insertShape(videoFrame, 'Rectangle', bbox);
figure; imshow(videoFrame); title('Detected target');

detectedLocation=[bbox(1,1),bbox(1,2)];
% Convert the first box into a list of 4 points
% This is needed to be able to visualize the rotation of the object.
bboxPoints = bbox2points(bbox(1, :));

points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', bbox);

% Display the detected points.
figure, imshow(videoFrame), hold on, title('Detected features');
plot(points);

pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Initialize the tracker with the initial point locations and the initial
% video frame.
points = points.Location;
initialize(pointTracker, points, videoFrame);

videoPlayer  = vision.VideoPlayer('Position',...
    [100 100 [size(videoFrame, 2), size(videoFrame, 1)]+30]);
end
oldPoints = points;
while ~isDone(videoFileReader)
    % get the next frame
    videoFrame = step(videoFileReader);

    % Track the points. Note that some points may be lost.
        if isempty(points)==0
            [points, isFound] = step(pointTracker, videoFrame);
            visiblePoints = points(isFound, :);
            oldInliers = oldPoints(isFound, :);
        else
            visiblePoints=[];
        end    
    if size(visiblePoints, 1) >= 2 % need at least 2 points
        
        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
        
        % Apply the transformation to the bounding box points
        bboxPoints = transformPointsForward(xform, bboxPoints);
                
        % Insert a bounding box around the object being tracked
        bboxPolygon = reshape(bboxPoints', 1, []);
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, ...
            'LineWidth', 2);
                
        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints, '+', ...
            'Color', 'white');       
        
        % Reset the points
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints);        
    else
        TargetDetector = vision.CascadeObjectDetector('AerialTargetDetector.xml');

        % Read a video frame and run the face detector.
        clear bbox
        bbox = step(TargetDetector, videoFrame);
        if size(bbox,1)>1
            bbox=bbox(1,:);
        end

        % Draw the returned bounding box around the detected target.
        videoFrame = insertShape(videoFrame, 'Rectangle', bbox);

        % Convert the first box into a list of 4 points
        % This is needed to be able to visualize the rotation of the object.
        if isempty(bbox)==0
        bboxPoints = bbox2points(bbox(1, :));
        points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', bbox);
        pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
        % Initialize the tracker with the initial point locations and the initial
        % video frame.
        points = points.Location;
        end
        if isempty(points)==0
            release(pointTracker);
            initialize(pointTracker, points, videoFrame);
            oldPoints = points;
        end
    end
    
    if isempty(bbox)==0
        przes_x=floor(bbox(1,3)/2);
        przes_y=floor(bbox(1,4)/2);
        detectedLocation=[bbox(1,1),bbox(1,2)];
        isObjectDetected=1;
    else
        isObjectDetected=0;
    end
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
           if isempty(bbox)==0
               
               circle(1,1)=floor(circle(1,1))+przes_x;
               circle(1,2)=floor(circle(1,2))+przes_y;
           else
               circle(1,1)=floor(circle(1,1));
               circle(1,2)=floor(circle(1,2));
           end
       end
       %Zebranie wyników
       if isempty(bbox)==0
            X_of(1,n)=bbox(1,1)+przes_x;
            Y_of(1,n)=bbox(1,2)+przes_y;
       else
            X_of(1,n)=0;
            Y_of(1,n)=0;
       end
        if isempty(trackedLocation)==0
            X_of_kalman(1,n)=floor(circle(1,1));
            Y_of_kalman(1,n)=floor(circle(1,2));
        else
            X_of_kalman(1,n)=0;
            Y_of_kalman(1,n)=0;            
        end
        frames(1,n)=n;
          n=n+1;       
       
end

%% Mean Shift

nagranie2 = VideoReader(nazwa);
vidSzer = nagranie2.Width;
vidWys = nagranie2.Height;
Tp=1/nagranie2.FrameRate;
licznik=1;
n=1;
wymiar_okienko=81;
okienko=ones(wymiar_okienko,wymiar_okienko);
kalmanFilter = []; 
isTrackInitialized = false;
trackedLocation=[];

wynik=zeros(vidWys,vidSzer);
[X_start,Y_start]=Detection_start(wynik,wymiar_okienko);
%G³ówny program, realizuje funkcjê pamiêci i wczytywania z niej danych

while hasFrame(nagranie2)
      klatka_akwizycja = rgb2gray(readFrame(nagranie2));
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
                            isObjectDetected=0;
                            detectedLocation=[];
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
                  if x_mean+y_mean>0
                      Xmean=x_mean;
                      Ymean=y_mean;
                      licznik=2;
                      isObjectDetected=1;
                      detectedLocation=[Xmean, Ymean];
                  end
              case 2                  
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
                            detectedLocation=[];
                            isObjectDetected=0;
                        else
                            shift=((x_mean-x_mean_prev)+(y_mean-y_mean_prev))/2;
                             if shift<delta
                                target=1;
                                Xc=x_mean;
                                Yc=y_mean;
                                detectedLocation=[Xmean, Ymean];
                            end                           
                        end

                        end
                  
                  wynik=uint8(insertShape(uint8(klatka_akwizycja),'Circle',[Xc Yc 40],'Color','black','LineWidth',5)); 
              otherwise
                  disp('B³¹d dzia³ania algorytmu, wezwij in¿yniera')
          end
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
          X_ms(1,n)=Xc;
          Y_ms(1,n)=Yc;
      %Zebranie wyników
        if isempty(trackedLocation)==0
            X_ms_kalman(1,n)=floor(circle(1,1));
            Y_ms_kalman(1,n)=floor(circle(1,2));
        else
            X_ms_kalman(1,n)=0;
            Y_ms_kalman(1,n)=0;            
        end
        n=n+1;
end

%% Wyniki testu

%Wyliczenie odl do celu
A=fscanf(fileID,'%d');
x=zeros(1,length(A)/3);
y=zeros(1,length(A)/3);
z=zeros(1,length(A)/3);
odl_m=zeros(1,length(A)/3);
for k=0:1:length(A)/3-1
    x(1,k+1)=A(1+3*k,1);
    y(1,k+1)=A(2+3*k,1);
    z(1,k+1)=A(3+3*k,1);
    odl_m(1,k+1)=sqrt(x(1,k+1)^2+y(1,k+1)^2+z(1,k+1)^2);
end
%Przeliczenie pikseli na metry
odl_p=zeros(1,length(odl_m));
if i==0
    deg=30;
else
    deg=1;
end
for j=1:1:length(odl_p)
    odl_p(1,j)=odl_m(1,j)*tand(deg)/(0.5*vidSzer);
end
%Center of mass
X_com_wyniki=X_com-X_true;
Y_com_wyniki=Y_com-Y_true;
%Com_wyniki=(X_com_wyniki+Y_com_wyniki)/2;
%Com_wyniki_m=Com_wyniki.*odl_p;
X_com_wyniki_m=X_com_wyniki.*odl_p;
Y_com_wyniki_m=Y_com_wyniki.*odl_p;

%{
X_com_wyniki_kalman=X_com_kalman-X_true;
Y_com_wyniki_kalman=Y_com_kalman-Y_true;
Com_wyniki_kalman=(X_com_wyniki_kalman+Y_com_wyniki_kalman)/2;
Com_wyniki_kalman_m=Com_wyniki_kalman.*odl_p;

%}

%figure, plot(frames,Com_wyniki_m)
figure, plot(frames,X_com_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi X w metrach')
saveas(gcf,x_wyniki_com)

figure, plot(frames,Y_com_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi Y w metrach')
saveas(gcf,y_wyniki_com)

histogram_com_x=histogram(X_com_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi X')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,x_hist_com)

histogram_com_y=histogram(Y_com_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi Y')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,y_hist_com)

%Optical flow
X_of_wyniki=X_of-X_true;
Y_of_wyniki=Y_of-Y_true;
%Of_wyniki=(X_of_wyniki+Y_of_wyniki)/2;
%Of_wyniki_m=Of_wyniki.*odl_p;
X_of_wyniki_m=X_of_wyniki.*odl_p;
Y_of_wyniki_m=Y_of_wyniki.*odl_p;

%{
X_of_wyniki_kalman=X_of_kalman-X_true;
Y_of_wyniki_kalman=Y_of_kalman-Y_true;
Of_wyniki_kalman=(X_of_wyniki_kalman+Y_of_wyniki_kalman)/2;
Of_wyniki_kalman_m=Of_wyniki_kalman.*odl_p;
%}

figure, plot(frames,X_of_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi X w metrach')
saveas(gcf,x_wyniki_of)

figure, plot(frames,Y_of_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi Y w metrach')
saveas(gcf,y_wyniki_of)

histogram_of_x=histogram(X_of_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi X')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,x_hist_of)

histogram_of_y=histogram(Y_of_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi Y')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,y_hist_of)

%Mean Shift
X_ms_wyniki=X_ms-X_true;
Y_ms_wyniki=Y_ms-Y_true;
%Ms_wyniki=(X_ms_wyniki+Y_ms_wyniki)/2;
%Ms_wyniki_m=Ms_wyniki.*odl_p;
X_ms_wyniki_m=X_ms_wyniki.*odl_p;
Y_ms_wyniki_m=Y_ms_wyniki.*odl_p;

%{
X_ms_wyniki_kalman=X_ms_kalman-X_true;
Y_ms_wyniki_kalman=Y_ms_kalman-Y_true;
Ms_wyniki_kalman=(X_ms_wyniki_kalman+Y_ms_wyniki_kalman)/2;
Ms_wyniki_kalman_m=Ms_wyniki_kalman.*odl_p;
%}

figure, plot(frames,X_ms_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi X w metrach')
saveas(gcf,x_wyniki_ms)

figure, plot(frames,Y_ms_wyniki_m)
xlabel('Numer klatki nagrania');
ylabel('Œrednie odchylenie od œrodka celu w osi Y w metrach')
saveas(gcf,y_wyniki_ms)

histogram_ms_x=histogram(X_ms_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi X')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,x_hist_ms)

histogram_ms_y=histogram(Y_ms_wyniki_m,'Normalization','pdf','BinWidth',0.1);
xlabel('Wartoœæ b³êdu wskazania po³o¿enia celu w osi Y')
ylabel('Gêstoœæ prawdopodobieñstwa')
saveas(gcf,y_hist_ms)


close all
clear all
end
disp('Ukoñczone!')
close all
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
