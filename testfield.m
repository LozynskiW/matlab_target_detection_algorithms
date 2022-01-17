clc

nagranie = VideoReader('syt_boj_1.mkv');
filtr = VideoWriter('Optical_flow_tracking.mp4');
filtr.FrameRate = nagranie.FrameRate;

kalmanFilter = []; 
isTrackInitialized = false;
trackedLocation=[];

Wynik_x=[];
Wynik_y=[];

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

%% Algorytm

% Create a cascade detector object.
TargetDetector = vision.CascadeObjectDetector('AerialTargetDetector.xml');

% Read a video frame and run the face detector.
videoFileReader = vision.VideoFileReader('syt_boj_1.mkv');
videoFrame      = step(videoFileReader);
bbox            = step(TargetDetector, videoFrame);

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

oldPoints = points;
open(filtr)
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
        bbox            = step(TargetDetector, videoFrame);
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
               
           wynik=uint8(insertShape(uint8(255*videoFrame),'Circle',circle,'Color','white','LineWidth',5));
       end
    % Display the annotated video frame using the video player object
    imshow(wynik)
    writeVideo(filtr,wynik)
end
close(filtr)
% Clean up
release(videoFileReader);
release(pointTracker);



   %Funkcja do kasowania zawartoœci poza bramk¹ œledz¹c¹
function [gate]=kasowanie(max_x_bramki,min_x_bramki,max_y_bramki,min_y_bramki,klatka)
szer = size(klatka,2);
wys = size(klatka,1);
        for y=1:1:wys
        for x=1:1:szer
            yp=JestwZakresie(y,max_y_bramki,min_y_bramki);
            xp=JestwZakresie(x,max_x_bramki,min_x_bramki);
            if yp+xp~=2
                klatka(y,x)=0;
            end
        end
        end
        gate=klatka;
end


%Funkcja do ustalenia rozmiarów bramki œledz¹cej

function [min_x_bramki, max_x_bramki,min_y_bramki, max_y_bramki,szer_bramki,wys_bramki,success]=bramka_sledzaca(Xc,Yc,zakres,klatka_segmentacja)
szer = size(klatka_segmentacja,2);
wys = size(klatka_segmentacja,1);
y=Yc;
y_max=1;
success=1;
while y_max>0
    y_max=max(klatka_segmentacja(y,:));
    max_y_bramki=y+zakres;
    y=y+1;
    if y==wys-1
        success=0;        
        break
    end
end
y_min=1;
while y_min>0
    y_min=max(klatka_segmentacja(y,:));
    min_y_bramki=y-zakres;
    y=y-1;
    if y==1
        success=0;
        break
    end
end

x=Xc;
x_max=1;
while x_max>0
    x_max=max(klatka_segmentacja(:,x));
    max_x_bramki=x+zakres;
    x=x+1;
    if x==szer-1
        success=0;
        break
    end
end
x_min=1;
while x_min>0
    x_min=max(klatka_segmentacja(:,x));
    min_x_bramki=x-zakres;
    x=x-1;
    if x==1
        success=0;
        break
    end
end

%Korekty rozmiarow bramki

if max_x_bramki>szer-1
    max_x_bramki=szer-1;
end

if min_x_bramki<1
    min_x_bramki=1;
end

szer_bramki=max_x_bramki-min_x_bramki;

if max_y_bramki>wys-1
    max_y_bramki=wys-1;
end

if min_y_bramki<1
    min_y_bramki=1;
end
wys_bramki=max_y_bramki-min_y_bramki;

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



