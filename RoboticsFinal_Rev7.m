%% Aamir Baksh, Ben Bolt, Justin Kibbe
% Robotics Final: Climber Tracking
clc; close all; clear;

% https://www.mathworks.com/help/vision/ug/augmented-reality-using-apriltag-markers.html

%% Initializing
arduinoObj = arduino("COM3", "Mega2560",'Libraries','Adafruit\MotorShieldV2');
shield = addon(arduinoObj,'Adafruit\MotorShieldV2');
m5 = dcmotor(shield,1); % Positive =
m4 = dcmotor(shield,4); % Positive =
m3 = dcmotor(shield,3); % Positive =
m2 = dcmotor(shield,2); % Positive =
stop(m2);
stop(m5);
curl(1, m2, m3, m4);

cam = webcam('HD Web Camera');                 % Change this to match your camera name
ImageFolder ='D:\Google Drive\ECE 6311_Introduction to Robotics\Demo\Final Project\LockPictures';
centered = false;
lastVal1 = 0; %difference in x and y for previous
lastVal2 = 0; %difference in x and y
CentX = 0;
CentY = 0;
lockCount = 0;
isInside = false;


IDwewant = 2;  % which tag are you looking for.


medianFilterSz = 1;
lastPoses = zeros(4,2,medianFilterSz);

%% Loop to calculate and display centroid location and aprilTag endpoints
while(centered == false)
    AprilTagIMG = snapshot(cam);
    FinalIMG = imrotate(AprilTagIMG,90);
    %FinalIMG = AprilTagIMG;
    tagFamily = ("tag36h11");   %todo: you can tell AprilTag you are only looking for tag '3'

    [id,loc,detectedFamily] = readAprilTag(FinalIMG,tagFamily);
    tempDiff = 0;

    idx = find(id==IDwewant,1,'first');
    stop(m5);
    stop(m2);
    stop(m3);


    if (~isempty(idx))


        lastPoses(:,:,1:medianFilterSz-1) = lastPoses(:,:,2:medianFilterSz);
        lastPoses(:,:,end) = loc(:,:,idx);  % look at the top left


        % Insert markers to indicate the locations
        %  if (tempDiff < lastVal1+25 && tempDiff > lastVal1-25 && tempDiff < lastVal2+25 && tempDiff > lastVal2-25)
        markerRadius = 15;
        numCorners = size(loc,1);
        %markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];

        markerPosition = [median(lastPoses(:,:,:),3),repmat(markerRadius,numCorners,1)];


        FinalIMG = insertShape(FinalIMG,"FilledCircle",markerPosition,"Color","red");

        %Locate the centroid of the AprilTag
        markerPosition(:,3)=[];                                                                 %Removes the 3rd colum from "markerPosition" so that there are just x,y columns
        %tagArea = (max(makerPosition(:,1)) - min(markerPosition(:,1))) .* ((max(makerPosition(:,2)) - min(markerPosition(:,2))));
        AprilTagCentroidXY = mean(markerPosition);                                              %Defines the centroid from average of 4 X-values and 4 Y-Values
        CentX = AprilTagCentroidXY(1,1);                                                        %Isolates the 1st column of  AprilTagCentroidXY so that only the X value remains
        CentY = AprilTagCentroidXY(1,2);                                                        %Isolates the 2nd column of  AprilTagCentroidXY so that only the Y value remains
        FinalIMG = insertMarker(FinalIMG,AprilTagCentroidXY,"x-mark","Color","blue",Size=20);   %Inserts an "X" on the image at the defined centroid
        imshow(FinalIMG)
        CentDist = (((markerPosition(1,1))-(CentX)).^2 + ((markerPosition(1,2))-(CentY)).^2).^(1/2);
        hold on
        th = 0:pi/50:2*pi;
        lockfactor = 3;
        xunit = CentDist*lockfactor * cos(th) + CentX;
        yunit = CentDist*lockfactor * sin (th) + CentY;
        plot(xunit,yunit,'green','LineWidth',2);
        [m1_X, m1_Y, m2_X, m2_Y] = wristTrack(FinalIMG);
        if (m1_X ~=0 && m1_Y ~=0 && m2_X ~= 0 && m2_Y ~= 0)
        CentWristDist1 = sqrt((m1_X-CentX)^2+(m1_Y-CentY)^2);
        CentWristDist2 = sqrt((m2_X-CentX)^2+(m2_Y-CentY)^2);
%% Motor movement to center camera on apriltag %%
        if (CentWristDist1 < CentDist*lockfactor && CentWristDist2 < CentDist*lockfactor && isInside == false)
            isInside = true;
            lockCount = lockCount + 1
            file_name = sprintf('LockNum%d.png', lockCount);
            fullFileName = fullfile(ImageFolder, file_name);
            imwrite(FinalIMG,file_name,'png');
        elseif (CentWristDist1 < CentDist*lockfactor && CentWristDist2 < CentDist*lockfactor && isInside == true)
        else
            isInside = false;
        end
        end
        lastVal2 = lastVal1;
        lastVal1 = tempDiff;
        if (CentY <= 900 && CentY >= 500)
            start(m2);
            m2.Speed = 0.175;
         
        elseif (CentY < 500)
            start(m3);
            m3.Speed = 0.25;
        elseif (CentY >= 1020 && CentY <=1420)
            start(m2);
            m2.Speed = -0.175;
         elseif (CentY > 1420)
            start(m3);
            m3.Speed = -0.25;
        end
        if (CentX<= 490)
            start(m5);
            m5.Speed = 0.25;

        elseif(CentX >= 590)
            start(m5);
            m5.Speed = -0.25;
        end
    end

end









%% Functions go below here:

% Function to find different between X values and Y values
function totalDiff = findXYDiff(loc, idx)
indXmin = min(loc(:,1, idx));
indYmin = min(loc(:,2, idx));
indXmax = max(loc(:,1, idx));
indYmax = max(loc(:,2, idx));
indXdif = indXmax - indXmin;
indYdif = indYmax - indYmin;
totalDiff = indYdif+indXdif;
end

%% Function to track the wrist bands using color thresholder %%

function [m1_X, m1_Y, m2_X, m2_Y] = wristTrack(RGB)
m1_X = 0; m1_Y = 0; m2_X = 0; m2_Y = 0;

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.468;
channel1Max = 0.557;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.278;
channel2Max = 0.578;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.380;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    %%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%%

    CC = bwconncomp(sliderBW);
    s = regionprops(CC,'Centroid','Area');
    centroids = cat(1,s.Centroid);
    areas = cat(1,s.Area);
    [m1,ind] = max(areas); % find the largest connected component
    if (m1 > 150)
    plot(centroids(ind,1),centroids(ind,2),'m*','markersize',32)
    m1_X = centroids(ind,1);
    m1_Y = centroids(ind,2);
    end
    centroids2 = centroids;
    area2=areas;
    area2(ind,:)=[];
    centroids2(ind,:)=[];
    [m2, ind2] = max(area2);
        if (m2 > 150)
     hold on
     plot(centroids2(ind2,1),centroids2(ind2,2),'m*','markersize',75)
     m2_X = centroids2(ind2,1);
     m2_Y = centroids2(ind2,2);
    end
end

%% starting position and hibernate position %%
function curl(direction, m2, m3, m4)
    start(m4);
    start(m3);
    start(m2);
    m4.Speed = (-1)*direction*0.175;
    m3.Speed = direction*0.25;
    m2.Speed = direction*0.25;
    pause(10);
    stop(m4);
    stop(m3);
    pause(1);
    stop(m2);
end