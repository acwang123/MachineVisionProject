close all;
%% Initializes Necessary Components 

videoOutput = VideoWriter('wipeTransition');
open(videoOutput);

% APPLE IS READER 1
videoFileReader1 = vision.VideoFileReader('apple.mp4');
vidObj = VideoReader('apple.mp4');
numFrames = vidObj.NumberOfFrames;
% ORANGE IS READER 2
videoFileReader2 = vision.VideoFileReader('test.mp4');

objectFrame1 = step(videoFileReader1);
objectFrame1 = imresize(objectFrame1, .4);
figure; imshow(objectFrame1);
objectRegion1 = round(getPosition(imrect));

frame2 = step(videoFileReader2);
frame2 = imresize(frame2, .4);
frame2 = im2double(frame2);
figure; imshow(frame2);
objectRegion2 = round(getPosition(imrect));

%% Get Points of Interest and Mark Them
points1 = detectMinEigenFeatures(rgb2gray(objectFrame1), 'ROI', objectRegion1);
pointImage1 = insertMarker(objectFrame1, points1.Location, '+', 'Color', 'white');
figure, imshow(pointImage1), title('Detected interest points');

tracker1 = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker1, points1.Location, objectFrame1);

points2 = detectMinEigenFeatures(rgb2gray(frame2), 'ROI', objectRegion2);
points2 = points2.Location;
pointImage2 = insertMarker(frame2, points2, '+', 'Color', 'white');
figure, imshow(pointImage2), title('Detected interest points');

%tracker2 = vision.PointTracker('MaxBidirectionalError', 1);
%initialize(tracker2, points2.Location, objectFrame2);

%% Track the Object
increment = -1;
counter = 0;
cutPosition = 0;
offset = 20;
vid2_x1 = round(min(points2(:,1)));
vid2_x2 = round(max(points2(:,1)));
vid2_mid = round((vid2_x1 + vid2_x2) / 2);
vid2_y1 = round(min(points2(:,2)));
vid2_y2 = round(max(points2(:,2)));
      
% make sure the extraction with offset is within bounds
vid2_x1 = max(1, vid2_x1 - offset);
vid2_x2 = min(size(frame2, 2), vid2_x2 + offset);
vid2_y1 = max(1, vid2_y1 - offset);
vid2_y2 = min(size(frame2, 1), vid2_y2 + offset);
      
while ~isDone(videoFileReader1)% && ~isDone(videoFileReader2)
%for x = 1:1

      counter = counter + 1;
      if counter == increment
          counter = 0;
          cutPosition = min(cutPosition + 1, vid1_x2 - vid1_x1);
      end

      frame1 = step(videoFileReader1);
      frame1 = imresize(frame1,.4);
      [points1, validity1] = step(tracker1, frame1);
      points1 = points1(validity1,:);
      
      %frame2 = step(videoFileReader2);
      %frame2 = imresize(frame2, .4);
      %[points2, validity2] = step(tracker2, frame2);
      %points2 = points2(validity2,:);
      
      % video 1 bounding box
      vid1_x1 = round(min(points1(:,1)));
      vid1_x2 = round(max(points1(:,1)));
      vid1_y1 = round(min(points1(:,2)));
      %vid1_y2 = round(max(points1(:,2)));
      
      if increment == -1
          increment = floor(numFrames / (vid1_x2 - vid1_x1));
      end

      % scale the extraction to the same width as the blending object
      extraction = frame2(vid2_y1:vid2_y2, vid2_x1:vid2_x2, :);
      scale = (vid1_x2 - vid1_x1 + 2 * offset) / (vid2_x2 - vid2_x1);
      extraction = imresize(extraction, scale);

      % temp_im is the extraction placed in the right frame position
      temp_im = zeros(size(frame1));
      x_align = vid1_x1 - offset;
      y_align = vid1_y1 - offset;
      temp_im(y_align:y_align+size(extraction,1)-1, x_align:x_align+size(extraction,2)-1, :) = extraction;
      
      % create mask with a different cut at each iteration
      mask = ones(size(frame1,1),size(frame1,2));
      objYSize = round(size(extraction, 1) - 2 * offset * scale);
      mask(vid1_y1:vid1_y1+objYSize, vid1_x1:vid1_x1+cutPosition) = 0;
      
      blended_im = BlendImages(im2double(frame1),temp_im,mask);
      blended_im(blended_im > 1) = 1;
      %figure; imshow(blended_im);
      writeVideo(videoOutput, blended_im);
      
end
close(videoOutput);
%% Clean Up Objects
%release(videoPlayer1);
release(videoFileReader1);
%release(videoPlayer2);
release(videoFileReader2);