%% Initializes Necessary Components 

videoOutput = VideoWriter('blendedVideoPicture');
open(videoOutput);

% APPLE IS READER 1
videoFileReader1 = vision.VideoFileReader('boylans_hi.mp4');
% ORANGE IS READER 2
videoFileReader2 = vision.VideoFileReader('stella_hi.mp4');

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

%% Track the Object
offset = 5;
vid2_x1 = round(min(points2(:,1)));
vid2_x2 = round(max(points2(:,1)));
vid2_mid = round((vid2_x1 + vid2_x2) / 2);
vid2_y1 = round(min(points2(:,2)));
vid2_y2 = round(max(points2(:,2)));
      
% make sure the extraction with offset is within bounds
% offset is used to blend the surroundings of the desired objects
vid2_x1 = max(1, vid2_mid - offset);
vid2_x2 = min(size(frame2, 2), vid2_x2 + offset);
vid2_y1 = max(1, vid2_y1 - offset);
vid2_y2 = min(size(frame2, 1), vid2_y2 + offset);
      
while ~isDone(videoFileReader1)

      frame1 = step(videoFileReader1);
      frame1 = imresize(frame1,.4);
      [points1, validity1] = step(tracker1, frame1);
      points1 = points1(validity1,:);
      
      % video 1 bounding box
      vid1_x1 = round(min(points1(:,1)));
      vid1_x2 = round(max(points1(:,1)));
      vid1_mid = round((vid1_x1 + vid1_x2) / 2);
      vid1_y1 = round(min(points1(:,2)));
      vid1_y2 = round(max(points1(:,2)));
      
      % scale the extraction to the same height as the blending object
      extraction = frame2(vid2_y1:vid2_y2, vid2_x1:vid2_x2, :);
      scale = (vid1_y2 - vid1_y1 + 2 * offset) / (vid2_y2 - vid2_y1);
      extraction = imresize(extraction, scale);

      % temp_im is the extraction placed in the right frame position
      temp_im = zeros(size(frame1));
      x_align = vid1_mid - offset;
      y_align = vid1_y1 - offset;
      temp_im(y_align:y_align+size(extraction,1)-1, x_align:x_align+size(extraction,2)-1, :) = extraction;
      
      % create mask with a different cut at each iteration
      mask = ones(size(frame1,1),size(frame1,2));
      objXSize = round(size(extraction, 2) - 2 * offset * scale);
      mask(vid1_y1:vid1_y2, vid1_mid:vid1_mid+objXSize) = 0;
      
      blended_im = BlendImages(im2double(frame1),temp_im,mask);
      blended_im(blended_im > 1) = 1;
      blended_im(blended_im < 0) = 0; 
      writeVideo(videoOutput, blended_im);
      
end
%% Clean Up Objects
close(videoOutput);
release(videoFileReader1);
release(videoFileReader2);