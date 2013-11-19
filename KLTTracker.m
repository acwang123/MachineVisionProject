close all;
%% Initializes Necessary Components 

new_vid_object = VideoWriter('vid_output');
open(new_vid_object);

% APPLE IS READER 1
videoFileReader1 = vision.VideoFileReader('apple.mp4');
%videoPlayer1 = vision.VideoPlayer('Position', [100, 100, 680, 520]);
size(videoFileReader1)
%videoFileReader1.NumberOfFrames
% ORANGE IS READER 2
videoFileReader2 = vision.VideoFileReader('test.mp4');
%videoPlayer2 = vision.VideoPlayer('Position', [100, 100, 680, 520]);

objectFrame1 = step(videoFileReader1);
objectFrame1 = imresize(objectFrame1, .4);
figure; imshow(objectFrame1);
%objectRegion1 = round(getPosition(imrect))

objectFrame2 = step(videoFileReader2);
objectFrame2 = imresize(objectFrame2, .4);
figure; imshow(objectFrame2);
%objectRegion2 = round(getPosition(imrect))


%% Shows the Bounding Box 
%objectImage = insertShape(objectFrame, 'Rectangle', objectRegion,'Color', 'red'); 
%figure; imshow(objectImage); title('Yellow box shows object region');

%% Get Points of Interest and Mark Them
points1 = detectMinEigenFeatures(rgb2gray(objectFrame1), 'ROI', objectRegion1);
pointImage1 = insertMarker(objectFrame1, points1.Location, '+', 'Color', 'white');
figure, imshow(pointImage1), title('Detected interest points');

markerInserter1 = vision.MarkerInserter('Shape','Plus','BorderColor','White');
tracker1 = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker1, points1.Location, objectFrame1);

points2 = detectMinEigenFeatures(rgb2gray(objectFrame2), 'ROI', objectRegion2);
pointImage2 = insertMarker(objectFrame2, points2.Location, '+', 'Color', 'white');
figure, imshow(pointImage2), title('Detected interest points');

markerInserter2 = vision.MarkerInserter('Shape','Plus','BorderColor','White');
tracker2 = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker2, points2.Location, objectFrame2);

%% Track the Object
%while ~isDone(videoFileReader)
while ~isDone(videoFileReader1) && ~isDone(videoFileReader2)
      frame1 = step(videoFileReader1);
      frame1 = imresize(frame1,.4);
      [points1, validity1] = step(tracker1, frame1);
      out1 = insertMarker(frame1, points1(validity1, :), '+');
      % you can comment out this line if you don't want the videoplayer
      %step(videoPlayer1, out1);
      
      frame2 = step(videoFileReader2);
      frame2 = imresize(frame2, .4);
      [points2, validity2] = step(tracker2, frame2);
      out2 = insertMarker(frame2, points2(validity2, :), '+');
      points1 = points1(validity1,:);
      points2 = points2(validity2,:);
      % you can comment out this line if you don't want the videoplayer
      %step(videoPlayer2, out2);
      
      %{
      get bounding boxes
      extract larger box from right image
      make new image with larger box in the correct position
      make mask
      blur images
      %}
      vid1_x1 = round(min(points1(:,1)));
      vid1_x2 = round(max(points1(:,1)));
      vid1_mid = round((vid1_x1 + vid1_x2) /2);
      vid1_y1 = round(min(points1(:,2)));
      vid1_y2 = round(max(points1(:,2)));
      
      
      vid2_x1 = round(min(points2(:,1)));
      vid2_x2 = round(max(points2(:,1)));
      vid2_mid = round((vid2_x1 + vid2_x2) / 2);
      vid2_y1 = round(min(points2(:,2)));
      vid2_y2 = round(max(points2(:,2)));
      
      offset = 20;
      vid2_x1 = max(1, vid2_mid - offset);
      x_align = vid1_x2 - (vid2_x2 - vid2_x1);
      x_align = vid1_mid - offset;
      vid2_x2 = min(size(frame2, 2), vid2_x2 + offset);
      vid2_y1 = max(1, vid2_y1 - offset);
      y_align = vid1_y1 - offset;
      vid2_y2 = min(size(frame2, 1), vid2_y2 + offset);
      extraction = frame2( vid2_y1:vid2_y2,vid2_x1:vid2_x2,:);

      extraction_height = vid2_y2 - vid2_y1;
      frame1_height = vid1_y2 - vid1_y1;
      scale = (frame1_height+(2*offset)) / (extraction_height);
      extraction = imresize(extraction, scale);

      
      temp_im = zeros(size(frame1));
      temp_im( y_align:y_align+size(extraction,1)-1,x_align:x_align+size(extraction,2)-1, :) = extraction;

      mask = ones(size(frame1,1),size(frame1,2));
      mask( y_align+offset:y_align+size(extraction,1)-offset,x_align+offset:x_align+size(extraction,2)-offset) = 0;
      blended_im = BlendImages(im2double(frame1),temp_im,mask);
      %figure;
      %imshow(blended_im);
      writeVideo(new_vid_object, blended_im);
      
end
close(new_vid_object);
%% Clean Up Objects
%release(videoPlayer1);
release(videoFileReader1);
%release(videoPlayer2);
release(videoFileReader2);