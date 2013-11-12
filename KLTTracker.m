
%% Initializes Necessary Components 
videoFileReader = vision.VideoFileReader('apple.mp4');
videoPlayer = vision.VideoPlayer('Position', [100, 100, 680, 520]);

objectFrame = step(videoFileReader);
figure; imshow(objectFrame);
objectRegion=round(getPosition(imrect))


%% Shows the Bounding Box 
%objectImage = insertShape(objectFrame, 'Rectangle', objectRegion,'Color', 'red'); 
%figure; imshow(objectImage); title('Yellow box shows object region');

%% Get Points of Interest and Mark Them
points = detectMinEigenFeatures(rgb2gray(objectFrame), 'ROI', objectRegion);
pointImage = insertMarker(objectFrame, points.Location, '+', 'Color', 'white');
figure, imshow(pointImage), title('Detected interest points');

markerInserter = vision.MarkerInserter('Shape','Plus','BorderColor','White');
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, points.Location, objectFrame);

%% Track the Object
while ~isDone(videoFileReader)
      frame = step(videoFileReader);
      [points, validity] = step(tracker, frame);
      out = insertMarker(frame, points(validity, :), '+');
      % you can comment out this line if you don't want the videoplayer
      step(videoPlayer, out);
end

%% Clean Up Objects
release(videoPlayer);
release(videoFileReader);