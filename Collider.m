close all;
videoOutput = VideoWriter('vid_collide');
open(videoOutput);

% APPLE IS READER 1
videoFileReader1 = vision.VideoFileReader('dway_sidestep1.m4v');
%videoPlayer1 = vision.VideoPlayer('Position', [100, 100, 680, 520]);


% ORANGE IS READER 2
videoFileReader2 = vision.VideoFileReader('alex_sidestep1.m4v');
%videoPlayer2 = vision.VideoPlayer('Position', [100, 100, 680, 520]);

%{
% get average pixles for each video
sum_frames = step(videoFileReader1);
num_frames = 1;
while ~isDone(videoFileReader1)
    frame = step(videoFileReader1);
    sum_frames = sum_frames + frame;
    num_frames = num_frames +1;
end
average_frame1 = sum_frames ./ num_frames;

disp('done with 1');

sum_frames = step(videoFileReader2);
num_frames = 1;
while ~isDone(videoFileReader2)
    frame = step(videoFileReader2);
    sum_frames = sum_frames + frame;
    num_frames = num_frames +1;
end
average_frame2 = sum_frames ./ num_frames;

disp('done with 2');
%}

% go to good spots in each video

% APPLE IS READER 1
%videoFileReader1 = vision.VideoFileReader('hand1.mp4');
%videoPlayer1 = vision.VideoPlayer('Position', [100, 100, 680, 520]);

start = 80;
for i = 1:start
    ObjectFrame1 = step(videoFileReader1);
end

drawnow;
% ORANGE IS READER 2
%videoFileReader2 = vision.VideoFileReader('hand2.mp4');
%videoPlayer2 = vision.VideoPlayer('Position', [100, 100, 680, 520]);
for i = 1:120
    ObjectFrame2 = step(videoFileReader2);
end
%}
%{
figure(1);
imshow(ObjectFrame1);
figure(2);

imshow(ObjectFrame2);
drawnow;
%}
disp('starting to blur');
count = 0;
threshold = .3;
% do segmentation for current frames
% get cur centroid
frame1 = step(videoFileReader1);
cur_frame = zeros(size(frame1));
while ~isDone(videoFileReader1) && ~isDone(videoFileReader2)
    frame1 = step(videoFileReader1);
    frame2 = step(videoFileReader2);
    
    blobs1 = abs(average_frame1 - frame1); 
    blobs1 = sum(blobs1,3) > threshold;
    SE = strel('rectangle', [20, 15]);
    blobs1 = imdilate(blobs1, SE);
    blobs1 = imerode(blobs1, SE);
    blobs1 = imerode(blobs1, SE);
    blobs1 = imdilate(blobs1, SE);
    blobs1 = imdilate(blobs1, SE);
    %[c,d,e] = find(blobs1==1);
    %cur_frame(c,d,:) = frame1(c,d,:);
    blobs2 = zeros(size(frame1));
    blobs2(:,:,1) = blobs1;
    blobs2(:,:,2) = blobs1;
    blobs2(:,:,3) = blobs1;
	cur_frame = frame1 .* blobs2;
    %(videoPlayer1, cur_frame);
    SE = strel('rectangle', [10,10]);
    blobs1 = imerode(blobs1,SE);

    blended_im = BlendImages(im2double(cur_frame), im2double(frame2), +blobs1);
    blended_im(blended_im > 1) = 1;
    blended_im(blended_im <0 ) = 0;
    writeVideo(videoOutput, blended_im);

    %cur_frame(blobs1==1)=0;
    count = count +1;
    disp('done with frame');
    disp(count);
end
%}
close(videoOutput);
release(videoFileReader1);
release(videoFileReader2);
