%% Rectify Stereo Images
%%
% Specify images containing a checkerboard for calibration.
imageDir = fullfile(toolboxdir('vision'),'visiondata', ...
    'calibration','stereo');
leftImages = imageDatastore(fullfile(imageDir,'left'));
rightImages = imageDatastore(fullfile(imageDir,'right'));
%%
% Detect the checkerboards.
[imagePoints,boardSize] = detectCheckerboardPoints(...
    leftImages.Files,rightImages.Files);
%%
% Specify world coordinates of checkerboard keypoints.
squareSizeInMillimeters = 108;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMillimeters);
%%
% Read in the images.
I1 = readimage(leftImages,1);
I2 = readimage(rightImages,1);
imageSize = [size(I1,1),size(I1,2)];
%%
% Calibrate the stereo camera system.
stereoParams = estimateCameraParameters(imagePoints,worldPoints, ...
                                        'ImageSize',imageSize);
%%
% Rectify the images using 'full' output view.
[J1_full,J2_full] = rectifyStereoImages(I1,I2,stereoParams, ...
  'OutputView','full');
%%
% Display the result for 'full' output view.
figure; 
imshow(stereoAnaglyph(J1_full,J2_full));
%%
% Rectify the images using 'valid' output view. This is most suitable
% for computing disparity.
[J1_valid,J2_valid] = rectifyStereoImages(I1,I2,stereoParams, ...
  'OutputView','valid');
%%
% Display the result for 'valid' output view.
figure; 
imshow(stereoAnaglyph(J1_valid,J2_valid));
  
