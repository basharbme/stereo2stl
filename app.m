%% Three-dimensional stereo reconstruction.
%> @file stereo2stl/app.m
%> @brief GUI for point cloud STL generation from stereo input.
%> @requires surf2stl
%> =======================================================================
%> @mainpage Three-dimensional stereo reconstruction.
%
%> @section introduction Introduction
%>
%> `stereo2stl` is a MATLAB generator script for stereo reconstruction.
%>
%> Input: landscape, JPG/JPEG, grayscale stereo images.
%>  - Add checkerboard images from left stereo view to `./calibration/left`
%>  - Add checkerboard images from right stereo view to `./calibration/right`
%>  - Add modelling input images from stereo view to `./input`
%>
%>    imformats() %  supported images formats in MATLAB
%>
%> Output: `pointCloud` file in script location (STL format).
%    - A 3D representation of the stereo image set.
%>
%> =======================================================================
%> @section image-inputs Image inputs
%>  - Images should be same orientation as checkerboard to reduce pixel
%>    error from reprojection.
%>  - For calibration images:
%>    - asymmetric (odd-even) checkerboard should be in all views
%>    - minimum image count per folder is 4 (for low reprojection error)
%>    - naming convention: `./calibration/<VIEW>/<VIEW>##.jpg`
%>     (e.g. `./calibration/left/left01.jpg`)
%>  - Image folders:
%>    - `./input`: actual 3D model target scene
%>    - `./calibration/left`: left stereo view w. checkerboard
%>    - `./calibration/right`: right stereo view w. checkerboard
%>
%> N.B: the image file names must be numbered in ascending order.
%>
%> The script targets the left & right image in the `./input` folder and
%> converts it into a STL file.
%>
%> =======================================================================

%> =======================================================================
%% Initialise generator script.
%> @section initialise-generator-script

%%
%  Cleanup of MATLAB workspace, IDE & output file.
close all;
clear;
clc;

%%
%  Show experimental data modal with 4 figures.
%>  Figure 1 - experimental camera setup
%>  Figure 2 - reprojection error graph
%>  Figure 3 - anaglyph image (3D, corrected image)
%>  Figure 4 - disparity map
figure2D = figure;
figure2D.Units = 'normalized';
movegui(figure2D, 'center');
figure2D.OuterPosition = [0 0 1 1];
rotate3d on;

%%
%  Show loading indicator. Script takes T = ~30s to execute.
loadingWaitbar = waitbar(0, 'Loading');
loadingWaitbar.Visible = 'on';

%> =======================================================================
%% Script variable configuration.
%> @section script-variable-configuration

%%
%  Load surf2stl if not installed.
if ~exist('surf2stl', 'file')
    if ~matlab.addons.isAddonEnabled('mpm')
        error([                                                             ...
            'Please install MPM as a MATLAB Addon.\n'                       ...
            '<a href="'                                                     ...
            'https://uk.mathworks.com/matlabcentral/fileexchange/54548'     ...
            '">'                                                            ...
            'mpm - File Exchange - MATLAB Central'                          ...
            '</a>'                                                          ...
        ])
    else
        mpm install surf2stl;
    end
end

%%
%  Acccessible modal prompt for easy configuration.
%> @var filePath path to current folder.
%> @var stlPath name of point cloud STL.
%> @var imageMinimum min. no of images in calib folder.
%> @var squareWidthMm Checkerboard square width in mm.
%> @var ptCloudDensity Point density within squareWidthMm.
%> @var sGolayFiltOrder Savitsky-Golay extrapolation curve order.
%> @var sGolayFiltFrameLen Savitsky-Golay sliding window point count.
filePath = pwd;
stlPath = 'point-cloud.stl';
imageMinimum = 3;
squareWidthMm = 50;
ptCloudDensity = 5;
sGolayFiltOrder = 2;
sGolayFiltFrameLen = 9;

dlgtitle = 'Three-dimensional stereo reconstruction';
prompt = {};
definput = {};

prompt{1} = 'Checkerboard square width (mm)';
definput{1} = num2str(squareWidthMm);
prompt{2} = 'Point cloud density within square width';
definput{2} = num2str(ptCloudDensity);
prompt{3} = 'Savitsky-Golay extrapolation curve order';
definput{3} = num2str(sGolayFiltOrder);
prompt{4} = 'Savitsky-Golay sliding window point count';
definput{4} = num2str(sGolayFiltFrameLen);

answer = inputdlg(prompt, dlgtitle, 1, definput);

if ~isnan(answer{2}); squareWidthMm = str2double(answer{2}); end
if ~isnan(answer{3}); ptCloudDensity = str2double(answer{3}); end
if ~isnan(answer{4}); sGolayFiltOrder = str2double(answer{4}); end
if ~isnan(answer{5}); sGolayFiltFrameLen = str2double(answer{5}); end

%> =======================================================================
%% Camera calibration of stereo images.
%> @section camera-calibration-of-stereo-images
%  Code based on MATLAB `rectifyStereoImages` code sample [1].

%%
%  Delete STL if it exists.
if exist(fullfile(filePath, stlPath), 'file')
    recycle on;
    delete(fullfile(filePath, stlPath));
end

%%
%  Load all of our images in one go.
%> @var inputImages Images from `./input` subfolder.
%> @var calibLeftImages Images from `./calibration/left` subfolder.
%> @var calibRightImages Images from `./calibration/right` subfolder.
%> @see #image-inputs
inputImages = imageDatastore(fullfile(filePath, 'input'));
calibLeftImages = imageDatastore(fullfile(filePath, 'calibration', 'left'));
calibRightImages = imageDatastore(fullfile(filePath, 'calibration', 'right'));
waitbar(0.1, loadingWaitbar);

%%
%  Detect the checkerboards in the original files.
%> @see #images
[imagePoints, boardSize] = detectCheckerboardPoints(                    ...
    calibLeftImages.Files,                                              ...
    calibRightImages.Files                                              ...
);
waitbar(0.2, loadingWaitbar);

%%
%  Calculate undistorted, real-world coordinates of checkerboard keypoints.
worldPoints = generateCheckerboardPoints(boardSize, squareWidthMm);

%%
%  Read input images into MATLAB, and convert to grayscale.
%  This reduces image data (& overhead) in calibration phase. [4]
I1 = readimage(inputImages, 1);
if size(I1, 3) == 3
    I1 = rgb2gray(I1);
end
I2 = readimage(inputImages, 2);
if size(I2, 3) == 3
    I2 = rgb2gray(I2);
end
imageSize = [size(I1, 1), size(I1, 2)];

%%
%  Image validation (for camera calibration).
%> Throws error if:
%>  - mismatch in image count between `./calibration/left` & `./calibration/right`
%>  - below 4 images in `./calibration/left` & `./calibration/right`
%>  - mismatch in resolution of `./input` images
imageSize2 = [size(I2, 1), size(I2, 2)];

imageAmounts = struct;
imageAmounts.L = size(calibLeftImages.Files, 1);
imageAmounts.R = size(calibRightImages.Files, 1);

if imageAmounts.L ~= imageAmounts.R      % error #1

    e = sprintf(                                                        ...
        'stereo2stl::ERR_MISMATCH_IMG_COUNT (L: %d, R: %d)',            ...
        imageAmounts.L, imageAmounts.R                                  ...
    );
    errordlg(e);
    error(e); %#ok<*SPERR>

elseif imageAmounts.L < imageMinimum    % error #2

    e = sprintf('stereo2stl::ERR_CALIB_IMG_INSUFFICIENT (%d)', imageAmounts.L);
    errordlg(e);
    error(e);

elseif ~isequal(imageSize, imageSize2)   % error #3

    e = sprintf(                                                        ...
        'stereo2stl::ERR_MISMATCH_IMG_DIM (L: %dx%dpx, R: %dx%dpx)',    ...
        imageSize(1), imageSize(2), imageSize2(1), imageSize2(2)        ...
    );
    errordlg(e);
    error(e);

end

%%
%  Use MATLAB toolbox to calibrate the stereo camera system.
%>  @param EstimateSkew Are image axes exactly perpendicular?
%>         Default: `true`.
%>  @param EstimateTangentialDistortion Factor in whether the camera is
%>         horizontal. Default: `true`.
%>  @param NumRadialDistortionCoefficients Good for fish-eye lenses.
%>         Default: `2`.
%>  @param ImageSize Matrix for size of image - `imageSize`.
%>  @todo  Adjust `estimateCameraParameters` parameters for experimental
%>         stage.
[stereoParams, ~, estimationErrors] = estimateCameraParameters(         ...
    imagePoints, worldPoints,                                           ...
    'EstimateSkew', true,                                               ...
    'EstimateTangentialDistortion', false                               ...
);
waitbar(0.3, loadingWaitbar);

%%
%  Display camera extrinisics.
%  Figure 1 & 2: locations & reprojection errors.
%  Reprojection is the process of "reprojecting" original image from a
%  camera image. Most camera images have distortion (e.g. "fisheye" lens
%  effect).
A1 = subplot(2, 2, 1);
showExtrinsics(stereoParams, 'CameraCentric');
waitbar(0.4, loadingWaitbar);
A2 = subplot(2, 2, 2);
showReprojectionErrors(stereoParams);
displayErrors(estimationErrors, stereoParams);
waitbar(0.5, loadingWaitbar);

%%
%  Rectify the images using "valid" output view.
%> The "valid" option is most suitable for computing disparity. [3]
%> @param OutputView crops the image to a rectangle, fitting inside the
%>        overlapping, curved 3D anaglyph. Default: `valid`.
[F1, F2] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView', 'valid');
pixelDensityMm = mrdivide(                                              ...
    mean([                                                              ...
        stereoParams.CameraParameters1.FocalLength,                     ...
        stereoParams.CameraParameters2.FocalLength                      ...
    ], 2),                                                              ...
    mean([                                                            ...
        stereoParams.CameraParameters1.IntrinsicMatrix(1, 1),           ...
        stereoParams.CameraParameters2.IntrinsicMatrix(1, 1)            ...
    ], 2)                                                               ...
);
approxImageHeight = 2 * mean([size(F1, 1), size(F2, 1)], 2) / pixelDensityMm;
approxImageWidth = 2 * mean([size(F1, 2), size(F2, 2)], 2) / pixelDensityMm;
waitbar(0.6, loadingWaitbar);
%%
%  Display an anaglyph image for "valid" output view.
A3 = subplot(2, 2, 3);
imshow(stereoAnaglyph(F1, F2));
title 'Rectified Image';
waitbar(0.7, loadingWaitbar);

%> =======================================================================
%% Disparity computation from stereo images.
%> Code based on MATLAB `disparity` code sample. [4]
%> @section disparity-computation-from-stereo-images

%%
%  Compute disparity map from stereo images (colormap of depth).
disparityMap = disparity(F1, F2);
waitbar(0.8, loadingWaitbar);

%%
% Display disparity map as jet colormap image.
% Figure 4 - disparity map.
A4 = subplot(2, 2, 4);
imshow(disparityMap, [0, 64]);
colormap(A4, jet);
title 'Disparity Map';
colorbar(A4);
waitbar(0.9, loadingWaitbar);

%> =======================================================================
%% Point cloud generation using depth data.
%> @section point-cloud-generation-using-depth-data

%%
%  Reconstruct the 3D image input into organised point cache
%  matrix.
%  Produces raw point cloud of X-Y-Z data in m - standard STL dimensions.
rawPoints3D = reconstructScene(disparityMap, stereoParams);
rawPoints3D(isinf(rawPoints3D)) = NaN;
rawPoints3D = double(rawPoints3D) ./ 1000;

%%
%  Initialise axial, co-ordinate cache for point cloud.
pointsCache = struct;
axesKeys = ['X', 'Y', 'Z'];
for m = 1:3
    k = char(axesKeys(m));
    p = rawPoints3D(:, :, m);
    pointsCache.(k) = p;
end
clearvars p k;

%%
%  Compute checkerboard position as a point cloud. It's the
%  closest set of co-ordinates to the origin in the z-axis.
%
%  @todo See if I need to change `min` in some way (assumes convex).
checkerboardCentroid.Z = min(min(pointsCache.Z));
checkerboardIndex = sort(find(checkerboardCentroid.Z == pointsCache.Z));
checkerboardCentroid.X = mean(pointsCache.X(checkerboardIndex));
checkerboardCentroid.Y = mean(pointsCache.Y(checkerboardIndex));

%%
%  Restrict point cloud to following conditions:
%   - point cloud width/2 ~ image height
%   - point cloud length = point cloud height = point cloud width
limits = struct;
cacheAxes = char(fieldnames(pointsCache));

for m = 1:3
    switch m
        case 1
            bound = approxImageWidth / 2;
        case 2
            bound = approxImageHeight / 2;
        case 3
            bound = mean([approxImageHeight, approxImageWidth], 2) / 2;
        otherwise
            bound = mean([approxImageHeight, approxImageWidth], 2) / 2;
    end
    k = cacheAxes(m);
    lim = [                                                 ...
        checkerboardCentroid.(k) - bound/1000,              ...
        checkerboardCentroid.(k) + bound/1000               ...
    ];
    limits.(k) = lim;

    p = pointsCache.(k);
    p(p < lim(1) | p > lim(2)) = NaN;
    pointsCache.(k) = p;
end

clearvars k lim p;

%%
%  Remove invalid (NaN) values inside point cloud.
%    - Values that are `+Inf`/`-Inf`/`NaN`.
%    - Points that fall outside range of point cloud.
nanPoints = ( 0                                                         ...
    | isnan(pointsCache.X)                                              ...
    | isnan(pointsCache.Y)                                              ...
    | isnan(pointsCache.Z)                                              ...
);

for m = 1:3
    k = cacheAxes(m);
    p = pointsCache.(k);
    p(nanPoints) = checkerboardCentroid.(k);
    pointsCache.(k) = p;
end

clearvars k p;

%%
%  Surface mesh denoising and interpolation.
%> Code adapted from StackOverflow. [5]
%>
%>  # The `scatteredInterpolant` factory function computes interpolant. [5]
%>  # MATLAB maps `meshgrid` regular matrix of points for X and Y.
%>  # Savitzky-Golay filter used to denoise points in Z axis.
%>
%> @see https://commons.wikimedia.org/wiki/File:Lissage_sg3_anim.gif
gs = (1 / ptCloudDensity) * (squareWidthMm / 1000);

I = scatteredInterpolant(pointsCache.X(:), pointsCache.Y(:), pointsCache.Z(:), 'natural');

gridPoints = struct;
intX = min(pointsCache.X(:)):gs:max(pointsCache.X(:));
intY = min(pointsCache.Y(:)):gs:max(pointsCache.Y(:));
[gridPoints.X, gridPoints.Y] = meshgrid(intX, intY);

gridPoints.Z = I(gridPoints.X, gridPoints.Y);
intZ1 = sgolayfilt(gridPoints.Z.', sGolayFiltOrder, sGolayFiltFrameLen);
intZ2 = sgolayfilt(gridPoints.Z, sGolayFiltOrder, sGolayFiltFrameLen);
gridPoints.Z = (intZ1.' + intZ2)/2;

points3D = double.empty();
for m = 1:3
    points3D(:, :, m) = gridPoints.(cacheAxes(m));
end

clearvars cacheAxes;

%%
%  Convert 3D points to point cloud.
ptCloud = pointCloud(points3D);

%%
%  Launch rotatable, 3D point cloud viewer.
%  Figure 5 - scattered point cloud.
figure;
figure3D = pcshow(                                                      ...
    ptCloud,                                                            ...
    'VerticalAxis', 'y',                                                ...
    'VerticalAxisDir', 'Down',                                          ...
    'MarkerSize', 12                                                    ...
);
title '\color{black} Point Cloud';
movegui(figure3D, 'center');
figure3D.OuterPosition = [0 0 1 1];

xlabel 'x (horizontal displacement in m)';
ylabel 'y (vertical displacement in m)';
zlabel 'z (point depth in m)';

view(0, -90);
set(gcf, 'Color', 'w');
set(gca, 'XColor', 'k');
set(gca, 'YColor', 'k');
set(gca, 'ZColor', 'k');

colormap(flipud(jet));
caxis(zlim(figure3D));
colorbar(figure3D);
rotate3d on;

waitbar(1, loadingWaitbar);
close(loadingWaitbar);

%> =======================================================================
%% STL file generation from point cloud.
%  @section stl-file-generation-from-point-cloud
%  Using surf2stl for high stability & speed (low interpolation).
stlPath = char(fullfile(filePath, stlPath));
surf2stl(stlPath, gridPoints.X, gridPoints.Y, gridPoints.Z);
winopen(stlPath);

%> =======================================================================
%% References.
%> @section references
%>  1. https://www.mathworks.com/help/matlab/ref/rgb2gray.html
%>  2. https://www.mathworks.com/help/vision/examples/depth-estimation-from-stereo-video.html
%>  3. https://www.mathworks.com/help/vision/ref/rectifystereoimages.html
%>  4. https://www.mathworks.com/help/vision/ref/disparitysgm.html
%>  5. https://stackoverflow.com/a/39576639
%>  6. https://www.mathworks.com/help/matlab/ref/scatteredinterpolant.html
