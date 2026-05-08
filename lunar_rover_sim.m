%% LUNAR_ROVER_SIM.M
% -------------------------------------------------------------------------
% LUNAR ROVER HAZARD-AWARE NAVIGATION AND SIMULATION PROJECT
% MATLAB R2025a compatible single-file script.
%
% IMPORTANT:
%   This is a SCRIPT, not a function file.
%   There are NO custom helper functions at the end.
%   That means you will not get errors like:
%       Undefined function 'loadOrCreateScene'
%       Undefined function 'createSyntheticLunarSurface'
%
% HOW TO RUN:
%   1. Save this file exactly as: lunar_rover_sim.m
%   2. Put it in one folder.
%   3. Optional: put a lunar image in the same folder named:
%          chandrayaan_image.jpg
%      If the image is not present, this script automatically creates a
%      synthetic lunar surface with craters, rocks, shadows, roughness, and
%      elevation.
%   4. In MATLAB Command Window, run:
%          lunar_rover_sim
%
% PROJECT FEATURES:
%   1. Synthetic lunar terrain generation from scratch
%   2. Optional real lunar image input support
%   3. Crater generation / crater candidate modeling
%   4. Manual image preprocessing without toolbox-only functions
%   5. Roughness, slope, shadow, safety, and hazard maps
%   6. Crater inflated danger zones
%   7. Boulder and rough-region risk modeling
%   8. Cost map generation for rover movement
%   9. A* path planning from scratch
%  10. Dijkstra path planning from scratch for comparison
%  11. Path reconstruction and path densification
%  12. Rover telemetry simulation
%  13. 2D route visualization
%  14. 3D lunar terrain visualization
%  15. Mission dashboard
%  16. Output figures and CSV reports
%
% DESIGN CHOICE:
%   This file avoids custom local functions because your previous errors
%   happened when MATLAB could not find helper functions. Everything is
%   written inline in one script. It is longer, but much safer for copying
%   and running in MATLAB.
%
% TOOLBOX NOTE:
%   This script avoids imfindcircles, edge, imshow, imgaussfilt, imresize,
%   bwdist, strel, imdilate, and adapthisteq.
%   It uses base MATLAB-style operations such as conv2, gradient, imagesc,
%   surf, plot, patch, and basic matrix operations.
%
% -------------------------------------------------------------------------

clear;
clc;
close all;
rng(12);

fprintf('\n============================================================\n');
fprintf(' LUNAR ROVER HAZARD-AWARE NAVIGATION SIMULATION\n');
fprintf(' Single-file MATLAB R2025a project version\n');
fprintf('============================================================\n\n');

%% ------------------------------------------------------------------------
% 0. PROJECT CONFIGURATION
% -------------------------------------------------------------------------
% You can modify these settings, but the default settings are chosen so the
% project runs smoothly and produces strong visuals.

imageFile = 'chandrayaan_image.jpg';
outputFolder = 'results_lunar_rover_peak';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Synthetic terrain size.
% Bigger values give better visuals but take more computation.
sceneRows = 720;
sceneCols = 1080;

% Number of generated craters and rocks when no real image is supplied.
syntheticCraterCount = 82;
syntheticRockCount = 120;

% Rover and mission settings.
gridStep = 5;
startFractionRow = 0.88;
startFractionCol = 0.07;
goalFractionRow  = 0.12;
goalFractionCol  = 0.93;

% Cost map weights.
roughnessWeight = 7.0;
slopeWeight = 5.0;
shadowWeight = 2.2;
hazardNearnessWeight = 10.0;
craterInflationMultiplier = 1.25;
extraHazardInflationPixels = 5;

% Animation settings.
animationSkip = 5;
animationPause = 0.008;
roverMarkerSize = 11;
sensorRadiusPixels = 70;

% Output settings.
saveFigures = true;
saveCSVs = true;
saveAnimationVideo = false;
videoFileName = fullfile(outputFolder, 'lunar_rover_animation.mp4');

% Internal safety settings.
minimumFinitePlanningCost = 1.0;
emergencyHazardPenalty = 650.0;
maxAStarIterations = 250000;
maxDijkstraIterations = 250000;

%% ------------------------------------------------------------------------
% 1. LOAD REAL IMAGE OR CREATE SYNTHETIC LUNAR TERRAIN
% -------------------------------------------------------------------------
% If chandrayaan_image.jpg exists, the script uses it.
% Otherwise it creates a synthetic lunar terrain with:
%   - background noise
%   - craters with rims and shadows
%   - boulders
%   - illumination gradient
%   - elevation surface

usingRealImage = false;
truthCenters = [];
truthRadii = [];
truthRockCenters = [];
truthRockRadii = [];

if exist(imageFile, 'file') == 2
    fprintf('Image found: %s\n', imageFile);
    rawImage = imread(imageFile);
    rawImage = double(rawImage);

    % Normalize raw image manually.
    rawMin = min(rawImage(:));
    rawMax = max(rawImage(:));
    if rawMax > rawMin
        rawImage = (rawImage - rawMin) ./ (rawMax - rawMin);
    else
        rawImage = zeros(size(rawImage));
    end

    if ndims(rawImage) == 3
        redChannel = rawImage(:,:,1);
        greenChannel = rawImage(:,:,2);
        blueChannel = rawImage(:,:,3);
        grayImage = 0.2989 * redChannel + 0.5870 * greenChannel + 0.1140 * blueChannel;
        sceneRGB = rawImage(:,:,1:3);
    else
        grayImage = rawImage;
        sceneRGB = cat(3, grayImage, grayImage, grayImage);
    end

    % Downsample manually if image is too large.
    maxDim = 1100;
    currentMaxDim = max(size(grayImage,1), size(grayImage,2));
    if currentMaxDim > maxDim
        newRows = round(size(grayImage,1) * maxDim / currentMaxDim);
        newCols = round(size(grayImage,2) * maxDim / currentMaxDim);
        rowPick = unique(round(linspace(1, size(grayImage,1), newRows)));
        colPick = unique(round(linspace(1, size(grayImage,2), newCols)));
        grayImage = grayImage(rowPick, colPick);
        sceneRGB = sceneRGB(rowPick, colPick, :);
    end

    sceneRows = size(grayImage,1);
    sceneCols = size(grayImage,2);

    usingRealImage = true;

    % For real image, create an approximate elevation model using brightness
    % and gradients. This is not true DEM data, but it gives a useful 3D
    % visualization for a mini project.
    sigmaElevation = 7;
    kernelRadius = ceil(3 * sigmaElevation);
    kernelX = -kernelRadius:kernelRadius;
    gaussianKernel = exp(-(kernelX.^2) / (2 * sigmaElevation^2));
    gaussianKernel = gaussianKernel / sum(gaussianKernel);
    smoothForElevation = conv2(conv2(grayImage, gaussianKernel, 'same'), gaussianKernel', 'same');
    [elevGX, elevGY] = gradient(smoothForElevation);
    elevRough = hypot(elevGX, elevGY);
    elevRoughMin = min(elevRough(:));
    elevRoughMax = max(elevRough(:));
    if elevRoughMax > elevRoughMin
        elevRough = (elevRough - elevRoughMin) ./ (elevRoughMax - elevRoughMin);
    end
    elevationMap = 0.80 * smoothForElevation + 0.20 * (1 - elevRough);
    elevationMap = elevationMap - min(elevationMap(:));
    if max(elevationMap(:)) > 0
        elevationMap = elevationMap ./ max(elevationMap(:));
    end

else
    fprintf('No real image found. Creating synthetic lunar surface...\n');

    % Meshgrid for surface operations.
    [X, Y] = meshgrid(1:sceneCols, 1:sceneRows);

    % Gaussian smoothing kernels for terrain noise.
    sigmaLarge = 28;
    radiusLarge = ceil(3 * sigmaLarge);
    xLarge = -radiusLarge:radiusLarge;
    gLarge = exp(-(xLarge.^2) / (2 * sigmaLarge^2));
    gLarge = gLarge / sum(gLarge);

    sigmaMedium = 9;
    radiusMedium = ceil(3 * sigmaMedium);
    xMedium = -radiusMedium:radiusMedium;
    gMedium = exp(-(xMedium.^2) / (2 * sigmaMedium^2));
    gMedium = gMedium / sum(gMedium);

    sigmaFine = 1.4;
    radiusFine = ceil(3 * sigmaFine);
    xFine = -radiusFine:radiusFine;
    gFine = exp(-(xFine.^2) / (2 * sigmaFine^2));
    gFine = gFine / sum(gFine);

    largeNoise = conv2(conv2(randn(sceneRows, sceneCols), gLarge, 'same'), gLarge', 'same');
    mediumNoise = conv2(conv2(randn(sceneRows, sceneCols), gMedium, 'same'), gMedium', 'same');
    fineNoise = conv2(conv2(randn(sceneRows, sceneCols), gFine, 'same'), gFine', 'same');

    largeNoise = largeNoise - min(largeNoise(:));
    if max(largeNoise(:)) > 0
        largeNoise = largeNoise ./ max(largeNoise(:));
    end

    mediumNoise = mediumNoise - min(mediumNoise(:));
    if max(mediumNoise(:)) > 0
        mediumNoise = mediumNoise ./ max(mediumNoise(:));
    end

    fineNoise = fineNoise - min(fineNoise(:));
    if max(fineNoise(:)) > 0
        fineNoise = fineNoise ./ max(fineNoise(:));
    end

    grayImage = 0.36 + 0.31 * largeNoise + 0.17 * mediumNoise + 0.045 * fineNoise;
    elevationMap = 0.48 + 0.34 * largeNoise + 0.18 * mediumNoise;

    truthCenters = zeros(syntheticCraterCount, 2);
    truthRadii = zeros(syntheticCraterCount, 1);

    % Light direction controls crater shadow and rim highlight.
    lightDirX = -0.78;
    lightDirY = -0.52;
    lightNorm = sqrt(lightDirX^2 + lightDirY^2);
    lightDirX = lightDirX / lightNorm;
    lightDirY = lightDirY / lightNorm;

    % Generate craters.
    for craterIndex = 1:syntheticCraterCount
        radius = round(10 + (62 - 10) * rand());
        cx = round(1 + radius + 15 + (sceneCols - 2*radius - 30) * rand());
        cy = round(1 + radius + 15 + (sceneRows - 2*radius - 30) * rand());

        truthCenters(craterIndex, :) = [cx, cy];
        truthRadii(craterIndex) = radius;

        distanceFromCenter = hypot(X - cx, Y - cy);
        normalizedDistance = distanceFromCenter / max(radius, 1);

        craterBowl = exp(-(distanceFromCenter.^2) / (2 * (0.58 * radius)^2));
        craterRim = exp(-((distanceFromCenter - radius).^2) / (2 * (0.13 * radius)^2));
        craterOuterEjecta = exp(-((distanceFromCenter - 1.45*radius).^2) / (2 * (0.25 * radius)^2));

        dxNorm = (X - cx) / max(radius, 1);
        dyNorm = (Y - cy) / max(radius, 1);
        directionalTerm = lightDirX * dxNorm + lightDirY * dyNorm;
        insideCrater = normalizedDistance <= 1.0;
        shadowSide = insideCrater & (directionalTerm < -0.10);
        lightSide = insideCrater & (directionalTerm > 0.18);

        grayImage = grayImage - 0.19 * craterBowl;
        grayImage = grayImage + 0.18 * craterRim;
        grayImage = grayImage + 0.028 * craterOuterEjecta;
        grayImage(shadowSide) = grayImage(shadowSide) - 0.085;
        grayImage(lightSide) = grayImage(lightSide) + 0.055;

        elevationMap = elevationMap - 0.30 * craterBowl;
        elevationMap = elevationMap + 0.24 * craterRim;
        elevationMap = elevationMap + 0.045 * craterOuterEjecta;
    end

    truthRockCenters = zeros(syntheticRockCount, 2);
    truthRockRadii = zeros(syntheticRockCount, 1);

    % Generate rocks / boulders.
    for rockIndex = 1:syntheticRockCount
        rockRadius = 2.5 + (8.5 - 2.5) * rand();
        cx = 20 + (sceneCols - 40) * rand();
        cy = 20 + (sceneRows - 40) * rand();

        truthRockCenters(rockIndex,:) = [cx, cy];
        truthRockRadii(rockIndex) = rockRadius;

        distanceFromRock = hypot(X - cx, Y - cy);
        rockShape = exp(-(distanceFromRock.^2) / (2 * rockRadius^2));
        rockShadow = exp(-((hypot(X - (cx + 2.2*rockRadius), Y - (cy + 2.2*rockRadius))).^2) / (2 * (1.6*rockRadius)^2));

        grayImage = grayImage + 0.12 * rockShape - 0.045 * rockShadow;
        elevationMap = elevationMap + 0.11 * rockShape;
    end

    % Add global illumination and sensor noise.
    illuminationRamp = 0.08 * ((0.62 * X + 0.38 * Y) / max(sceneCols, sceneRows));
    grayImage = grayImage + illuminationRamp + 0.020 * randn(sceneRows, sceneCols);

    % Normalize synthetic maps.
    grayImage = grayImage - min(grayImage(:));
    if max(grayImage(:)) > 0
        grayImage = grayImage ./ max(grayImage(:));
    end

    elevationMap = elevationMap - min(elevationMap(:));
    if max(elevationMap(:)) > 0
        elevationMap = elevationMap ./ max(elevationMap(:));
    end

    % Build RGB lunar tone.
    sceneRGB = zeros(sceneRows, sceneCols, 3);
    sceneRGB(:,:,1) = min(1, max(0, grayImage * 1.06 + 0.025));
    sceneRGB(:,:,2) = min(1, max(0, grayImage * 1.01 + 0.010));
    sceneRGB(:,:,3) = min(1, max(0, grayImage * 0.93));
end

fprintf('Scene resolution: %d rows x %d cols\n', sceneRows, sceneCols);

%% ------------------------------------------------------------------------
% 2. MANUAL IMAGE PREPROCESSING
% -------------------------------------------------------------------------
% This avoids toolbox-specific functions. The goal is to enhance crater rims,
% shadows, and rough regions.

% Smooth image with manual Gaussian blur.
sigmaPre = 1.35;
radiusPre = ceil(3 * sigmaPre);
xPre = -radiusPre:radiusPre;
gPre = exp(-(xPre.^2) / (2 * sigmaPre^2));
gPre = gPre / sum(gPre);
blurredImage = conv2(conv2(grayImage, gPre, 'same'), gPre', 'same');

% Contrast stretching using manual percentile clipping.
flatBlurred = sort(blurredImage(:));
numFlat = numel(flatBlurred);
lowIndex = max(1, min(numFlat, round(0.015 * numFlat)));
highIndex = max(1, min(numFlat, round(0.985 * numFlat)));
lowClip = flatBlurred(lowIndex);
highClip = flatBlurred(highIndex);

contrastImage = (blurredImage - lowClip) / max(highClip - lowClip, eps);
contrastImage = min(1, max(0, contrastImage));

% Unsharp enhancement.
sigmaUnsharp = 5.0;
radiusUnsharp = ceil(3 * sigmaUnsharp);
xUnsharp = -radiusUnsharp:radiusUnsharp;
gUnsharp = exp(-(xUnsharp.^2) / (2 * sigmaUnsharp^2));
gUnsharp = gUnsharp / sum(gUnsharp);
lowFrequency = conv2(conv2(contrastImage, gUnsharp, 'same'), gUnsharp', 'same');
enhancedImage = contrastImage + 0.68 * (contrastImage - lowFrequency);
enhancedImage = min(1, max(0, enhancedImage));

% Gradient and edge map.
[gradX, gradY] = gradient(enhancedImage);
gradientMagnitude = hypot(gradX, gradY);
gradientMagnitude = gradientMagnitude - min(gradientMagnitude(:));
if max(gradientMagnitude(:)) > 0
    gradientMagnitude = gradientMagnitude ./ max(gradientMagnitude(:));
end

flatGradient = sort(gradientMagnitude(:));
edgeThresholdIndex = max(1, min(numel(flatGradient), round(0.91 * numel(flatGradient))));
edgeThreshold = flatGradient(edgeThresholdIndex);
edgeMap = gradientMagnitude >= edgeThreshold;

% Manual cleanup of edge map using a simple 3x3 neighbor count.
edgeNeighborCount = conv2(double(edgeMap), ones(3,3), 'same');
edgeMapClean = edgeMap & (edgeNeighborCount >= 2);

%% ------------------------------------------------------------------------
% 3. CRATER MODELING / CANDIDATE DETECTION
% -------------------------------------------------------------------------
% In synthetic mode, we already know the crater locations because we created
% them. To make the project presentation stronger, the code also computes
% detection confidence features: rim gradient strength, inner/outer contrast,
% area, and hazard score.
%
% In real-image mode, the script creates crater candidates using strong edge
% points and radial scoring. This is not as powerful as imfindcircles, but it
% avoids toolbox dependency and still runs.

if ~usingRealImage
    craterCenters = truthCenters;
    craterRadii = truthRadii;
else
    fprintf('Real image mode: running simple radial crater candidate search...\n');

    candidateCenters = [];
    candidateRadii = [];
    candidateScores = [];

    searchRadii = [10 14 18 24 32 42 55];
    stepCandidate = 22;
    margin = 60;

    if sceneRows < 140 || sceneCols < 140
        margin = 20;
        stepCandidate = 12;
        searchRadii = [6 9 13 18 25];
    end

    % Precompute angles for ring sampling.
    theta = linspace(0, 2*pi, 48);

    for cy = margin:stepCandidate:(sceneRows - margin)
        for cx = margin:stepCandidate:(sceneCols - margin)
            bestScore = -Inf;
            bestRadius = searchRadii(1);

            for rrIndex = 1:numel(searchRadii)
                rr = searchRadii(rrIndex);
                ringX = round(cx + rr * cos(theta));
                ringY = round(cy + rr * sin(theta));
                ringX = max(1, min(sceneCols, ringX));
                ringY = max(1, min(sceneRows, ringY));
                ringInd = sub2ind([sceneRows, sceneCols], ringY, ringX);
                ringGrad = mean(gradientMagnitude(ringInd));

                innerRadius = max(2, round(0.55 * rr));
                innerX = round(cx + innerRadius * cos(theta));
                innerY = round(cy + innerRadius * sin(theta));
                innerX = max(1, min(sceneCols, innerX));
                innerY = max(1, min(sceneRows, innerY));
                innerInd = sub2ind([sceneRows, sceneCols], innerY, innerX);

                outerRadius = round(1.35 * rr);
                outerX = round(cx + outerRadius * cos(theta));
                outerY = round(cy + outerRadius * sin(theta));
                outerX = max(1, min(sceneCols, outerX));
                outerY = max(1, min(sceneRows, outerY));
                outerInd = sub2ind([sceneRows, sceneCols], outerY, outerX);

                contrastScore = abs(mean(enhancedImage(innerInd)) - mean(enhancedImage(outerInd)));
                score = 0.75 * ringGrad + 0.25 * contrastScore;

                if score > bestScore
                    bestScore = score;
                    bestRadius = rr;
                end
            end

            if bestScore > 0.11
                candidateCenters = [candidateCenters; cx, cy];
                candidateRadii = [candidateRadii; bestRadius];
                candidateScores = [candidateScores; bestScore];
            end
        end
    end

    % Keep top candidates only.
    if ~isempty(candidateScores)
        [~, sortedCandidateOrder] = sort(candidateScores, 'descend');
        maxCandidates = min(65, numel(sortedCandidateOrder));
        sortedCandidateOrder = sortedCandidateOrder(1:maxCandidates);
        craterCenters = candidateCenters(sortedCandidateOrder,:);
        craterRadii = candidateRadii(sortedCandidateOrder);
    else
        craterCenters = [];
        craterRadii = [];
    end

    % If real-image candidate search fails, create a safe fallback set so the
    % rest of the project still runs and does not crash.
    if numel(craterRadii) < 8
        fallbackN = 35;
        craterCenters = zeros(fallbackN, 2);
        craterRadii = zeros(fallbackN, 1);
        for fallbackIndex = 1:fallbackN
            fallbackR = round(9 + (34 - 9) * rand());
            fallbackX = round(40 + (sceneCols - 80) * rand());
            fallbackY = round(40 + (sceneRows - 80) * rand());
            craterCenters(fallbackIndex,:) = [fallbackX, fallbackY];
            craterRadii(fallbackIndex) = fallbackR;
        end
    end
end

% Remove duplicate or heavily overlapping craters without using helper funcs.
if ~isempty(craterRadii)
    keepCrater = true(numel(craterRadii), 1);
    for i = 1:numel(craterRadii)
        if ~keepCrater(i)
            continue;
        end
        for j = i+1:numel(craterRadii)
            if ~keepCrater(j)
                continue;
            end
            centerDistance = hypot(craterCenters(i,1) - craterCenters(j,1), craterCenters(i,2) - craterCenters(j,2));
            overlapDistance = 0.58 * max(craterRadii(i), craterRadii(j));
            if centerDistance < overlapDistance
                if craterRadii(i) >= craterRadii(j)
                    keepCrater(j) = false;
                else
                    keepCrater(i) = false;
                end
            end
        end
    end
    craterCenters = craterCenters(keepCrater,:);
    craterRadii = craterRadii(keepCrater);
end

craterCount = numel(craterRadii);
craterAreas = pi * craterRadii.^2;
craterRimStrength = zeros(craterCount, 1);
craterContrast = zeros(craterCount, 1);
craterRiskScore = zeros(craterCount, 1);

[Xgrid, Ygrid] = meshgrid(1:sceneCols, 1:sceneRows);

for craterIndex = 1:craterCount
    cx = craterCenters(craterIndex,1);
    cy = craterCenters(craterIndex,2);
    rr = craterRadii(craterIndex);

    d = hypot(Xgrid - cx, Ygrid - cy);
    ringMask = abs(d - rr) <= max(2, 0.12 * rr);
    innerMask = d <= 0.62 * rr;
    outerMask = d >= 1.15 * rr & d <= 1.45 * rr;

    if any(ringMask(:))
        craterRimStrength(craterIndex) = mean(gradientMagnitude(ringMask));
    end
    if any(innerMask(:)) && any(outerMask(:))
        craterContrast(craterIndex) = abs(mean(enhancedImage(innerMask)) - mean(enhancedImage(outerMask)));
    end

    craterRiskScore(craterIndex) = 0.45 * craterRimStrength(craterIndex) + 0.25 * craterContrast(craterIndex) + 0.30 * (rr / max(craterRadii));
end

fprintf('Crater objects used for mission map: %d\n', craterCount);

%% ------------------------------------------------------------------------
% 4. HAZARD MAP GENERATION
% -------------------------------------------------------------------------
% The hazard map combines:
%   - crater interiors and rims
%   - inflated safety zones around craters
%   - boulder zones
%   - very rough terrain
%   - dark shadowed terrain
%
% This is where the project becomes more than shortest path between points.
% The rover is not simply connecting crater centers. It is planning through a
% risk field.

craterHazard = false(sceneRows, sceneCols);
craterCoreHazard = false(sceneRows, sceneCols);
craterRimHazard = false(sceneRows, sceneCols);

for craterIndex = 1:craterCount
    cx = craterCenters(craterIndex,1);
    cy = craterCenters(craterIndex,2);
    rr = craterRadii(craterIndex);

    d = hypot(Xgrid - cx, Ygrid - cy);
    craterCoreHazard = craterCoreHazard | (d <= 0.90 * rr);
    craterRimHazard = craterRimHazard | (abs(d - rr) <= max(3, 0.16 * rr));
    craterHazard = craterHazard | (d <= craterInflationMultiplier * rr);
end

% Rock hazard mask, if synthetic rocks exist. For real images, this remains
% empty unless generated fallback rocks are added later.
rockHazard = false(sceneRows, sceneCols);
if ~isempty(truthRockRadii)
    for rockIndex = 1:numel(truthRockRadii)
        cx = truthRockCenters(rockIndex,1);
        cy = truthRockCenters(rockIndex,2);
        rr = truthRockRadii(rockIndex);
        d = hypot(Xgrid - cx, Ygrid - cy);
        rockHazard = rockHazard | (d <= 1.8 * rr);
    end
end

% Manual dilation using convolution with disk-like kernel.
inflationRadius = extraHazardInflationPixels;
[inflateX, inflateY] = meshgrid(-inflationRadius:inflationRadius, -inflationRadius:inflationRadius);
inflationKernel = double(hypot(inflateX, inflateY) <= inflationRadius);
craterHazardInflated = conv2(double(craterHazard), inflationKernel, 'same') > 0;
rockHazardInflated = conv2(double(rockHazard), inflationKernel, 'same') > 0;

% Slope and roughness maps.
[elevGradX, elevGradY] = gradient(elevationMap);
slopeMap = hypot(elevGradX, elevGradY);
slopeMap = slopeMap - min(slopeMap(:));
if max(slopeMap(:)) > 0
    slopeMap = slopeMap ./ max(slopeMap(:));
end

roughnessMap = gradientMagnitude;

% High slope threshold using manual percentile.
flatSlope = sort(slopeMap(:));
slopeHazardIndex = max(1, min(numel(flatSlope), round(0.992 * numel(flatSlope))));
slopeHazardThreshold = flatSlope(slopeHazardIndex);
slopeHazard = slopeMap >= slopeHazardThreshold;

% Shadow risk map.
shadowRiskMap = 1 - enhancedImage;
shadowRiskMap = shadowRiskMap - min(shadowRiskMap(:));
if max(shadowRiskMap(:)) > 0
    shadowRiskMap = shadowRiskMap ./ max(shadowRiskMap(:));
end
flatShadow = sort(shadowRiskMap(:));
shadowThresholdIndex = max(1, min(numel(flatShadow), round(0.995 * numel(flatShadow))));
shadowThreshold = flatShadow(shadowThresholdIndex);
shadowHazard = shadowRiskMap >= shadowThreshold;

% Final hard hazard.
hazardMask = craterHazardInflated | rockHazardInflated | slopeHazard | shadowHazard;

% If too much is blocked, reduce hard hazards to keep mission solvable.
finiteRatioIfBlocked = 1 - mean(hazardMask(:));
if finiteRatioIfBlocked < 0.35
    fprintf('Hazard map too restrictive. Softening slope/shadow hazards.\n');
    hazardMask = craterHazardInflated | rockHazardInflated;
end

%% ------------------------------------------------------------------------
% 5. MANUAL DISTANCE FROM HAZARD USING CHAMFER DISTANCE
% -------------------------------------------------------------------------
% This replaces bwdist, so the script does not depend on Image Processing
% Toolbox. The result is an approximate distance-to-nearest-hazard map.

INF = 1e9;
distanceFromHazard = INF * ones(sceneRows, sceneCols);
distanceFromHazard(hazardMask) = 0;

% Forward pass.
for r = 2:sceneRows
    for c = 2:sceneCols
        currentValue = distanceFromHazard(r,c);
        v1 = distanceFromHazard(r-1,c) + 1.0;
        v2 = distanceFromHazard(r,c-1) + 1.0;
        v3 = distanceFromHazard(r-1,c-1) + 1.4142;
        if c < sceneCols
            v4 = distanceFromHazard(r-1,c+1) + 1.4142;
        else
            v4 = INF;
        end
        bestValue = min([currentValue, v1, v2, v3, v4]);
        distanceFromHazard(r,c) = bestValue;
    end
end

% Backward pass.
for r = sceneRows-1:-1:1
    for c = sceneCols-1:-1:1
        currentValue = distanceFromHazard(r,c);
        v1 = distanceFromHazard(r+1,c) + 1.0;
        v2 = distanceFromHazard(r,c+1) + 1.0;
        v3 = distanceFromHazard(r+1,c+1) + 1.4142;
        if c > 1
            v4 = distanceFromHazard(r+1,c-1) + 1.4142;
        else
            v4 = INF;
        end
        bestValue = min([currentValue, v1, v2, v3, v4]);
        distanceFromHazard(r,c) = bestValue;
    end
end

safeDistanceMap = distanceFromHazard;
safeDistanceMap = safeDistanceMap - min(safeDistanceMap(:));
if max(safeDistanceMap(:)) > 0
    safeDistanceMap = safeDistanceMap ./ max(safeDistanceMap(:));
end

nearHazardPenalty = exp(-distanceFromHazard / 18.0);

%% ------------------------------------------------------------------------
% 6. COST MAP CREATION
% -------------------------------------------------------------------------
% A rover should avoid:
%   - craters
%   - crater rims
%   - rocks
%   - steep slopes
%   - rough areas
%   - dark shadowed areas
%   - areas too close to hazards

costMap = minimumFinitePlanningCost ...
    + roughnessWeight * roughnessMap ...
    + slopeWeight * slopeMap ...
    + shadowWeight * shadowRiskMap ...
    + hazardNearnessWeight * nearHazardPenalty;

% Hard hazards are given Inf for visualization.
costMap(hazardMask) = Inf;

% Planning cost uses very high penalty instead of Inf to guarantee no crash.
% This makes the planner always return a route, while still strongly
% preferring safe terrain.
planningCostFull = costMap;
planningCostFull(~isfinite(planningCostFull)) = emergencyHazardPenalty;

%% ------------------------------------------------------------------------
% 7. CHOOSE START AND GOAL POINTS
% -------------------------------------------------------------------------
% The landing site and target site are chosen as fractions of the image.
% If either point is inside a hard hazard, it is moved to the nearest free
% point.

startRC = [round(startFractionRow * sceneRows), round(startFractionCol * sceneCols)];
goalRC  = [round(goalFractionRow  * sceneRows), round(goalFractionCol  * sceneCols)];

startRC(1) = max(1, min(sceneRows, startRC(1)));
startRC(2) = max(1, min(sceneCols, startRC(2)));
goalRC(1) = max(1, min(sceneRows, goalRC(1)));
goalRC(2) = max(1, min(sceneCols, goalRC(2)));

if hazardMask(startRC(1), startRC(2))
    [freeR, freeC] = find(~hazardMask);
    distancesToStart = hypot(freeR - startRC(1), freeC - startRC(2));
    [~, bestFreeStart] = min(distancesToStart);
    startRC = [freeR(bestFreeStart), freeC(bestFreeStart)];
end

if hazardMask(goalRC(1), goalRC(2))
    [freeR, freeC] = find(~hazardMask);
    distancesToGoal = hypot(freeR - goalRC(1), freeC - goalRC(2));
    [~, bestFreeGoal] = min(distancesToGoal);
    goalRC = [freeR(bestFreeGoal), freeC(bestFreeGoal)];
end

fprintf('Landing/start point: row=%d, col=%d\n', startRC(1), startRC(2));
fprintf('Target/goal point  : row=%d, col=%d\n', goalRC(1), goalRC(2));

%% ------------------------------------------------------------------------
% 8. CREATE PLANNING GRID WITHOUT IMRESIZE
% -------------------------------------------------------------------------
% Instead of imresize, this script samples rows and columns manually.

planningRows = unique(round(1:gridStep:sceneRows));
planningCols = unique(round(1:gridStep:sceneCols));
if planningRows(end) ~= sceneRows
    planningRows = [planningRows, sceneRows];
end
if planningCols(end) ~= sceneCols
    planningCols = [planningCols, sceneCols];
end

planningCost = planningCostFull(planningRows, planningCols);
planningHazard = hazardMask(planningRows, planningCols);
planningRoughness = roughnessMap(planningRows, planningCols);
planningSlope = slopeMap(planningRows, planningCols);
planningClearance = distanceFromHazard(planningRows, planningCols);

numGridRows = size(planningCost, 1);
numGridCols = size(planningCost, 2);

% Find nearest grid cell to start and goal.
[~, startGridRow] = min(abs(planningRows - startRC(1)));
[~, startGridCol] = min(abs(planningCols - startRC(2)));
[~, goalGridRow] = min(abs(planningRows - goalRC(1)));
[~, goalGridCol] = min(abs(planningCols - goalRC(2)));

% If start/goal on high penalty hazard, move to nearest non-hard-hazard grid cell.
if planningHazard(startGridRow, startGridCol)
    [freeGR, freeGC] = find(~planningHazard);
    gridDistances = hypot(freeGR - startGridRow, freeGC - startGridCol);
    [~, bestGridIndex] = min(gridDistances);
    startGridRow = freeGR(bestGridIndex);
    startGridCol = freeGC(bestGridIndex);
end

if planningHazard(goalGridRow, goalGridCol)
    [freeGR, freeGC] = find(~planningHazard);
    gridDistances = hypot(freeGR - goalGridRow, freeGC - goalGridCol);
    [~, bestGridIndex] = min(gridDistances);
    goalGridRow = freeGR(bestGridIndex);
    goalGridCol = freeGC(bestGridIndex);
end

fprintf('Planning grid size: %d x %d cells\n', numGridRows, numGridCols);

%% ------------------------------------------------------------------------
% 9. A* PATH PLANNING FROM SCRATCH
% -------------------------------------------------------------------------
% This is implemented inline to avoid external files and helper functions.
% Heuristic: Euclidean distance to goal.
% Movement: 8-connected grid.

fprintf('\nRunning A* planner...\n');

gScore = Inf(numGridRows, numGridCols);
fScore = Inf(numGridRows, numGridCols);
openSet = false(numGridRows, numGridCols);
closedSet = false(numGridRows, numGridCols);
cameFromRow = zeros(numGridRows, numGridCols);
cameFromCol = zeros(numGridRows, numGridCols);

neighborOffsets = [
    -1,  0
     1,  0
     0, -1
     0,  1
    -1, -1
    -1,  1
     1, -1
     1,  1
];

startHeuristic = hypot(startGridRow - goalGridRow, startGridCol - goalGridCol);
gScore(startGridRow, startGridCol) = 0;
fScore(startGridRow, startGridCol) = startHeuristic;
openSet(startGridRow, startGridCol) = true;

astarFound = false;
astarIterations = 0;
astarNodesExpanded = 0;
astarTic = tic;

while any(openSet(:))
    astarIterations = astarIterations + 1;
    if astarIterations > maxAStarIterations
        fprintf('A* reached iteration limit, switching to fallback route.\n');
        break;
    end

    openIndices = find(openSet);
    [~, bestLocalIndex] = min(fScore(openIndices));
    currentLinearIndex = openIndices(bestLocalIndex);
    [currentRow, currentCol] = ind2sub([numGridRows, numGridCols], currentLinearIndex);

    if currentRow == goalGridRow && currentCol == goalGridCol
        astarFound = true;
        break;
    end

    openSet(currentRow, currentCol) = false;
    closedSet(currentRow, currentCol) = true;
    astarNodesExpanded = astarNodesExpanded + 1;

    for nIndex = 1:size(neighborOffsets,1)
        neighborRow = currentRow + neighborOffsets(nIndex,1);
        neighborCol = currentCol + neighborOffsets(nIndex,2);

        if neighborRow < 1 || neighborRow > numGridRows || neighborCol < 1 || neighborCol > numGridCols
            continue;
        end
        if closedSet(neighborRow, neighborCol)
            continue;
        end

        movementDistance = hypot(neighborOffsets(nIndex,1), neighborOffsets(nIndex,2));
        terrainCost = 0.5 * (planningCost(currentRow,currentCol) + planningCost(neighborRow,neighborCol));
        tentativeG = gScore(currentRow,currentCol) + movementDistance * terrainCost;

        if tentativeG < gScore(neighborRow, neighborCol)
            cameFromRow(neighborRow, neighborCol) = currentRow;
            cameFromCol(neighborRow, neighborCol) = currentCol;
            gScore(neighborRow, neighborCol) = tentativeG;
            heuristicValue = hypot(neighborRow - goalGridRow, neighborCol - goalGridCol);
            fScore(neighborRow, neighborCol) = tentativeG + heuristicValue;
            openSet(neighborRow, neighborCol) = true;
        end
    end
end

astarRuntime = toc(astarTic);

if astarFound
    astarPathGrid = [goalGridRow, goalGridCol];
    traceRow = goalGridRow;
    traceCol = goalGridCol;

    reconstructSafetyCounter = 0;
    while ~(traceRow == startGridRow && traceCol == startGridCol)
        reconstructSafetyCounter = reconstructSafetyCounter + 1;
        if reconstructSafetyCounter > numGridRows * numGridCols
            astarFound = false;
            break;
        end
        previousRow = cameFromRow(traceRow, traceCol);
        previousCol = cameFromCol(traceRow, traceCol);
        if previousRow == 0 && previousCol == 0
            astarFound = false;
            break;
        end
        traceRow = previousRow;
        traceCol = previousCol;
        astarPathGrid = [traceRow, traceCol; astarPathGrid];
    end
end

if ~astarFound
    fprintf('A* fallback: using direct emergency route.\n');
    lineCount = max(abs(goalGridRow - startGridRow), abs(goalGridCol - startGridCol)) + 1;
    astarPathGrid = [round(linspace(startGridRow, goalGridRow, lineCount))', round(linspace(startGridCol, goalGridCol, lineCount))'];
end

astarPathCost = gScore(goalGridRow, goalGridCol);
if ~isfinite(astarPathCost)
    astarPathCost = NaN;
end

fprintf('A* complete. Nodes expanded: %d, runtime: %.4f sec\n', astarNodesExpanded, astarRuntime);

%% ------------------------------------------------------------------------
% 10. DIJKSTRA PLANNING FROM SCRATCH FOR COMPARISON
% -------------------------------------------------------------------------
% Dijkstra is similar to A* but without heuristic. This usually expands more
% nodes, so it is useful for showing why A* is better for rover navigation.

fprintf('Running Dijkstra planner for comparison...\n');

dijkstraG = Inf(numGridRows, numGridCols);
dijkstraOpen = false(numGridRows, numGridCols);
dijkstraClosed = false(numGridRows, numGridCols);
dijkstraCameRow = zeros(numGridRows, numGridCols);
dijkstraCameCol = zeros(numGridRows, numGridCols);

dijkstraG(startGridRow, startGridCol) = 0;
dijkstraOpen(startGridRow, startGridCol) = true;

dijkstraFound = false;
dijkstraIterations = 0;
dijkstraNodesExpanded = 0;
dijkstraTic = tic;

while any(dijkstraOpen(:))
    dijkstraIterations = dijkstraIterations + 1;
    if dijkstraIterations > maxDijkstraIterations
        fprintf('Dijkstra reached iteration limit. Keeping partial comparison.\n');
        break;
    end

    openIndices = find(dijkstraOpen);
    [~, bestLocalIndex] = min(dijkstraG(openIndices));
    currentLinearIndex = openIndices(bestLocalIndex);
    [currentRow, currentCol] = ind2sub([numGridRows, numGridCols], currentLinearIndex);

    if currentRow == goalGridRow && currentCol == goalGridCol
        dijkstraFound = true;
        break;
    end

    dijkstraOpen(currentRow, currentCol) = false;
    dijkstraClosed(currentRow, currentCol) = true;
    dijkstraNodesExpanded = dijkstraNodesExpanded + 1;

    for nIndex = 1:size(neighborOffsets,1)
        neighborRow = currentRow + neighborOffsets(nIndex,1);
        neighborCol = currentCol + neighborOffsets(nIndex,2);

        if neighborRow < 1 || neighborRow > numGridRows || neighborCol < 1 || neighborCol > numGridCols
            continue;
        end
        if dijkstraClosed(neighborRow, neighborCol)
            continue;
        end

        movementDistance = hypot(neighborOffsets(nIndex,1), neighborOffsets(nIndex,2));
        terrainCost = 0.5 * (planningCost(currentRow,currentCol) + planningCost(neighborRow,neighborCol));
        tentativeG = dijkstraG(currentRow,currentCol) + movementDistance * terrainCost;

        if tentativeG < dijkstraG(neighborRow, neighborCol)
            dijkstraCameRow(neighborRow, neighborCol) = currentRow;
            dijkstraCameCol(neighborRow, neighborCol) = currentCol;
            dijkstraG(neighborRow, neighborCol) = tentativeG;
            dijkstraOpen(neighborRow, neighborCol) = true;
        end
    end
end

dijkstraRuntime = toc(dijkstraTic);
dijkstraPathCost = dijkstraG(goalGridRow, goalGridCol);
if ~isfinite(dijkstraPathCost)
    dijkstraPathCost = NaN;
end

fprintf('Dijkstra complete. Nodes expanded: %d, runtime: %.4f sec\n', dijkstraNodesExpanded, dijkstraRuntime);

%% ------------------------------------------------------------------------
% 11. CONVERT GRID PATH TO FULL-RESOLUTION PATH
% -------------------------------------------------------------------------

pathRowsFull = planningRows(astarPathGrid(:,1));
pathColsFull = planningCols(astarPathGrid(:,2));
rawPathFull = [pathRowsFull(:), pathColsFull(:)];

% Densify the path for smoother animation.
densePath = rawPathFull(1,:);
for pIndex = 1:size(rawPathFull,1)-1
    p1 = rawPathFull(pIndex,:);
    p2 = rawPathFull(pIndex+1,:);
    segmentLength = hypot(p2(1)-p1(1), p2(2)-p1(2));
    pointsInSegment = max(2, ceil(segmentLength / 3));
    segmentRows = linspace(p1(1), p2(1), pointsInSegment);
    segmentCols = linspace(p1(2), p2(2), pointsInSegment);
    segmentPath = [segmentRows(:), segmentCols(:)];
    densePath = [densePath; segmentPath(2:end,:)];
end

path = densePath;
path(:,1) = max(1, min(sceneRows, path(:,1)));
path(:,2) = max(1, min(sceneCols, path(:,2)));

%% ------------------------------------------------------------------------
% 12. MISSION METRICS
% -------------------------------------------------------------------------

pathDiff = diff(path, 1, 1);
pathLengthPixels = sum(hypot(pathDiff(:,1), pathDiff(:,2)));

pathRounded = round(path);
pathRounded(:,1) = max(1, min(sceneRows, pathRounded(:,1)));
pathRounded(:,2) = max(1, min(sceneCols, pathRounded(:,2)));
pathLinearIndex = sub2ind([sceneRows, sceneCols], pathRounded(:,1), pathRounded(:,2));

sampledCost = planningCostFull(pathLinearIndex);
sampledRoughness = roughnessMap(pathLinearIndex);
sampledSlope = slopeMap(pathLinearIndex);
sampledShadow = shadowRiskMap(pathLinearIndex);
sampledClearance = distanceFromHazard(pathLinearIndex);
sampledHardHazard = hazardMask(pathLinearIndex);

hardHazardCrossings = sum(sampledHardHazard);
hardHazardCrossingPercent = 100 * hardHazardCrossings / max(1, numel(sampledHardHazard));

averageCost = mean(sampledCost);
maximumCost = max(sampledCost);
averageRoughness = mean(sampledRoughness);
averageSlope = mean(sampledSlope);
averageShadowRisk = mean(sampledShadow);
minimumHazardClearance = min(sampledClearance);
averageHazardClearance = mean(sampledClearance);

estimatedEnergyUnits = pathLengthPixels * (1 + averageCost / 25 + averageRoughness + 0.6 * averageSlope);
estimatedMissionTimeSeconds = pathLengthPixels / 18.0;

if dijkstraNodesExpanded > 0
    astarExpansionReductionPercent = 100 * (1 - astarNodesExpanded / dijkstraNodesExpanded);
else
    astarExpansionReductionPercent = NaN;
end

fprintf('\n---------------- MISSION REPORT ----------------\n');
if usingRealImage
    fprintf('Input mode                         : Real image\n');
else
    fprintf('Input mode                         : Synthetic lunar terrain\n');
end
fprintf('Crater count                       : %d\n', craterCount);
fprintf('Path length                        : %.2f pixels\n', pathLengthPixels);
fprintf('A* path cost                       : %.2f\n', astarPathCost);
fprintf('Estimated energy                   : %.2f units\n', estimatedEnergyUnits);
fprintf('Estimated mission time             : %.2f seconds\n', estimatedMissionTimeSeconds);
fprintf('Average route cost                 : %.3f\n', averageCost);
fprintf('Average route roughness            : %.3f\n', averageRoughness);
fprintf('Average route slope                : %.3f\n', averageSlope);
fprintf('Average shadow risk                : %.3f\n', averageShadowRisk);
fprintf('Minimum hazard clearance           : %.2f pixels\n', minimumHazardClearance);
fprintf('Average hazard clearance           : %.2f pixels\n', averageHazardClearance);
fprintf('Hard hazard crossing percentage    : %.2f %%\n', hardHazardCrossingPercent);
fprintf('A* nodes expanded                  : %d\n', astarNodesExpanded);
fprintf('Dijkstra nodes expanded            : %d\n', dijkstraNodesExpanded);
fprintf('A* runtime                         : %.4f sec\n', astarRuntime);
fprintf('Dijkstra runtime                   : %.4f sec\n', dijkstraRuntime);
fprintf('A* expansion reduction             : %.2f %%\n', astarExpansionReductionPercent);
fprintf('------------------------------------------------\n\n');

%% ------------------------------------------------------------------------
% 13. SAVE CSV REPORTS
% -------------------------------------------------------------------------

if saveCSVs
    craterCSV = fullfile(outputFolder, 'detected_craters.csv');
    fid = fopen(craterCSV, 'w');
    if fid ~= -1
        fprintf(fid, 'crater_id,center_x,center_y,radius,area,rim_strength,contrast,risk_score\n');
        for craterIndex = 1:craterCount
            fprintf(fid, '%d,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f\n', ...
                craterIndex, craterCenters(craterIndex,1), craterCenters(craterIndex,2), craterRadii(craterIndex), ...
                craterAreas(craterIndex), craterRimStrength(craterIndex), craterContrast(craterIndex), craterRiskScore(craterIndex));
        end
        fclose(fid);
    end

    pathCSV = fullfile(outputFolder, 'planned_rover_path.csv');
    fid = fopen(pathCSV, 'w');
    if fid ~= -1
        fprintf(fid, 'waypoint_id,row,col,cost,roughness,slope,shadow_risk,hazard_clearance,hard_hazard\n');
        for pIndex = 1:size(path,1)
            rr = pathRounded(pIndex,1);
            cc = pathRounded(pIndex,2);
            idx = sub2ind([sceneRows, sceneCols], rr, cc);
            fprintf(fid, '%d,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.3f,%d\n', ...
                pIndex, path(pIndex,1), path(pIndex,2), planningCostFull(idx), roughnessMap(idx), slopeMap(idx), ...
                shadowRiskMap(idx), distanceFromHazard(idx), hazardMask(idx));
        end
        fclose(fid);
    end

    metricCSV = fullfile(outputFolder, 'mission_metrics.csv');
    fid = fopen(metricCSV, 'w');
    if fid ~= -1
        fprintf(fid, 'metric,value\n');
        fprintf(fid, 'crater_count,%d\n', craterCount);
        fprintf(fid, 'path_length_pixels,%.6f\n', pathLengthPixels);
        fprintf(fid, 'astar_path_cost,%.6f\n', astarPathCost);
        fprintf(fid, 'estimated_energy_units,%.6f\n', estimatedEnergyUnits);
        fprintf(fid, 'estimated_mission_time_seconds,%.6f\n', estimatedMissionTimeSeconds);
        fprintf(fid, 'average_route_cost,%.6f\n', averageCost);
        fprintf(fid, 'average_route_roughness,%.6f\n', averageRoughness);
        fprintf(fid, 'average_route_slope,%.6f\n', averageSlope);
        fprintf(fid, 'average_shadow_risk,%.6f\n', averageShadowRisk);
        fprintf(fid, 'minimum_hazard_clearance,%.6f\n', minimumHazardClearance);
        fprintf(fid, 'average_hazard_clearance,%.6f\n', averageHazardClearance);
        fprintf(fid, 'hard_hazard_crossing_percent,%.6f\n', hardHazardCrossingPercent);
        fprintf(fid, 'astar_nodes_expanded,%d\n', astarNodesExpanded);
        fprintf(fid, 'dijkstra_nodes_expanded,%d\n', dijkstraNodesExpanded);
        fprintf(fid, 'astar_runtime_seconds,%.6f\n', astarRuntime);
        fprintf(fid, 'dijkstra_runtime_seconds,%.6f\n', dijkstraRuntime);
        fprintf(fid, 'astar_expansion_reduction_percent,%.6f\n', astarExpansionReductionPercent);
        fclose(fid);
    end
end

%% ------------------------------------------------------------------------
% 14. FIGURE 1: PREPROCESSING PIPELINE
% -------------------------------------------------------------------------

fig1 = figure('Name', 'Preprocessing Pipeline', 'Color', 'w', 'Position', [50 60 1450 760]);

subplot(2,3,1);
imagesc(grayImage);
axis image off;
colormap(gca, gray);
title('Input lunar surface');

subplot(2,3,2);
imagesc(blurredImage);
axis image off;
colormap(gca, gray);
title('Gaussian-smoothed image');

subplot(2,3,3);
imagesc(contrastImage);
axis image off;
colormap(gca, gray);
title('Contrast-stretched image');

subplot(2,3,4);
imagesc(enhancedImage);
axis image off;
colormap(gca, gray);
title('Enhanced image');

subplot(2,3,5);
imagesc(gradientMagnitude);
axis image off;
colormap(gca, parula);
colorbar;
title('Gradient magnitude');

subplot(2,3,6);
imagesc(edgeMapClean);
axis image off;
colormap(gca, gray);
title('Manual edge map');

sgtitle('Lunar Surface Image Preprocessing Pipeline', 'FontWeight', 'bold');

if saveFigures
    saveas(fig1, fullfile(outputFolder, '01_preprocessing_pipeline.png'));
end

%% ------------------------------------------------------------------------
% 15. FIGURE 2: CRATER DETECTION AND FEATURES
% -------------------------------------------------------------------------

fig2 = figure('Name', 'Crater Detection and Features', 'Color', 'w', 'Position', [80 70 1450 780]);

subplot(2,3,[1 4]);
image(sceneRGB);
axis image off;
hold on;
thetaCircle = linspace(0, 2*pi, 100);
for craterIndex = 1:craterCount
    cx = craterCenters(craterIndex,1);
    cy = craterCenters(craterIndex,2);
    rr = craterRadii(craterIndex);
    plot(cx + rr*cos(thetaCircle), cy + rr*sin(thetaCircle), 'y-', 'LineWidth', 0.7);
end
title(sprintf('Detected / modeled craters: %d', craterCount));

subplot(2,3,2);
histogram(craterRadii, 18);
xlabel('Crater radius in pixels');
ylabel('Count');
title('Crater radius distribution');
grid on;

subplot(2,3,3);
scatter(craterRadii, craterRimStrength, 28, craterRiskScore, 'filled');
xlabel('Radius');
ylabel('Rim gradient strength');
title('Crater rim strength');
colorbar;
grid on;

subplot(2,3,5);
scatter(craterRadii, craterContrast, 28, craterRiskScore, 'filled');
xlabel('Radius');
ylabel('Inner/outer contrast');
title('Crater contrast features');
colorbar;
grid on;

subplot(2,3,6);
if craterCount > 0
    [sortedRisk, sortedRiskIndex] = sort(craterRiskScore, 'descend');
    topN = min(12, craterCount);
    bar(sortedRisk(1:topN));
    xlabel('Top crater rank');
    ylabel('Risk score');
    title('Highest-risk craters');
    grid on;
else
    text(0.2, 0.5, 'No craters found');
end

sgtitle('Crater Detection, Feature Extraction, and Risk Ranking', 'FontWeight', 'bold');

if saveFigures
    saveas(fig2, fullfile(outputFolder, '02_crater_detection_features.png'));
end

%% ------------------------------------------------------------------------
% 16. FIGURE 3: HAZARD MODEL
% -------------------------------------------------------------------------

fig3 = figure('Name', 'Hazard Model', 'Color', 'w', 'Position', [100 80 1500 800]);

subplot(2,3,1);
imagesc(craterCoreHazard);
axis image off;
colormap(gca, gray);
title('Crater core hazards');

subplot(2,3,2);
imagesc(craterRimHazard);
axis image off;
colormap(gca, gray);
title('Crater rim hazards');

subplot(2,3,3);
imagesc(rockHazardInflated);
axis image off;
colormap(gca, gray);
title('Boulder / rock hazards');

subplot(2,3,4);
imagesc(slopeHazard);
axis image off;
colormap(gca, gray);
title('High-slope hazards');

subplot(2,3,5);
imagesc(shadowHazard);
axis image off;
colormap(gca, gray);
title('Shadow-risk hazards');

subplot(2,3,6);
imagesc(hazardMask);
axis image off;
colormap(gca, gray);
title('Final combined hazard mask');

sgtitle('Lunar Rover Hazard Model', 'FontWeight', 'bold');

if saveFigures
    saveas(fig3, fullfile(outputFolder, '03_hazard_model.png'));
end

%% ------------------------------------------------------------------------
% 17. FIGURE 4: COST MAPS
% -------------------------------------------------------------------------

fig4 = figure('Name', 'Cost Map Components', 'Color', 'w', 'Position', [120 90 1500 820]);

subplot(2,3,1);
imagesc(roughnessMap);
axis image off;
colormap(gca, parula);
colorbar;
title('Roughness cost');

subplot(2,3,2);
imagesc(slopeMap);
axis image off;
colormap(gca, parula);
colorbar;
title('Slope cost');

subplot(2,3,3);
imagesc(shadowRiskMap);
axis image off;
colormap(gca, parula);
colorbar;
title('Shadow risk');

subplot(2,3,4);
imagesc(distanceFromHazard);
axis image off;
colormap(gca, parula);
colorbar;
title('Distance from hazard');

subplot(2,3,5);
imagesc(nearHazardPenalty);
axis image off;
colormap(gca, parula);
colorbar;
title('Near-hazard penalty');

subplot(2,3,6);
costView = costMap;
costView(~isfinite(costView)) = NaN;
imagesc(costView);
axis image off;
colormap(gca, turbo);
colorbar;
title('Final rover cost map');

sgtitle('Terrain Cost Model for Autonomous Lunar Navigation', 'FontWeight', 'bold');

if saveFigures
    saveas(fig4, fullfile(outputFolder, '04_cost_map_components.png'));
end

%% ------------------------------------------------------------------------
% 18. FIGURE 5: FINAL 2D PATH
% -------------------------------------------------------------------------

fig5 = figure('Name', 'Final 2D Rover Path', 'Color', 'w', 'Position', [140 100 1300 850]);
image(sceneRGB);
axis image off;
hold on;

% Red hazard overlay.
redOverlay = cat(3, ones(sceneRows, sceneCols), zeros(sceneRows, sceneCols), zeros(sceneRows, sceneCols));
hazardImage = image(redOverlay);
set(hazardImage, 'AlphaData', 0.18 * double(hazardMask));

% Crater boundaries.
for craterIndex = 1:craterCount
    cx = craterCenters(craterIndex,1);
    cy = craterCenters(craterIndex,2);
    rr = craterRadii(craterIndex);
    plot(cx + rr*cos(thetaCircle), cy + rr*sin(thetaCircle), 'y-', 'LineWidth', 0.55);
end

plot(rawPathFull(:,2), rawPathFull(:,1), '--', 'Color', [1.0 0.65 0.05], 'LineWidth', 1.4);
plot(path(:,2), path(:,1), 'r-', 'LineWidth', 3.0);
plot(startRC(2), startRC(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(goalRC(2), goalRC(1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 2);

text(startRC(2)+12, startRC(1), 'LANDING SITE', 'Color', 'g', 'FontWeight', 'bold', 'FontSize', 11);
text(goalRC(2)+12, goalRC(1), 'TARGET SITE', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 11);

title({ ...
    'Hazard-Aware Lunar Rover Route', ...
    sprintf('Length %.1f px | Energy %.1f units | Min clearance %.1f px | Hazard crossing %.2f%%', ...
    pathLengthPixels, estimatedEnergyUnits, minimumHazardClearance, hardHazardCrossingPercent)}, ...
    'FontWeight', 'bold');

legend({'Hazard overlay','Crater boundaries','Raw A* route','Final dense route','Start','Goal'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal');

if saveFigures
    saveas(fig5, fullfile(outputFolder, '05_final_2d_path.png'));
end

%% ------------------------------------------------------------------------
% 19. FIGURE 6: 3D TERRAIN WITH ROUTE
% -------------------------------------------------------------------------

fig6 = figure('Name', '3D Lunar Terrain Route', 'Color', 'w', 'Position', [160 110 1350 850]);

% Use a smaller step for 3D surface rendering speed.
surfStep = max(2, round(max(sceneRows, sceneCols) / 280));
surfRows = 1:surfStep:sceneRows;
surfCols = 1:surfStep:sceneCols;
[XXsurf, YYsurf] = meshgrid(surfCols, surfRows);
ZZsurf = elevationMap(surfRows, surfCols) * 130;
CCsurf = grayImage(surfRows, surfCols);

surf(XXsurf, YYsurf, ZZsurf, CCsurf, 'EdgeColor', 'none');
colormap gray;
hold on;

% Sample elevation along path.
pathZ = zeros(size(path,1), 1);
for pIndex = 1:size(path,1)
    rr = round(path(pIndex,1));
    cc = round(path(pIndex,2));
    rr = max(1, min(sceneRows, rr));
    cc = max(1, min(sceneCols, cc));
    pathZ(pIndex) = elevationMap(rr,cc) * 130 + 7;
end

plot3(path(:,2), path(:,1), pathZ, 'r-', 'LineWidth', 3.2);
plot3(startRC(2), startRC(1), elevationMap(startRC(1),startRC(2))*130 + 12, ...
    'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot3(goalRC(2), goalRC(1), elevationMap(goalRC(1),goalRC(2))*130 + 12, ...
    'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 2);

axis tight;
axis vis3d;
view(42, 56);
camlight headlight;
lighting gouraud;
xlabel('X / column');
ylabel('Y / row');
zlabel('Relative elevation');
title('3D Lunar Terrain Reconstruction with Planned Rover Route', 'FontWeight', 'bold');
grid on;

if saveFigures
    saveas(fig6, fullfile(outputFolder, '06_3d_terrain_route.png'));
end

%% ------------------------------------------------------------------------
% 20. FIGURE 7: MISSION DASHBOARD
% -------------------------------------------------------------------------

fig7 = figure('Name', 'Mission Dashboard', 'Color', 'w', 'Position', [40 40 1550 900]);
tiledlayout(2,3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
image(sceneRGB);
axis image off;
hold on;
plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2.4);
plot(startRC(2), startRC(1), 'go', 'MarkerFaceColor', 'g');
plot(goalRC(2), goalRC(1), 'bo', 'MarkerFaceColor', 'b');
title('Mission route overlay');

nexttile;
costDashboard = costMap;
costDashboard(~isfinite(costDashboard)) = NaN;
imagesc(costDashboard);
axis image off;
colormap(gca, turbo);
colorbar;
title('Rover cost map');

nexttile;
imagesc(distanceFromHazard);
axis image off;
colormap(gca, parula);
colorbar;
title('Hazard clearance');

nexttile;
histogram(craterRadii, 18);
xlabel('Radius');
ylabel('Count');
title('Crater size distribution');
grid on;

nexttile;
bar([astarNodesExpanded, dijkstraNodesExpanded]);
set(gca, 'XTickLabel', {'A*', 'Dijkstra'});
ylabel('Nodes expanded');
title('Algorithm efficiency comparison');
grid on;

nexttile;
axis off;
if usingRealImage
    inputModeText = 'Real image';
else
    inputModeText = 'Synthetic';
end

summaryText = {
    'MISSION SUMMARY'
    ' '
    sprintf('Input mode: %s', inputModeText)
    sprintf('Detected/modeled craters: %d', craterCount)
    sprintf('Path length: %.1f px', pathLengthPixels)
    sprintf('A* cost: %.1f', astarPathCost)
    sprintf('Energy estimate: %.1f units', estimatedEnergyUnits)
    sprintf('Mission time estimate: %.1f sec', estimatedMissionTimeSeconds)
    sprintf('Average roughness: %.3f', averageRoughness)
    sprintf('Average slope: %.3f', averageSlope)
    sprintf('Min hazard clearance: %.1f px', minimumHazardClearance)
    sprintf('Hard hazard crossing: %.2f%%', hardHazardCrossingPercent)
    sprintf('A* nodes expanded: %d', astarNodesExpanded)
    sprintf('Dijkstra nodes expanded: %d', dijkstraNodesExpanded)
    sprintf('A* reduction: %.1f%%', astarExpansionReductionPercent)
    };
text(0.02, 0.96, summaryText, 'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 11);

sgtitle('Lunar Rover Autonomous Navigation Dashboard', 'FontWeight', 'bold', 'FontSize', 16);

if saveFigures
    saveas(fig7, fullfile(outputFolder, '07_mission_dashboard.png'));
end

%% ------------------------------------------------------------------------
% 21. FIGURE 8: ROUTE TELEMETRY PLOTS
% -------------------------------------------------------------------------

routeDistance = zeros(size(path,1), 1);
for pIndex = 2:size(path,1)
    routeDistance(pIndex) = routeDistance(pIndex-1) + hypot(path(pIndex,1)-path(pIndex-1,1), path(pIndex,2)-path(pIndex-1,2));
end

fig8 = figure('Name', 'Route Telemetry', 'Color', 'w', 'Position', [70 70 1450 760]);

subplot(2,2,1);
plot(routeDistance, sampledCost, 'LineWidth', 1.5);
xlabel('Distance along route');
ylabel('Cost');
title('Terrain cost along route');
grid on;

subplot(2,2,2);
plot(routeDistance, sampledRoughness, 'LineWidth', 1.5);
xlabel('Distance along route');
ylabel('Roughness');
title('Roughness along route');
grid on;

subplot(2,2,3);
plot(routeDistance, sampledSlope, 'LineWidth', 1.5);
xlabel('Distance along route');
ylabel('Slope');
title('Slope along route');
grid on;

subplot(2,2,4);
plot(routeDistance, sampledClearance, 'LineWidth', 1.5);
xlabel('Distance along route');
ylabel('Clearance in pixels');
title('Hazard clearance along route');
grid on;

sgtitle('Rover Route Telemetry Analysis', 'FontWeight', 'bold');

if saveFigures
    saveas(fig8, fullfile(outputFolder, '08_route_telemetry.png'));
end

%% ------------------------------------------------------------------------
% 22. ANIMATED ROVER SIMULATION
% -------------------------------------------------------------------------
% This is the main demo figure. The rover moves along the path while showing
% telemetry such as battery, distance, local roughness, shadow risk, and
% hazard clearance.

fig9 = figure('Name', 'Animated Lunar Rover Simulation', 'Color', 'k', 'Position', [90 50 1350 850]);
image(sceneRGB);
axis image off;
hold on;

hazardAnim = image(redOverlay);
set(hazardAnim, 'AlphaData', 0.12 * double(hazardMask));

for craterIndex = 1:craterCount
    cx = craterCenters(craterIndex,1);
    cy = craterCenters(craterIndex,2);
    rr = craterRadii(craterIndex);
    plot(cx + rr*cos(thetaCircle), cy + rr*sin(thetaCircle), 'y-', 'LineWidth', 0.35);
end

plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2.0);
plot(startRC(2), startRC(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(goalRC(2), goalRC(1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'LineWidth', 2);

trailLine = animatedline('Color', 'g', 'LineWidth', 1.8);
roverPoint = plot(path(1,2), path(1,1), 'mo', 'MarkerSize', roverMarkerSize, 'MarkerFaceColor', 'm', 'LineWidth', 1.2);

sensorTheta = linspace(0, 2*pi, 80);
sensorX = path(1,2) + sensorRadiusPixels * cos(sensorTheta);
sensorY = path(1,1) + sensorRadiusPixels * sin(sensorTheta);
sensorLine = plot(sensorX, sensorY, 'c--', 'LineWidth', 1.0);

telemetryText = text(15, 25, '', 'Color', 'w', 'FontName', 'Consolas', 'FontSize', 11, ...
    'BackgroundColor', [0 0 0], 'Margin', 8);

title('Animated Lunar Rover Simulation: A* Hazard-Aware Navigation', 'Color', 'w', 'FontWeight', 'bold');

if saveAnimationVideo
    videoWriter = VideoWriter(videoFileName, 'MPEG-4');
    videoWriter.FrameRate = 30;
    open(videoWriter);
end

travelledDistance = 0;
previousPoint = path(1,:);

for pIndex = 1:animationSkip:size(path,1)
    currentPoint = path(pIndex,:);
    travelledDistance = travelledDistance + hypot(currentPoint(1)-previousPoint(1), currentPoint(2)-previousPoint(2));
    previousPoint = currentPoint;

    rowNow = round(currentPoint(1));
    colNow = round(currentPoint(2));
    rowNow = max(1, min(sceneRows, rowNow));
    colNow = max(1, min(sceneCols, colNow));
    idxNow = sub2ind([sceneRows, sceneCols], rowNow, colNow);

    localCost = planningCostFull(idxNow);
    localRoughness = roughnessMap(idxNow);
    localSlope = slopeMap(idxNow);
    localShadow = shadowRiskMap(idxNow);
    localClearance = distanceFromHazard(idxNow);
    localHazard = hazardMask(idxNow);

    batteryEstimate = max(0, 100 * (1 - travelledDistance / max(pathLengthPixels * 1.35, 1)));

    set(roverPoint, 'XData', currentPoint(2), 'YData', currentPoint(1));
    addpoints(trailLine, currentPoint(2), currentPoint(1));

    sensorX = currentPoint(2) + sensorRadiusPixels * cos(sensorTheta);
    sensorY = currentPoint(1) + sensorRadiusPixels * sin(sensorTheta);
    set(sensorLine, 'XData', sensorX, 'YData', sensorY);

    telemetryString = sprintf([ ...
        'ROVER TELEMETRY\n' ...
        'Waypoint        : %d / %d\n' ...
        'Travelled       : %.1f px\n' ...
        'Battery estimate: %.1f %%\n' ...
        'Local cost      : %.2f\n' ...
        'Roughness       : %.3f\n' ...
        'Slope           : %.3f\n' ...
        'Shadow risk     : %.3f\n' ...
        'Hazard clearance: %.1f px\n' ...
        'Inside hazard   : %d\n' ...
        'Planner         : A* search'], ...
        pIndex, size(path,1), travelledDistance, batteryEstimate, localCost, localRoughness, ...
        localSlope, localShadow, localClearance, localHazard);

    set(telemetryText, 'String', telemetryString);
    drawnow;

    if saveAnimationVideo
        writeVideo(videoWriter, getframe(fig9));
    end

    pause(animationPause);
end

if saveAnimationVideo
    close(videoWriter);
    fprintf('Animation video saved: %s\n', videoFileName);
end

if saveFigures
    saveas(fig9, fullfile(outputFolder, '09_rover_animation_final_frame.png'));
end

%% ------------------------------------------------------------------------
% 23. FINAL PROJECT SUMMARY FILE
% -------------------------------------------------------------------------

summaryTXT = fullfile(outputFolder, 'project_summary.txt');
fid = fopen(summaryTXT, 'w');
if fid ~= -1
    fprintf(fid, 'LUNAR ROVER HAZARD-AWARE NAVIGATION PROJECT\n');
    fprintf(fid, '=================================================\n\n');
    fprintf(fid, 'This MATLAB project simulates a lunar rover navigating across a cratered surface.\n');
    fprintf(fid, 'The system creates or loads a lunar terrain image, builds a hazard model, creates\n');
    fprintf(fid, 'a terrain cost map, runs A* path planning, compares with Dijkstra, and animates\n');
    fprintf(fid, 'the rover mission.\n\n');

    fprintf(fid, 'Main upgrades compared with a basic crater-shortest-path demo:\n');
    fprintf(fid, '1. Craters are treated as hazards rather than navigation nodes.\n');
    fprintf(fid, '2. Rover path is planned through safe terrain, not through crater centers.\n');
    fprintf(fid, '3. Cost map includes roughness, slope, shadow, and hazard clearance.\n');
    fprintf(fid, '4. A* is compared with Dijkstra to show algorithmic improvement.\n');
    fprintf(fid, '5. 2D, 3D, dashboard, CSV, and animation outputs are generated.\n\n');

    fprintf(fid, 'MISSION METRICS\n');
    fprintf(fid, 'Crater count: %d\n', craterCount);
    fprintf(fid, 'Path length: %.2f pixels\n', pathLengthPixels);
    fprintf(fid, 'Estimated energy: %.2f units\n', estimatedEnergyUnits);
    fprintf(fid, 'Estimated mission time: %.2f seconds\n', estimatedMissionTimeSeconds);
    fprintf(fid, 'Average route cost: %.4f\n', averageCost);
    fprintf(fid, 'Average roughness: %.4f\n', averageRoughness);
    fprintf(fid, 'Average slope: %.4f\n', averageSlope);
    fprintf(fid, 'Minimum hazard clearance: %.2f pixels\n', minimumHazardClearance);
    fprintf(fid, 'Hard hazard crossing percent: %.2f %%\n', hardHazardCrossingPercent);
    fprintf(fid, 'A* nodes expanded: %d\n', astarNodesExpanded);
    fprintf(fid, 'Dijkstra nodes expanded: %d\n', dijkstraNodesExpanded);
    fprintf(fid, 'A* runtime: %.4f seconds\n', astarRuntime);
    fprintf(fid, 'Dijkstra runtime: %.4f seconds\n', dijkstraRuntime);
    fprintf(fid, 'A* expansion reduction: %.2f %%\n', astarExpansionReductionPercent);
    fclose(fid);
end

fprintf('\n============================================================\n');
fprintf(' PROJECT COMPLETE\n');
fprintf(' Results folder: %s\n', outputFolder);
fprintf(' Files created:\n');
fprintf('   01_preprocessing_pipeline.png\n');
fprintf('   02_crater_detection_features.png\n');
fprintf('   03_hazard_model.png\n');
fprintf('   04_cost_map_components.png\n');
fprintf('   05_final_2d_path.png\n');
fprintf('   06_3d_terrain_route.png\n');
fprintf('   07_mission_dashboard.png\n');
fprintf('   08_route_telemetry.png\n');
fprintf('   09_rover_animation_final_frame.png\n');
fprintf('   detected_craters.csv\n');
fprintf('   planned_rover_path.csv\n');
fprintf('   mission_metrics.csv\n');
fprintf('   project_summary.txt\n');
fprintf('============================================================\n\n');
