clc; clear; close all;
rng(5);
warning('off','fuzzy:general:NoRuleFired');   % should not trigger after fixes

%% ======================= OUTPUT ===========================
outDir = fullfile(pwd,'fig_out');
if ~exist(outDir,'dir'), mkdir(outDir); end

%% ======================= MAIN PARAMETERS ================================
areaSize      = [120, 120, 60];           % [length,width,height]
iterations    = 180;
runs          = 8;                        % multi-start runs for robustness

numSensors    = 7;
numRouters    = 3;

baseSensorR   = [42, 36, 38, 45, 40, 34, 37];     % base (nominal) sensor radii
baseRouterR   = 35*ones(1,numRouters);            % base router radii

% -------------------- City-like buildings (obstacles) --------------------
numBuildings  = 12;
minBase       = [8, 8];  maxBase = [18, 18];
minHeight     = 18;      maxHeight = 55;
streetMargin  = 6;
obstacles = zeros(numBuildings,6);
for b = 1:numBuildings
    w = randi([minBase(1), maxBase(1)]);
    d = randi([minBase(2), maxBase(2)]);
    h = randi([minHeight,   maxHeight]);
    x1 = randi([streetMargin, areaSize(1)-w-streetMargin]);
    y1 = randi([streetMargin, areaSize(2)-d-streetMargin]);
    z1 = 0;
    x2 = x1 + w; y2 = y1 + d; z2 = h;
    obstacles(b,:) = [x1,y1,z1,x2,y2,z2];
end

% -------------------- Initial sensor/router positions --------------------
gridX = linspace(10, areaSize(1)-10, 4);
gridY = linspace(10, areaSize(2)-10, 4);
[GX,GY] = ndgrid(gridX,gridY);
streetPts = [GX(:),GY(:)];
streetPts = streetPts(randperm(size(streetPts,1)),:);
initialPos = [streetPts(1:numSensors,:), zeros(numSensors,1)];
routerPos  = [streetPts(end-numRouters+1:end,:), zeros(numRouters,1)];

%% ======================= EXPERIMENT (MULTI-RUN) =========================
% Histories (iterations+1 by runs)
covHistAVF_all = zeros(iterations+1, runs);
covHistFIS_all = zeros(iterations+1, runs);
covHistGWO_all = zeros(iterations+1, runs);
covHistFUZ_all = zeros(iterations+1, runs);
covHistPSO_all = zeros(iterations+1, runs);
covHistDNG_all = zeros(iterations+1, runs);   % <-- Dingo

% Finals
finalCovAVF = zeros(1, runs);  finalCovFIS = zeros(1, runs);
finalCovGWO = zeros(1, runs);  finalCovFUZ = zeros(1, runs);
finalCovPSO = zeros(1, runs);  finalCovDNG = zeros(1, runs);

% Connectivity
connAVF = zeros(1, runs); connFIS = zeros(1, runs);
connGWO = zeros(1, runs); connFUZ = zeros(1, runs);
connPSO = zeros(1, runs); connDNG = zeros(1, runs);

% Runtime (seconds)
timeAVF = zeros(1, runs); timeFIS = zeros(1, runs);
timeGWO = zeros(1, runs); timeFUZ = zeros(1, runs);
timePSO = zeros(1, runs); timeDNG = zeros(1, runs);

% Best trackers
bestRunAVF=1; bestRunFIS=1; bestRunGWO=1; bestRunFUZ=1; bestRunPSO=1; bestRunDNG=1;
bestCovAVF=-inf; bestCovFIS=-inf; bestCovGWO=-inf; bestCovFUZ=-inf; bestCovPSO=-inf; bestCovDNG=-inf;
bestOptAVF=struct; bestOptFIS=struct; bestOptGWO=struct; bestOptFUZ=struct; bestOptPSO=struct; bestOptDNG=struct;

for r = 1:runs
    rng(r);
    % same starts for all methods in this run
    initPosR  = jitterOnStreets(initialPos, 2.0, areaSize);
    initRoutR = jitterOnStreets(routerPos,  2.0, areaSize);

    % ---------- AVF (Adaptive Vector Force) ----------
    t0 = tic;
    pf = makeProgressFcn('AVF', r, iterations, false);  % set true for GUI waitbar
    [optPosAVF,optRoutAVF,covHistAVF] = optimizeSensorRouterLocationsAVF( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timeAVF(r)         = toc(t0);
    covHistAVF_all(:,r)= covHistAVF;
    finalCovAVF(r)     = covHistAVF(end);
    connAVF(r)         = connectivityMetric(optPosAVF, optRoutAVF, baseRouterR);
    if finalCovAVF(r) > bestCovAVF
        bestCovAVF = finalCovAVF(r); bestRunAVF = r;
        bestOptAVF.initPos  = initPosR; bestOptAVF.initRout = initRoutR;
        bestOptAVF.optPos   = optPosAVF; bestOptAVF.optRout  = optRoutAVF;
        bestOptAVF.covHist  = covHistAVF; bestOptAVF.sRad = baseSensorR; bestOptAVF.rRad = baseRouterR;
    end

    % ---------- FIS-Pro (StepFIS + SteerFIS) ----------
    t0 = tic;
    pf = makeProgressFcn('FIS-Pro', r, iterations, false);
    [optPosFIS,optRoutFIS,covHistFIS] = optimizeSensorRouterLocationsFIS_Pro( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timeFIS(r)         = toc(t0);
    covHistFIS_all(:,r)= covHistFIS;
    finalCovFIS(r)     = covHistFIS(end);
    connFIS(r)         = connectivityMetric(optPosFIS, optRoutFIS, baseRouterR);
    if finalCovFIS(r) > bestCovFIS
        bestCovFIS = finalCovFIS(r); bestRunFIS = r;
        bestOptFIS.initPos  = initPosR; bestOptFIS.initRout = initRoutR;
        bestOptFIS.optPos   = optPosFIS; bestOptFIS.optRout  = optRoutFIS;
        bestOptFIS.covHist  = covHistFIS; bestOptFIS.sRad = baseSensorR; bestOptFIS.rRad = baseRouterR;
    end

    % ---------- GWO-3D ----------
    t0 = tic;
    pf = makeProgressFcn('GWO-3D', r, iterations, false);
    [optPosGWO,optRoutGWO,covHistGWO] = optimizeSensorRouterLocationsGWO3D( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timeGWO(r)         = toc(t0);
    covHistGWO_all(:,r)= covHistGWO;
    finalCovGWO(r)     = covHistGWO(end);
    connGWO(r)         = connectivityMetric(optPosGWO, optRoutGWO, baseRouterR);
    if finalCovGWO(r) > bestCovGWO
        bestCovGWO = finalCovGWO(r); bestRunGWO = r;
        bestOptGWO.initPos  = initPosR; bestOptGWO.initRout = initRoutR;
        bestOptGWO.optPos   = optPosGWO; bestOptGWO.optRout  = optRoutGWO;
        bestOptGWO.covHist  = covHistGWO; bestOptGWO.sRad = baseSensorR; bestOptGWO.rRad = baseRouterR;
    end

    % ---------- FuzzyDet ----------
    t0 = tic;
    pf = makeProgressFcn('FuzzyDet', r, iterations, false);
    [optPosFUZ,optRoutFUZ,covHistFUZ] = optimizeSensorRouterLocationsFuzzyDet( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timeFUZ(r)         = toc(t0);
    covHistFUZ_all(:,r)= covHistFUZ;
    finalCovFUZ(r)     = covHistFUZ(end);
    connFUZ(r)         = connectivityMetric(optPosFUZ, optRoutFUZ, baseRouterR);
    if finalCovFUZ(r) > bestCovFUZ
        bestCovFUZ = finalCovFUZ(r); bestRunFUZ = r;
        bestOptFUZ.initPos  = initPosR; bestOptFUZ.initRout = initRoutR;
        bestOptFUZ.optPos   = optPosFUZ; bestOptFUZ.optRout  = optRoutFUZ;
        bestOptFUZ.covHist  = covHistFUZ; bestOptFUZ.sRad = baseSensorR; bestOptFUZ.rRad = baseRouterR;
    end

    % ---------- HybridPSO ----------
    t0 = tic;
    pf = makeProgressFcn('HybridPSO', r, iterations, false);
    [optPosPSO,optRoutPSO,covHistPSO] = optimizeSensorRouterLocationsHybridPSO( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timePSO(r)         = toc(t0);
    covHistPSO_all(:,r)= covHistPSO;
    finalCovPSO(r)     = covHistPSO(end);
    connPSO(r)         = connectivityMetric(optPosPSO, optRoutPSO, baseRouterR);
    if finalCovPSO(r) > bestCovPSO
        bestCovPSO = finalCovPSO(r); bestRunPSO = r;
        bestOptPSO.initPos  = initPosR; bestOptPSO.initRout = initRoutR;
        bestOptPSO.optPos   = optPosPSO; bestOptPSO.optRout  = optRoutPSO;
        bestOptPSO.covHist  = covHistPSO; bestOptPSO.sRad = baseSensorR; bestOptPSO.rRad = baseRouterR;
    end

    % ---------- MODOA (Dingo-inspired 3D) ----------
    t0 = tic;
    pf = makeProgressFcn('MODOA', r, iterations, false);
    [optPosDNG,optRoutDNG,covHistDNG] = optimizeSensorRouterLocationsDingo3D( ...
        initPosR,initRoutR,baseSensorR,baseRouterR,areaSize,obstacles,iterations, pf);
    timeDNG(r)         = toc(t0);
    covHistDNG_all(:,r)= covHistDNG;
    finalCovDNG(r)     = covHistDNG(end);
    connDNG(r)         = connectivityMetric(optPosDNG, optRoutDNG, baseRouterR);
    if finalCovDNG(r) > bestCovDNG
        bestCovDNG = finalCovDNG(r); bestRunDNG = r;
        bestOptDNG.initPos  = initPosR; bestOptDNG.initRout = initRoutR;
        bestOptDNG.optPos   = optPosDNG; bestOptDNG.optRout  = optRoutDNG;
        bestOptDNG.covHist  = covHistDNG; bestOptDNG.sRad = baseSensorR; bestOptDNG.rRad = baseRouterR;
    end
end

%% ======================= REPORT / PLOTS =================================
fprintf('\n=== SUMMARY OVER %d RUNS ===\n', runs);
fprintf('AVF       : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovAVF), std(finalCovAVF), mean(connAVF)*100, mean(timeAVF));
fprintf('FCO       : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovFIS), std(finalCovFIS), mean(connFIS)*100, mean(timeFIS));
fprintf('GWO-3D    : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovGWO), std(finalCovGWO), mean(connGWO)*100, mean(timeGWO));
fprintf('FuzzyDet  : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovFUZ), std(finalCovFUZ), mean(connFUZ)*100, mean(timeFUZ));
fprintf('HybridPSO : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovPSO), std(finalCovPSO), mean(connPSO)*100, mean(timePSO));
fprintf('MODOA  : mean final cov = %.2f%%  (std = %.2f) | mean conn = %.2f%% | mean time = %.2fs\n', ...
    mean(finalCovDNG), std(finalCovDNG), mean(connDNG)*100, mean(timeDNG));

% Coverage-per-iteration (Mean ± Std) for all methods
figure('Name','Coverage per Iteration (Mean ± Std)','Color','w');
mA = mean(covHistAVF_all,2); sA = std(covHistAVF_all,0,2);
mF = mean(covHistFIS_all,2); sF = std(covHistFIS_all,0,2);
mG = mean(covHistGWO_all,2); sG = std(covHistGWO_all,0,2);
mZ = mean(covHistFUZ_all,2); sZ = std(covHistFUZ_all,0,2);
mP = mean(covHistPSO_all,2); sP = std(covHistPSO_all,0,2);
mD = mean(covHistDNG_all,2); sD = std(covHistDNG_all,0,2);
x  = 0:iterations;
shadedError(x, mA, sA, [0.2 0.2 0.7], [0.85 0.85 0.95]); hold on;   % AVF
shadedError(x, mG, sG, [0.8 0.3 0.0], [0.98 0.90 0.85]);            % GWO
shadedError(x, mZ, sZ, [0.6 0.0 0.6], [0.93 0.85 0.95]);            % FuzzyDet
shadedError(x, mP, sP, [0.1 0.6 0.6], [0.85 0.95 0.95]);            % HybridPSO
shadedError(x, mF, sF, [0.0 0.5 0.0], [0.85 0.95 0.85]);            % FIS-Pro
shadedError(x, mD, sD, [0.2 0.2 0.2], [0.92 0.92 0.92]);            % MODOA
xlabel('Iteration'); ylabel('Coverage (%)'); grid on; box on;
legend('AVF','GWO-3D','FuzzyDet','HybridPSO','FCO','MODOA','Location','best');
title('Coverage vs Iteration');

% Final Metrics
figure('Name','Final Metrics','Color','w');
subplot(1,3,1);
boxplot([finalCovAVF',finalCovFIS',finalCovGWO',finalCovFUZ',finalCovPSO',finalCovDNG'], ...
    'Labels',{'AVF','FCO','GWO-3D','FuzzyDet','HybridPSO','MODOA'});
ylabel('Final Coverage (%)'); grid on; title('Coverage');
subplot(1,3,2);
boxplot([connAVF'*100,connFIS'*100,connGWO'*100,connFUZ'*100,connPSO'*100,connDNG'*100], ...
    'Labels',{'AVF','FCO','GWO-3D','FuzzyDet','HybridPSO','MODOA'});
ylabel('Connectivity (%)'); grid on; title('Connectivity');
subplot(1,3,3);
boxplot([timeAVF',timeFIS',timeGWO',timeFUZ',timePSO',timeDNG'], ...
    'Labels',{'AVF','FCO','GWO-3D','FuzzyDet','HybridPSO','MODOA'});
ylabel('Runtime (s)'); grid on; title('Runtime');
sgtitle('Final Coverage, Connectivity, and Runtime Across Runs');

% 3D before/after and heatmaps (best run per method)
plot3DResultsModel('AVF (Best)',bestOptAVF.initPos,bestOptAVF.optPos, ...
                   bestOptAVF.initRout,bestOptAVF.optRout,bestOptAVF.sRad, ...
                   bestOptAVF.rRad,areaSize,obstacles);
plot3DResultsModel('FCO (Best)',bestOptFIS.initPos,bestOptFIS.optPos, ...
                   bestOptFIS.initRout,bestOptFIS.optRout,bestOptFIS.sRad, ...
                   bestOptFIS.rRad,areaSize,obstacles);
plot3DResultsModel('GWO-3D (Best)',bestOptGWO.initPos,bestOptGWO.optPos, ...
                   bestOptGWO.initRout,bestOptGWO.optRout,bestOptGWO.sRad, ...
                   bestOptGWO.rRad,areaSize,obstacles);
plot3DResultsModel('FuzzyDet (Best)',bestOptFUZ.initPos,bestOptFUZ.optPos, ...
                   bestOptFUZ.initRout,bestOptFUZ.optRout,bestOptFUZ.sRad, ...
                   bestOptFUZ.rRad,areaSize,obstacles);
plot3DResultsModel('HybridPSO (Best)',bestOptPSO.initPos,bestOptPSO.optPos, ...
                   bestOptPSO.initRout,bestOptPSO.optRout,bestOptPSO.sRad, ...
                   bestOptPSO.rRad,areaSize,obstacles);
plot3DResultsModel('MODOA (Best)',bestOptDNG.initPos,bestOptDNG.optPos, ...
                   bestOptDNG.initRout,bestOptDNG.optRout,bestOptDNG.sRad, ...
                   bestOptDNG.rRad,areaSize,obstacles);

[Hxa, Hya, Ha] = coverageHeatmap([bestOptAVF.optPos; bestOptAVF.optRout], [bestOptAVF.sRad,bestOptAVF.rRad], areaSize, obstacles);
[Hxf, Hyf, Hf] = coverageHeatmap([bestOptFIS.optPos; bestOptFIS.optRout], [bestOptFIS.sRad,bestOptFIS.rRad], areaSize, obstacles);
[Hxg, Hyg, Hg] = coverageHeatmap([bestOptGWO.optPos; bestOptGWO.optRout], [bestOptGWO.sRad,bestOptGWO.rRad], areaSize, obstacles);
[Hxz, Hyz, Hz] = coverageHeatmap([bestOptFUZ.optPos; bestOptFUZ.optRout], [bestOptFUZ.sRad,bestOptFUZ.rRad], areaSize, obstacles);
[Hxp, Hyp, Hp] = coverageHeatmap([bestOptPSO.optPos; bestOptPSO.optRout], [bestOptPSO.sRad,bestOptPSO.rRad], areaSize, obstacles);
[Hxd, Hyd, Hd] = coverageHeatmap([bestOptDNG.optPos; bestOptDNG.optRout], [bestOptDNG.sRad,bestOptDNG.rRad], areaSize, obstacles);

figure('Name','Coverage Heatmaps (Max over Z)','Color','w');
subplot(2,3,1); imagesc(Hxa, Hya, Ha'); axis xy equal tight; colorbar; title('AVF - Heatmap'); xlabel('X'); ylabel('Y');
subplot(2,3,2); imagesc(Hxf, Hyf, Hf'); axis xy equal tight; colorbar; title('FCO - Heatmap'); xlabel('X'); ylabel('Y');
subplot(2,3,3); imagesc(Hxg, Hyg, Hg'); axis xy equal tight; colorbar; title('GWO-3D - Heatmap'); xlabel('X'); ylabel('Y');
subplot(2,3,4); imagesc(Hxz, Hyz, Hz'); axis xy equal tight; colorbar; title('FuzzyDet - Heatmap'); xlabel('X'); ylabel('Y');
subplot(2,3,5); imagesc(Hxp, Hyp, Hp'); axis xy equal tight; colorbar; title('HybridPSO - Heatmap'); xlabel('X'); ylabel('Y');
subplot(2,3,6); imagesc(Hxd, Hyd, Hd'); axis xy equal tight; colorbar; title('MODOA - Heatmap'); xlabel('X'); ylabel('Y');

% --------- Auto-save all open figures -----------
saveAllFigures(outDir);

%% ======================= FUNCTIONS ======================================
function P = jitterOnStreets(P, sigma, area)
    P = P + sigma*randn(size(P));
    P(:,1) = min(max(P(:,1), 0), area(1));
    P(:,2) = min(max(P(:,2), 0), area(2));
    P(:,3) = zeros(size(P,1),1);
end

% ====================== PROGRESS UTIL ====================================
function progressFcn = makeProgressFcn(tag, runIdx, iters, useWaitbar)
    t0 = tic; lastPrint = 0; h = [];
    if nargin<4, useWaitbar = false; end
    if useWaitbar && usejava('desktop')
        h = waitbar(0, sprintf('%s r=%d', tag, runIdx), 'Name','Progress');
    end
    progressFcn = @update;
    function update(it, cov)
        frac = max(0, min(1, it/iters));
        elapsed = toc(t0);
        rem = elapsed*(1-frac)/max(frac,1e-9);
        if (it - lastPrint) >= 1
            fprintf('%s r=%d: %3d/%3d it (%.0f%%) cov=%.2f%% ETA %s    \r', ...
                tag, runIdx, it, iters, 100*frac, cov, secs2hms(rem));
            lastPrint = it;
        end
        if ~isempty(h) && isvalid(h)
            waitbar(frac, h, sprintf('%s r=%d | %3d/%3d | cov=%.2f%% | ETA %s', ...
                tag, runIdx, it, iters, cov, secs2hms(rem)));
            drawnow limitrate nocallbacks
        end
        if it==iters
            fprintf('\n'); if ~isempty(h) && isvalid(h), close(h); end
        end
    end
end
function s = secs2hms(sec)
    sec = max(0, sec);
    h = floor(sec/3600); sec = sec - 3600*h;
    m = floor(sec/60);   sec = sec - 60*m;
    s = sprintf('%02d:%02d:%02d', h, m, round(sec));
end

% ------------------------ AVF OPTIMIZER --------------- (renamed Original)
function [optimizedPositions,optimizedRouterPositions,covHist] = ...
         optimizeSensorRouterLocationsAVF(initPos,initRout, sRad,rRad,area,obs,iters, progressFcn)

    if nargin < 8 || isempty(progressFcn), progressFcn = @(it,cov) []; end

    nS = size(initPos,1); nR = size(initRout,1);
    curPos  = initPos;  curRout = initRout;

    covHist = zeros(iters+1,1);
    covHist(1) = calculateCoverage([curPos;curRout],[sRad,rRad],area,obs);

    alphaS0 = 0.55; alphaR0 = 0.55;
    momS = zeros(nS,3); momR = zeros(nR,3); beta = 0.45;

    bestCov  = covHist(1); bestState = {curPos,curRout}; stallCnt = 0;

    for it = 1:iters
        t = it/iters;
        alphaS = alphaS0*(1-0.65*t);
        alphaR = alphaR0*(1-0.65*t);

        fS = zeros(nS,3); fR = zeros(nR,3);
        % Sensors
        for i = 1:nS
            totalF = [0 0 0];
            for j = 1:nS
                if i~=j, totalF = totalF + calculateForce(curPos(i,:),curPos(j,:),sRad(i)); end
            end
            for r = 1:nR
                totalF = totalF + calculateForce(curPos(i,:),curRout(r,:),rRad(r));
            end
            for k = 1:size(obs,1)
                totalF = totalF + calculateForceFromObstacle(curPos(i,:),obs(k,:),sRad(i));
            end
            totalF = totalF + calculateAdaptiveForce(curPos(i,:),sRad(i),area,obs);
            fS(i,:) = totalF;
        end
        % Routers
        for i = 1:nR
            totalF = [0 0 0];
            for j = 1:nR
                if i~=j, totalF = totalF + calculateForce(curRout(i,:),curRout(j,:),rRad(i)); end
            end
            for k = 1:size(obs,1)
                totalF = totalF + calculateForceFromObstacle(curRout(i,:),obs(k,:),rRad(i));
            end
            totalF = totalF + calculateAdaptiveForce(curRout(i,:),rRad(i),area,obs);
            fR(i,:) = totalF;
        end
        momS = beta*momS + (1-beta)*fS;
        momR = beta*momR + (1-beta)*fR;
        curPos  = adjustPositions(curPos  + alphaS*momS, area);
        curRout = adjustPositions(curRout + alphaR*momR, area);

        % Keep sensors within at least one router
        for i = 1:nS
            inside = any(vecnorm(curRout - curPos(i,:),2,2) <= rRad');
            if ~inside
                [~,idx] = min(vecnorm(curRout - curPos(i,:),2,2));
                dir     = curPos(i,:) - curRout(idx,:);
                dn = norm(dir);
                if dn>0, dir = dir/dn; curPos(i,:) = curRout(idx,:) + dir * rRad(idx)*0.9; end
            end
        end

        covHist(it+1) = calculateCoverage([curPos;curRout],[sRad,rRad],area,obs);
        progressFcn(it, covHist(it+1));

        if covHist(it+1) > bestCov + 1e-6
            bestCov = covHist(it+1); bestState = {curPos,curRout}; stallCnt = 0;
        else
            stallCnt = stallCnt + 1;
            if stallCnt > 25
                curPos  = adjustPositions(curPos  + 0.8*randn(size(curPos)), area);
                curRout = adjustPositions(curRout + 0.6*randn(size(curRout)), area);
                stallCnt = 0;
            end
        end
    end
    optimizedPositions       = bestState{1};
    optimizedRouterPositions = bestState{2};
end

% ------------------------ FIS OPTIMIZER  (StepFIS + SteerFIS) ------------
function [optS,optR,covHist] = optimizeSensorRouterLocationsFIS_Pro( ...
    initPos,initRout, baseSR,baseRR, area,obs,iters, progressFcn)

    if nargin < 9 || isempty(progressFcn), progressFcn = @(it,cov) []; end

    % ---------- Setup ----------
    nS = numel(baseSR); nR = numel(baseRR);
    curS = initPos; curR = initRout;
    curSR = baseSR(:)'; curRR = baseRR(:)';

    % FIS controllers
    fisStep  = createStepFIS();
    persistent fisSteer
    if isempty(fisSteer), fisSteer = createSteerFIS(); end

    % Schedules
    covHist  = zeros(iters+1,1);
    covHist(1) = calculateCoverage([curS;curR],[curSR,curRR],area,obs);

    alphaS0  = 0.68; alphaR0  = 0.62;
    beta     = 0.48; 
    momS     = zeros(nS,3); momR = zeros(nR,3);
    bestCov  = covHist(1); bestS  = curS; bestR  = curR;
    stallCnt = 0;

    % Targeting params
    Kmax       = nS + nR + 2; 
    MsampleHi  = 9000;
    MsampleLo  = 5000;

    % Connectivity maintenance
    KRepair    = 5;          
    edgeFrac   = 0.85;       
    dampBand   = 0.25;       
    minDamp    = 0.35;       

    % FCM options (fallback to k-means if fcm not available)
    useFCM     = (exist('fcm','file')==2);
    fcmExpo    = 2.0;       
    fcmMaxIter = 80;        
    fcmTol     = 1e-5;      
    fcmDisp    = 0;         
    fcmOptions = [fcmExpo, fcmMaxIter, fcmTol, fcmDisp];

    for it = 1:iters
        t = it/iters;
        alphaS = alphaS0*(1-0.6*t);
        alphaR = alphaR0*(1-0.6*t);

        % ---------- Global uncovered targets ----------
        if it <= round(0.4*iters)
            replanEvery = 1; Msample = MsampleHi;
        else
            replanEvery = 3; Msample = MsampleLo;
        end
        if mod(it-1,replanEvery)==0
            samples = sampleUncovered([curS;curR],[curSR,curRR],area,obs,Msample);
            if ~isempty(samples)
                K = min(Kmax, max(2, round(size(samples,1)/450)));
                if useFCM
                    [C, U] = fcm(samples, K, fcmOptions);  % C: K-by-3, U: K-by-N
                    demand = sum(U.^fcmExpo, 2);
                else
                    [idx, C] = kmeans(samples, K, 'MaxIter',80, 'Replicates',2, 'Distance','sqeuclidean'); %#ok<ASGLU>
                    demand = accumarray(idx,1,[K,1]);
                end
                demand = demand / max(demand+eps);
                nodes  = [curS; curR];
                Cuse=C; demUse=demand;
                while size(Cuse,1) < size(nodes,1)
                    [~,ix]=max(demUse);
                    Cuse  = [Cuse;  Cuse(ix,:)];
                    demUse= [demUse; demUse(ix)];
                end
                lambda = 0.45;
                cost = pdist2(nodes, Cuse) - lambda*(ones(size(nodes,1),1)*demUse');
                cost = cost - min(cost,[],'all') + 1e-6;
                assign = hungarian(cost);
                targets = Cuse(assign,:);
                targS = targets(1:nS,:);      demS = demUse(assign(1:nS));
            else
                targS = curS; demS = 0.2*ones(nS,1);
            end
        end

        % ---------- SENSOR MOTION (with SteerFIS) ----------
        fS = zeros(nS,3);
        for i=1:nS
            % Step size from StepFIS (distance & demand)
            dCent = min(norm(targS(i,:)-curS(i,:))/max(2*curSR(i),1e-6),1);
            dc = min(max(dCent, 0), 1);
            dm = min(max(demS(i), 0), 1);
            if ~isfinite(dc), dc = 0.5; end
            if ~isfinite(dm), dm = 0.5; end
            stepScale = evalfis(fisStep,[dc, dm]);

            % Steering magnitudes from SteerFIS, directions from geometry
            [edgeNorm, edgeDir] = edgeProximityAndDirection(curS(i,:),area,curSR(i));
            [obstNorm, obstDir] = obstacleProximityAndDirection(curS(i,:),obs,curSR(i));
            en = min(max(edgeNorm,0),1);
            on = min(max(obstNorm,0),1);
            sout = evalfis(fisSteer,[en on]);              % [steerE, steerO]
            steerEdge = sout(1) * edgeDir;
            steerObst = sout(2) * obstDir;

            % direction to target + local coverage-gain
            vTarget = targS(i,:) - curS(i,:); 
            nt = norm(vTarget); 
            if nt>0, vTarget=vTarget/nt; end
            fGain   = localDirectionalGainForce(curS(i,:), i, curS, curR, curSR, curRR, area, obs);

            % link-aware step damping near/outside router edge
            d = vecnorm(curR - curS(i,:),2,2);
            [dmin,ir] = min(d);
            linkRatio = dmin / max(curRR(ir),1e-6);  % 1.0 == on edge
            if linkRatio > edgeFrac
                raw = 1 - max(0,(linkRatio - edgeFrac)/dampBand);
                stepDamp = max(minDamp, min(1, raw));
                stepScale = stepScale * stepDamp;
            end

            % compose force
            k1=2.4; k2=0.7; k3=0.9; k4=0.8;
            fS(i,:) = k1*stepScale*vTarget + k2*steerEdge + k3*steerObst + k4*fGain;
        end

        % ---------- ROUTER MOTION (geometric steer) ----------
        fR = zeros(nR,3);
        assignSR = nearestRouterIdx(curS,curR);
        for r=1:nR
            idxS = find(assignSR==r);
            if isempty(idxS)
                cen = median(curS,1);
            else
                dS  = vecnorm(curS(idxS,:) - curR(r,:),2,2);
                w   = 1 + (dS/max(curRR(r),1)).^2;
                cen = sum(curS(idxS,:).*w,1)/(sum(w)+eps);
            end
            vC = cen - curR(r,:); 
            nv = norm(vC); if nv>0, vC=vC/nv; end
            kC=2.1; 
            steerEdgeR = localEdgeSteer(curR(r,:),area,curRR(r));
            steerObstR = localObstSteer(curR(r,:),obs,curRR(r));
            fR(r,:) = kC*vC + 0.6*steerEdgeR + 0.9*steerObstR;
        end

        % ---------- Integrate with momentum ----------
        momS = beta*momS + (1-beta)*fS;
        momR = beta*momR + (1-beta)*fR;
        curS  = adjustPositions(curS  + 0.5*alphaS*momS, area);
        curR  = adjustPositions(curR  + 0.5*alphaR*momR, area);

        % ---------- Periodic connectivity repair ----------
        if mod(it, KRepair)==0
            for i=1:nS
                d = vecnorm(curR - curS(i,:),2,2);
                [dmin,ir] = min(d);
                if dmin > curRR(ir)
                    v = curS(i,:) - curR(ir,:); 
                    nv = norm(v);
                    if nv>0
                        v = v/nv;
                        curS(i,:) = curR(ir,:) + v*(0.95*curRR(ir));
                    end
                end
            end
        end

        % ---------- Coverage & book-keeping ----------
        covHist(it+1) = calculateCoverage([curS;curR],[curSR,curRR],area,obs);
        progressFcn(it, covHist(it+1));

        if covHist(it+1) > bestCov + 1e-6
            bestCov = covHist(it+1); bestS = curS; bestR = curR; stallCnt = 0;
        else
            stallCnt = stallCnt + 1;
            if stallCnt > 12
                curS = adjustPositions(curS + 0.25*randn(size(curS)), area);
                curR = adjustPositions(curR + 0.20*randn(size(curR)), area);
                stallCnt = 0;
            end
        end
    end

    % ---------- Polishing ----------
    curS = bestS; curR = bestR;
    for p=1:20
        for i=1:nS
            fGain = localDirectionalGainForce(curS(i,:), i, curS, curR, curSR, curRR, area, obs);
            curS(i,:) = adjustPositions(curS(i,:) + 0.35*fGain, area);
        end
    end

    % ---------- Final connectivity repair ----------
    for i=1:nS
        d = vecnorm(curR - curS(i,:),2,2);
        [dmin,ir] = min(d);
        if dmin > curRR(ir)
            v = curS(i,:) - curR(ir,:); 
            nv = norm(v);
            if nv>0
                v = v/nv;
                curS(i,:) = curR(ir,:) + v*(0.95*curRR(ir));
            end
        end
    end

    optS = curS; optR = curR;
end

% ------------------------ GWO-3D -----------------------------------------
function [optS,optR,covHist] = optimizeSensorRouterLocationsGWO3D( ...
    initPos,initRout,sRad,rRad,area,obs,iters, progressFcn)

    if nargin < 9 || isempty(progressFcn), progressFcn = @(it,cov) []; end

    nS = size(initPos,1); nR = size(initRout,1);
    nVars = 3*(nS+nR);

    nWolves = 10;
    pop = zeros(nWolves,nVars);
    lb  = zeros(1,nVars);
    ub  = [repmat(area,1,nS), repmat(area,1,nR)];

    X0 = [initPos(:); initRout(:)]';
    for w=1:nWolves
        jitter = 0.5*randn(size(X0));
        pop(w,:) = min(max(X0 + jitter, lb), ub);
    end

    lambdaConn = 15; % penalty %points
    function [fit,cov,conn,XS,XR] = evalFit(X)
        S = reshape(X(1:3*nS), [nS,3]);
        R = reshape(X(3*nS+1:end), [nR,3]);
        S = clamp3D(S,area); R = clamp3D(R,area);
        cov = calculateCoverage([S;R],[sRad,rRad],area,obs);
        conn = connectivityMetric(S,R,rRad);
        fit = cov - lambdaConn*(1-conn)*100;  % higher is better
        XS=S; XR=R;
    end

    covHist = zeros(iters+1,1);
    [~,cov0,~,~,~] = evalFit(pop(1,:));
    covHist(1)=cov0;

    fits  = zeros(nWolves,1);
    Skeep = cell(nWolves,1); Rkeep = cell(nWolves,1);
    for w=1:nWolves
        [fits(w),~,~,Skeep{w},Rkeep{w}] = evalFit(pop(w,:));
    end
    [~, ia] = max(fits); XA = pop(ia,:);
    fitsA = fits; fitsA(ia) = -inf;
    [~, ib] = max(fitsA); XB = pop(ib,:);
    fitsB = fitsA; fitsB(ib) = -inf;
    [~, id] = max(fitsB); XD = pop(id,:);

    for it=1:iters
        a = 2 - 2*(it/iters); % linearly decreases 2->0
        for w=1:nWolves
            r1=rand(1,nVars); r2=rand(1,nVars);
            A1 = 2*a.*r1 - a; C1 = 2.*r2;
            Dalpha = abs(C1.*XA - pop(w,:));
            X1 = XA - A1.*Dalpha;

            r1=rand(1,nVars); r2=rand(1,nVars);
            A2 = 2*a.*r1 - a; C2 = 2.*r2;
            Dbeta = abs(C2.*XB - pop(w,:));
            X2 = XB - A2.*Dbeta;

            r1=rand(1,nVars); r2=rand(1,nVars);
            A3 = 2*a.*r1 - a; C3 = 2.*r2;
            Ddelta = abs(C3.*XD - pop(w,:));
            X3 = XD - A3.*Ddelta;

            newX = (X1 + X2 + X3)/3;
            newX = min(max(newX,lb),ub);
            pop(w,:) = newX;
        end

        for w=1:nWolves
            [fits(w),cov,~,Skeep{w},Rkeep{w}] = evalFit(pop(w,:)); %#ok<ASGLU>
        end
        [~, ia] = max(fits); XA = pop(ia,:);
        fitsA = fits; fitsA(ia) = -inf;
        [~, ib] = max(fitsA); XB = pop(ib,:);
        fitsB = fitsA; fitsB(ib) = -inf;
        [~, id] = max(fitsB); XD = pop(id,:);

        [~,covA,~,~,~] = evalFit(XA);
        covHist(it+1) = covA;
        progressFcn(it, covHist(it+1));
    end

    [~,bestIdx] = max(fits);
    optS = clamp3D(Skeep{bestIdx},area);
    optR = clamp3D(Rkeep{bestIdx},area);
end

% ------------------------ FuzzyDet ---------------------------------------
function [optS,optR,covHist] = optimizeSensorRouterLocationsFuzzyDet( ...
    initPos,initRout,sRad,rRad,area,obs,iters, progressFcn)

    if nargin < 9 || isempty(progressFcn), progressFcn = @(it,cov) []; end
    nS=size(initPos,1); nR=size(initRout,1);

    [seedS, seedR] = greedySeedSensorsRoutersSeparated(area, obs, nS, nR, sRad, rRad);

    curS = seedS; curR = seedR;
    steps = min(30, iters);
    covHist = zeros(iters+1,1);
    covHist(1) = calculateCoverage([curS;curR],[sRad,rRad],area,obs);
    for p=1:steps
        for i=1:nS
            fGain = localDirectionalGainForce(curS(i,:), i, curS, curR, sRad, rRad, area, obs);
            curS(i,:) = adjustPositions(curS(i,:) + 0.25*fGain, area);
        end
        idxR = nearestRouterIdx(curS,curR);
        for rr=1:nR
            Ii = find(idxR==rr);
            if ~isempty(Ii), curR(rr,:) = median(curS(Ii,:),1); end
        end
        covHist(p+1) = calculateCoverage([curS;curR],[sRad,rRad],area,obs);
        progressFcn(p, covHist(p+1));
    end
    if steps<iters
        covHist(steps+2:end) = covHist(steps+1);
        progressFcn(iters, covHist(end));
    end
    optS = curS; optR = curR;
end

% ------------------------ HybridPSO --------------------------------------
function [optS,optR,covHist] = optimizeSensorRouterLocationsHybridPSO( ...
    initPos,initRout,sRad,rRad,area,obs,iters, progressFcn)

    if nargin < 9 || isempty(progressFcn), progressFcn = @(it,cov) []; end
    nS = size(initPos,1); nR = size(initRout,1);
    nVars = 3*(nS+nR);
    lb  = zeros(1,nVars);
    ub  = [repmat(area,1,nS), repmat(area,1,nR)];

    swarmSize = 14;
    wInertia  = 0.72; c1 = 1.6; c2 = 1.6;
    Vmax = 3.5;

    X0 = [initPos(:); initRout(:)]';
    X  = zeros(swarmSize, nVars);
    V  = zeros(swarmSize, nVars);
    for p=1:swarmSize
        X(p,:) = min(max(X0 + 0.8*randn(size(X0)), lb), ub);
        V(p,:) = 0.2*randn(1,nVars);
    end

    lambdaConn = 15; % penalty weight for connectivity
    function [fit,cov,conn,XS,XR] = evalFit(x)
        S = reshape(x(1:3*nS), [nS,3]);
        R = reshape(x(3*nS+1:end), [nR,3]);
        S = clamp3D(S,area); R = clamp3D(R,area);
        cov = calculateCoverage([S;R],[sRad,rRad],area,obs);
        conn = connectivityMetric(S,R,rRad);
        fit = cov - lambdaConn*(1-conn)*100;
        XS=S; XR=R;
    end

    pBest = X; pBestFit = -inf(swarmSize,1);
    gBest = X(1,:); gBestFit = -inf;

    covHist = zeros(iters+1,1);
    [~,cov0,~,~,~] = evalFit(X(1,:)); covHist(1)=cov0;

    for it=1:iters
        for p=1:swarmSize
            r1 = rand(1,nVars); r2 = rand(1,nVars);
            V(p,:) = wInertia*V(p,:) ...
                + c1*r1.*(pBest(p,:) - X(p,:)) ...
                + c2*r2.*(gBest      - X(p,:));
            V(p,:) = max(min(V(p,:), Vmax), -Vmax);
            X(p,:) = X(p,:) + V(p,:);
            X(p,:) = min(max(X(p,:), lb), ub);

            [fit,~,~,~,~] = evalFit(X(p,:));
            if fit > pBestFit(p), pBestFit(p) = fit; pBest(p,:) = X(p,:); end
            if fit > gBestFit,    gBestFit = fit;   gBest      = X(p,:); end
        end
        if mod(it, max(5,round(iters/30)))==0
            X = 0.85*X + 0.15*(gBest + 0.5*randn(size(X)));
            X = min(max(X,lb),ub);
        end
        [~,covg,~,~,~] = evalFit(gBest);
        covHist(it+1) = covg;
        progressFcn(it, covHist(it+1));
    end

    [~,~,~,optS,optR] = evalFit(gBest);
    optS = clamp3D(optS,area); optR = clamp3D(optR,area);
end

function [optS,optR,covHist] = optimizeSensorRouterLocationsDingo3D( ...
    initPos,initRout,sRad,rRad,area,obs,iters, progressFcn)
% MODOA-flavored (single-objective) optimizer for 3D sensor + router placement.
% Same I/O as your original function. Keeps coverage objective with connectivity
% penalty, but updates positions using DOA strategies:
%   - Group Attack, Persecution, Scavenger (chosen with probs P and Q)
% Also adds:
%   - Elite archive (best non-worse solutions for single objective)
%   - Survival (refresh weakest few near elites)
%
% Dependencies (same as before): clamp3D, calculateCoverage, connectivityMetric.

    if nargin < 9 || isempty(progressFcn), progressFcn = @(it,cov) []; end

    % --- Problem sizes
    nS = size(initPos,1); 
    nR = size(initRout,1);
    nVars = 3*(nS+nR);

    % --- Bounds (area = [xmax ymax zmax])
    lb  = zeros(1,nVars);
    ub  = [repmat(area,1,nS), repmat(area,1,nR)];

    % --- Pack / DOA parameters (from paper-style defaults)
    pack         = 18;       % number of dingoes (population)
    P_strat      = 0.5;      % switch to hunting (Group/Persecution) vs Scavenger
    Q_group      = 0.7;      % within hunting: Group Attack vs Persecution
    lambdaConn   = 15;       % connectivity penalty weight (as you had)
    archiveFrac  = 0.25;     % elite archive fraction
    survivalFrac = 0.15;     % fraction of weakest to refresh
    jitterEsc    = 0.02;     % small jitter to escape local minima

    % Annealing of exploration/exploitation (similar spirit to your code)
    etaExpl0 = 0.7;    % initial exploration amplitude
    etaExpl1 = 0.15;   % final   exploration amplitude
    etaHunt0 = 0.5;    % initial hunting amplitude
    etaHunt1 = 1.0;    % final   hunting amplitude

    % --- Fitness helper (higher is better)
    function [fit,cov,conn,XS,XR] = evalFit(x)
        S = reshape(x(1:3*nS), [nS,3]);
        R = reshape(x(3*nS+1:end), [nR,3]);
        S = clamp3D(S,area); R = clamp3D(R,area);
        cov = calculateCoverage([S;R],[sRad,rRad],area,obs);
        conn = connectivityMetric(S,R,rRad);
        fit = cov - lambdaConn*(1-conn)*100;
        XS=S; XR=R;
    end

    % --- Initialize pack around start with jitter
    X0 = [initPos(:); initRout(:)]';
    X  = zeros(pack, nVars);
    for k=1:pack
        X(k,:) = min(max(X0 + 0.9*randn(size(X0)), lb), ub);
    end

    % --- Evaluate, build initial archive
    fits  = zeros(pack,1);
    covs  = zeros(pack,1);
    for k=1:pack
        [fits(k),covs(k)] = evalFit(X(k,:));
    end
    [fits,ord] = sort(fits,'descend'); %#ok<ASGLU>
    X = X(ord,:);
    covs = covs(ord);

    % Elite archive (single-objective: keep top K)
    Karch = max(3, round(archiveFrac*pack));
    archiveX = X(1:Karch,:);
    archiveF = fits(1:Karch);

    % Leaders from archive
    A = archiveX(1,:);        % alpha
    B = archiveX(min(2,Karch),:);
    D = archiveX(min(3,Karch),:);

    % --- Progress
    covHist = zeros(iters+1,1);
    [~,cov0] = evalFit(A); covHist(1)=cov0;

    % ===== Main loop =====
    for it=1:iters
        t  = it/max(1,iters);
        aExpl = etaExpl0*(1-t) + etaExpl1*t;
        aHunt = etaHunt0*(1-t) + etaHunt1*t;

        % --- Strategy selection per agent
        for idx=1:pack
            rMain = rand();
            if rMain < P_strat
                % Hunting branch: choose Group Attack or Persecution
                if rand() < Q_group
                    % -------- Strategy 1: Group Attack --------
                    % Move towards cooperative center of leaders with adaptive step
                    r1 = rand(1,nVars); r2 = rand(1,nVars);
                    center = (A + B + D)/3;
                    dist   = abs(center - X(idx,:));
                    step   = aHunt * (2*r1 - 1) .* dist ...
                          + aExpl * (r2 - 0.5) .* (ub - lb);
                    newX   = X(idx,:) + step;

                else
                    % -------- Strategy 2: Persecution --------
                    % Pursue alpha with encircling (A - a * |C * A - X|)
                    r1 = rand(1,nVars); r2 = rand(1,nVars); r3 = rand(1,nVars);
                    C  = 2*r1;                  % coefficient vector
                    Dv = abs(C.*A - X(idx,:));  % distance to prey (alpha)
                    encircle = A - aHunt*(2*r2-1).*Dv;
                    tug      = 0.25*aExpl*(B - X(idx,:)) + 0.15*aExpl*(D - X(idx,:));
                    newX     = encircle + tug + jitterEsc*(rand(1,nVars)-0.5).*(ub-lb);
                end
            else
                % -------- Strategy 3: Scavenger --------
                % Explore around random rival and random archive leader
                rivalIdx   = randi(pack);
                archIdx    = randi(Karch);
                rival      = X(rivalIdx,:);
                leader     = archiveX(archIdx,:);
                r1 = rand(1,nVars); r2 = rand(1,nVars); r3 = rand(1,nVars);

                drift = aExpl * (r1 - 0.5) .* (ub - lb);           % free roam
                pullL = 0.6*aExpl * (leader - X(idx,:)) .* r2;     % soft pull to archive
                pullR = 0.4*aExpl * (rival  - X(idx,:)) .* r3;     % soft pull to rival
                newX  = X(idx,:) + drift + pullL + pullR;
            end

            % Bound
            X(idx,:) = min(max(newX, lb), ub);
        end

        % --- Survival: refresh weakest few near elites (jump out of traps)
        sCnt = max(1, round(survivalFrac*pack));
        % Re-evaluate population to find weakest
        for k=1:pack
            [fits(k),covs(k)] = evalFit(X(k,:));
        end
        [~,ord] = sort(fits,'descend');
        X = X(ord,:); fits = fits(ord); covs = covs(ord);

        weakIdx = (pack-sCnt+1):pack;
        for j = weakIdx
            elite = archiveX(randi(Karch),:);
            noise = 0.2*(rand(1,nVars)-0.5).*(ub-lb);
            X(j,:) = min(max(elite + noise, lb), ub);
        end

        % --- Evaluate after survival
        for k=1:pack
            [fits(k),covs(k)] = evalFit(X(k,:));
        end
        [fits,ord] = sort(fits,'descend');
        X = X(ord,:); covs = covs(ord);

        % --- Update archive (top-K replacement for single objective)
        poolX = [archiveX; X];           % candidate pool
        poolF = [archiveF; fits];
        [poolF, pOrd] = sort(poolF,'descend');
        poolX = poolX(pOrd,:);
        archiveX = poolX(1:Karch,:);
        archiveF = poolF(1:Karch);

        % --- Update leaders from archive
        A = archiveX(1,:);
        B = archiveX(min(2,Karch),:);
        D = archiveX(min(3,Karch),:);

        % --- Record progress with best coverage
        covHist(it+1) = covs(1);
        progressFcn(it, covHist(it+1));
    end

    % --- Return best
    bestX = archiveX(1,:);
    [~,~,~,optS,optR] = evalFit(bestX);
    optS = clamp3D(optS,area); 
    optR = clamp3D(optR,area);
end


% ------------------------ FIS CONTROLLERS --------------------------------
% StepFIS: step size from (distance-to-target, demand) on [0,1]
function fis = createStepFIS()
    fis = sugfis('Name','StepFIS');
    fis = addInput(fis,[0 1],'Name','d');      % normalized distance
    fis = addMF(fis,'d','trimf',[-0.2 0 0.4],'Name','near');
    fis = addMF(fis,'d','trimf',[0.2 0.55 0.9],'Name','mid');
    fis = addMF(fis,'d','trimf',[0.7 1 1.2],'Name','far');

    fis = addInput(fis,[0 1],'Name','dem');    % demand
    fis = addMF(fis,'dem','trimf',[-0.2 0 0.4],'Name','low');
    fis = addMF(fis,'dem','trimf',[0.2 0.55 0.9],'Name','mid');
    fis = addMF(fis,'dem','trimf',[0.7 1 1.2],'Name','high');

    fis = addOutput(fis,[0 1],'Name','step');
    fis = addMF(fis,'step','constant',0.12,'Name','sml');
    fis = addMF(fis,'step','constant',0.55,'Name','med');
    fis = addMF(fis,'step','constant',1.00,'Name','big');

    rules = [
        1 1 1 1 1
        1 2 2 1 1
        1 3 2 1 1
        2 1 1 1 1
        2 2 2 1 1
        2 3 2 1 1
        3 1 2 1 1
        3 2 3 1 1
        3 3 3 1 1
    ];
    fis = addRule(fis, rules);
end

% SteerFIS: steering magnitudes from (edge proximity, obstacle proximity)
function fis = createSteerFIS()
    fis = sugfis('Name','SteerFIS');

    fis = addInput(fis,[0 1],'Name','edge');
    fis = addMF(fis,'edge','trimf',[-0.2 0 0.4],'Name','low');
    fis = addMF(fis,'edge','trimf',[0.2 0.55 0.9],'Name','mid');
    fis = addMF(fis,'edge','trimf',[0.7 1 1.2],'Name','high');

    fis = addInput(fis,[0 1],'Name','obst');
    fis = addMF(fis,'obst','trimf',[-0.2 0 0.4],'Name','low');
    fis = addMF(fis,'obst','trimf',[0.2 0.55 0.9],'Name','mid');
    fis = addMF(fis,'obst','trimf',[0.7 1 1.2],'Name','high');

    fis = addOutput(fis,[0 1],'Name','steerE');
    fis = addMF(fis,'steerE','constant',0.0,'Name','zero');
    fis = addMF(fis,'steerE','constant',0.8,'Name','pos');

    fis = addOutput(fis,[0 1],'Name','steerO');
    fis = addMF(fis,'steerO','constant',0.0,'Name','zero');
    fis = addMF(fis,'steerO','constant',1.0,'Name','pos');

    % complete 3×3 rule grid
    % [edge obst steerE steerO weight AND]
    rules = [
        1 1 1 1 1 1;  1 2 1 2 1 1;  1 3 1 2 1 1;
        2 1 2 1 1 1;  2 2 2 2 1 1;  2 3 1 2 1 1;
        3 1 2 1 1 1;  3 2 2 2 1 1;  3 3 2 2 1 1
    ];
    fis = addRule(fis, rules);
end

% ------------------------ LOCAL STEERS / ESTIMATES -----------------------
function steer = localEdgeSteer(p,area,r)
    [edgeNorm, edgeDir] = edgeProximityAndDirection(p,area,r);
    steer = 0.8*edgeNorm*edgeDir;
end
function steer = localObstSteer(p,obs,r)
    [obstNorm, obstDir] = obstacleProximityAndDirection(p,obs,r);
    steer = 1.0*obstNorm*obstDir;
end

% Reduced samples for speed
function f = localDirectionalGainForce(pos, idx, sensorPos, routerPos, sRad, rRad, area, obs)
    rad = sRad(idx);
    S = 450; Rmax = 1.6*rad;
    U = randn(S,3); U = U ./ max(sqrt(sum(U.^2,2)),1e-12);
    R = Rmax * (rand(S,1).^(1/3));
    pts = pos + U .* R;

    pts(:,1) = min(max(pts(:,1),0),area(1));
    pts(:,2) = min(max(pts(:,2),0),area(2));
    pts(:,3) = min(max(pts(:,3),0),area(3));

    if ~isempty(obs)
        keep = true(size(pts,1),1);
        for k=1:size(obs,1)
            o = obs(k,:);
            inside = pts(:,1)>=o(1) & pts(:,1)<=o(4) & ...
                     pts(:,2)>=o(2) & pts(:,2)<=o(5) & ...
                     pts(:,3)>=o(3) & pts(:,3)<=o(6);
            keep = keep & ~inside;
        end
        pts = pts(keep,:);
    end
    if isempty(pts), f = [0 0 0]; return; end
    S = size(pts,1);

    nS = size(sensorPos,1);
    othersIdx = setdiff(1:nS, idx);
    othersPos = [sensorPos(othersIdx,:); routerPos];
    tmpRad = sRad(othersIdx);
    othersRad = [tmpRad(:); rRad(:)];

    if isempty(othersPos)
        coveredByOthers = false(S,1);
    else
        dmat = pdist2(pts, othersPos);
        radMat = repmat(othersRad', S, 1);
        coveredByOthers = any(dmat <= radMat, 2);
    end

    g = [-1 0 1];
    [dx,dy,dz] = ndgrid(g,g,g);
    D = [dx(:),dy(:),dz(:)];
    D = D(any(D,2),:);
    D = D ./ max(vecnorm(D,2,2),1e-12);

    bestScore = -inf; bestDir = [0 0 0];
    probe = 0.65*rad; lambdaOverlap = 0.30;

    for k = 1:size(D,1)
        p2 = pos + probe*D(k,:);
        for j=1:3
            if p2(j) < 0, p2(j) = -p2(j);
            elseif p2(j) > area(j), p2(j) = 2*area(j) - p2(j); end
        end
        coveredByThis = vecnorm(pts - p2, 2, 2) <= rad;
        gainNew  = sum(coveredByThis & ~coveredByOthers);
        overlap  = sum(coveredByThis &  coveredByOthers);
        score = gainNew - lambdaOverlap*overlap;
        if score > bestScore, bestScore = score; bestDir = D(k,:); end
    end

    if bestScore <= 0 || all(bestDir==0)
        f = [0 0 0];
    else
        quality = min(1, bestScore / (0.45*S));
        k_gain  = 2.8; 
        f = k_gain * quality * bestDir;
    end
end

function idxR = nearestRouterIdx(S,R)
    nS=size(S,1); idxR=zeros(nS,1);
    for i=1:nS
        d = vecnorm(R - S(i,:), 2, 2);
        [~,idxR(i)] = min(d);
    end
end

% ------------------------ GEOMETRY / PROXIMITY ---------------------------
function [pNorm, inwardDir] = edgeProximityAndDirection(p,area,radius)
    d = [p(1)-0, area(1)-p(1), p(2)-0, area(2)-p(2), p(3)-0, area(3)-p(3)];
    [dmin, idx] = min(d);
    pNorm = max(0, 1 - dmin/max(radius,eps));
    dirs = [ 1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    inwardDir = dirs(idx,:);
end

function [pNorm, awayDir] = obstacleProximityAndDirection(p,obs,radius)
    if isempty(obs), pNorm=0; awayDir=[0 0 0]; return; end
    dmin = inf; cpBest = p;
    for k=1:size(obs,1)
        cp = closestPointOnBox(p,obs(k,:));
        d  = norm(p-cp);
        if d<dmin, dmin=d; cpBest=cp; end
    end
    pNorm = max(0, 1 - dmin/max(radius,eps));
    v = p - cpBest; nv = norm(v);
    if nv>0, awayDir = v/nv; else, awayDir=[0 0 0]; end
end

function cp = closestPointOnBox(p,ob)
    x1=ob(1); y1=ob(2); z1=ob(3);
    x2=ob(4); y2=ob(5); z2=ob(6);
    cx = min(max(p(1),x1),x2);
    cy = min(max(p(2),y1),y2);
    cz = min(max(p(3),z1),z2);
    cp = [cx, cy, cz];
end

% ------------------------ COVERAGE / UTILITIES ---------------------------
function samples = sampleUncovered(positions,radii,area,obs,M)
    if nargin<5, M = 3000; end
    pts = rand(M,3).*area;

    keep = true(M,1);
    for k=1:size(obs,1)
        o = obs(k,:);
        inside = pts(:,1)>=o(1) & pts(:,1)<=o(4) & ...
                 pts(:,2)>=o(2) & pts(:,2)<=o(5) & ...
                 pts(:,3)>=o(3) & pts(:,3)<=o(6);
        keep = keep & ~inside;
    end
    pts = pts(keep,:);
    if isempty(pts), samples = []; return; end

    cov = false(size(pts,1),1);
    for i = 1:size(positions,1)
        cov = cov | (vecnorm(pts - positions(i,:), 2, 2) <= radii(i));
    end
    samples = pts(~cov,:);
end

function coverage = calculateCoverage(positions,radii,areaSize,obstacles)
    numSensors   = size(positions,1);
    totalPoints  = prod(areaSize);
    coveredPoints= 0;
    for x = 1:areaSize(1)
        for y = 1:areaSize(2)
            for z = 1:areaSize(3)
                point  = [x y z];
                covered= false;
                for i = 1:numSensors
                    if norm(point - positions(i,:)) <= radii(i)
                        covered = true; break;
                    end
                end
                for j = 1:size(obstacles,1)
                    ob = obstacles(j,:);
                    if point(1)>=ob(1)&&point(1)<=ob(4)&&...
                       point(2)>=ob(2)&&point(2)<=ob(5)&&...
                       point(3)>=ob(3)&&point(3)<=ob(6)
                        covered = false; break;
                    end
                end
                if covered, coveredPoints = coveredPoints + 1; end
            end
        end
    end
    coverage = (coveredPoints/totalPoints)*100;
end

function [Xvec, Yvec, H] = coverageHeatmap(positions,radii,areaSize,obstacles)
    Xvec = 1:areaSize(1); Yvec = 1:areaSize(2);
    H = zeros(length(Xvec), length(Yvec));
    for ix = 1:length(Xvec)
        for iy = 1:length(Yvec)
            coveredXY = false;
            for z = 1:areaSize(3)
                point  = [Xvec(ix), Yvec(iy), z];
                insideOb = false;
                for j = 1:size(obstacles,1)
                    ob = obstacles(j,:);
                    if point(1)>=ob(1)&&point(1)<=ob(4)&&...
                       point(2)>=ob(2)&&point(2)<=ob(5)&&...
                       point(3)>=ob(3)&&point(3)<=ob(6)
                        insideOb = true; break;
                    end
                end
                if insideOb, continue; end
                for i = 1:size(positions,1)
                    if norm(point - positions(i,:)) <= radii(i)
                        coveredXY = true; break;
                    end
                end
                if coveredXY, break; end
            end
            H(ix,iy) = coveredXY;
        end
    end
end

function adjusted = adjustPositions(pos,area)
    adjusted = pos;
    for i = 1:size(pos,1)
        for j = 1:3
            if adjusted(i,j) < 0
                adjusted(i,j) = -adjusted(i,j);
            elseif adjusted(i,j) > area(j)
                adjusted(i,j) = 2*area(j) - adjusted(i,j);
            end
        end
    end
end

function force = calculateForce(pos1,pos2,radius)
    d = norm(pos1-pos2);
    if d<=radius, forceMag = -1/(d+1e-6);
    else,         forceMag = 0; end
    force = forceMag*(pos2-pos1);
end

function force = calculateForceFromObstacle(pos,obstacle,radius)
    closest = closestPointOnBox(pos,obstacle);
    d       = norm(pos-closest);
    if d<=radius, forceMag = -1/(d+1e-6);
    else,         forceMag = 0; end
    force = forceMag*(closest-pos);
end

% ------------------------ CONNECTIVITY & HELPERS -------------------------
function c = connectivityMetric(sensorPos, routerPos, routerRadii)
    nS = size(sensorPos,1);
    linked = false(nS,1);
    for i = 1:nS
        d = vecnorm(routerPos - sensorPos(i,:), 2, 2);
        linked(i) = any(d <= routerRadii');
    end
    c = mean(linked);
end

function P = clamp3D(P,area)
    P(:,1) = min(max(P(:,1),0),area(1));
    P(:,2) = min(max(P(:,2),0),area(2));
    P(:,3) = min(max(P(:,3),0),area(3));
end

% ------------------------ PLOTTING ---------------------------------------
function plot3DResultsModel(tag,initPos,optPos,initRout,optRout,sRad,rRad,area,obs)
    figure('Name',[tag ' - Before'],'Color','w');
    plotScene(area,obs); hold on;
    scatter3(initPos(:,1),initPos(:,2),initPos(:,3),45,'g','filled');
    scatter3(initRout(:,1),initRout(:,2),initRout(:,3),50,[0 0 0.5],'filled');
    drawSpheres(initPos,sRad,[0 0.8 0],0.07);
    drawSpheres(initRout,rRad,[0 0 0.5],0.07);
    title([tag ' - Before']); view(35,25); grid on;

    figure('Name',[tag ' - After Optimization'],'Color','w');
    plotScene(area,obs); hold on;
    scatter3(optPos(:,1),optPos(:,2),optPos(:,3),45,'g','filled');
    scatter3(optRout(:,1),optRout(:,2),optRout(:,3),50,[0 0 0.5],'filled');
    drawSpheres(optPos,sRad,[0 0.8 0],0.10);
    drawSpheres(optRout,rRad,[0 0 0.5],0.10);
    title([tag ' - After Optimization']); view(35,25); grid on;
end

function plotScene(area,obs)
    hold on; axis equal;
    xlim([0 area(1)]); ylim([0 area(2)]); zlim([0 area(3)]);
    for i = 1:size(obs,1)
        v = generateObstacleVertices(obs(i,:));
        f = generateObstacleFaces();
        patch('Vertices',v,'Faces',f,'FaceColor',[0.8 0.2 0.2],...
              'FaceAlpha',0.35,'EdgeColor','none');
    end
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

function drawSpheres(centers,radii,color,alphaFace)
    [x,y,z]=sphere(18);
    for i=1:size(centers,1)
        surf(radii(i)*x+centers(i,1),radii(i)*y+centers(i,2),radii(i)*z+centers(i,3),...
            'FaceAlpha',alphaFace,'EdgeColor','none','FaceColor',color);
    end
end

function v = generateObstacleVertices(ob)
    x = [ob(1) ob(4)]; y = [ob(2) ob(5)]; z = [ob(3) ob(6)];
    [X,Y,Z] = meshgrid(x,y,z);
    v = [X(:) Y(:) Z(:)];
end

function f = generateObstacleFaces()
    f = [1 3 4 2; 1 2 6 5; 2 4 8 6; 3 7 8 4; 1 5 7 3; 5 6 8 7];
end

function shadedError(x, m, s, lineColor, fillColor)
    x = x(:)'; m = m(:)'; s = s(:)';
    fill([x fliplr(x)],[m-s fliplr(m+s)], fillColor, ...
        'EdgeColor','none','HandleVisibility','off');
    hold on;
    plot(x,m,'-','LineWidth',1.8,'Color',lineColor);
end

% ------------------------ HUNGARIAN --------------------------------------
function assign = hungarian(cost)
    [N,M] = size(cost);
    C = cost; C = C - min(C,[],2); C = C - min(C,[],1);
    RowCov = false(N,1); ColCov = false(M,1);
    star  = false(N,M);  prime = false(N,M);
    for i=1:N
        for j=1:M
            if C(i,j)==0 && ~RowCov(i) && ~ColCov(j)
                star(i,j)=true; RowCov(i)=true; ColCov(j)=true;
            end
        end
    end
    RowCov(:)=false; ColCov(:)=false;
    coverCols();
    while sum(ColCov) < N
        [r,c,found] = findZero();
        while ~found
            adjust();
            [r,c,found] = findZero();
        end
        prime(r,c)=true;
        sc = find(star(r,:),1);
        if isempty(sc)
            augmentPath(r,c);
            prime(:)=false; RowCov(:)=false; ColCov(:)=false; coverCols();
        else
            RowCov(r)=true; ColCov(sc)=false;
        end
    end
    assign = zeros(1,N);
    for i=1:N
        j = find(star(i,:),1);
        if isempty(j), [~,j]=min(cost(i,:)); end
        assign(i)=j;
    end
    function coverCols()
        for j=1:M, if any(star(:,j)), ColCov(j)=true; end, end
    end
    function [r,c,found] = findZero()
        found=false; r=0; c=0;
        for ii=1:N
            if ~RowCov(ii)
                jj = find(~ColCov' & C(ii,:)==0,1);
                if ~isempty(jj), r=ii; c=jj; found=true; return; end
            end
        end
    end
    function adjust()
        a = min(C(~RowCov, ~ColCov),[],'all');
        C(~RowCov, ~ColCov) = C(~RowCov, ~ColCov) - a;
        C(RowCov,  ColCov ) = C(RowCov,  ColCov ) + a;
    end
    function augmentPath(r0,c0)
        path = [r0 c0]; done=false;
        while ~done
            r = find(star(:,path(end,2)),1);
            if isempty(r), done=true;
            else
                path = [path; r path(end,2)];
                c = find(prime(r,:),1);
                path = [path; r c];
            end
        end
        for k=1:size(path,1)
            rr=path(k,1); cc=path(k,2);
            star(rr,cc)=~star(rr,cc);
        end
    end
end

% =================== Greedy seeding with separation ======================
function [seedS, seedR] = greedySeedSensorsRoutersSeparated(area, obs, nS, nR, sRad, rRad)
    vx=4; vy=4; vz=6;
    [~,~,~,vox] = buildVoxelGrid(area, obs, vx, vy, vz);   % vox: Nx3

    cx=6; cy=6;
    xs = 1:cx:area(1); ys = 1:cy:area(2);
    [CX,CY] = ndgrid(xs,ys);
    cand = [CX(:), CY(:), zeros(numel(CX),1)];
    keep = ~insideAnyObstacle(cand, obs);
    cand = cand(keep,:);
    if isempty(cand), cand = [rand(300,2).*area(1:2), zeros(300,1)]; end

    D2 = pdist2(vox, cand).^2;

    [~,ordS] = sort(sRad,'descend');
    seedS = zeros(nS,3);
    covered = false(size(vox,1),1);
    candValid = true(size(cand,1),1);
    minSepS = 0.55*mean(sRad);
    for k = 1:nS
        r2 = sRad(ordS(k))^2;
        mask = D2 <= r2;                          
        gains = sum(~covered & mask, 1);
        gains(~candValid) = -inf;
        [~,jbest] = max(gains);
        seedS(ordS(k),:) = cand(jbest,:);
        covered = covered | mask(:,jbest);
        d = vecnorm(cand - seedS(ordS(k),:), 2, 2);
        candValid = candValid & (d >= minSepS);
        candValid(jbest) = false;
    end

    seedR = zeros(nR,3);
    sensorsCovered = false(nS,1);
    DS = pdist2(seedS, cand);                     
    candValidR = true(size(cand,1),1);
    minSepR = 0.60*mean(rRad);
    for rr = 1:nR
        r2 = rRad(rr)^2;
        mask = DS.^2 <= r2;
        gains = sum(~sensorsCovered & mask, 1);
        gains(~candValidR) = -inf;
        [~,jbest] = max(gains);
        seedR(rr,:) = cand(jbest,:);
        sensorsCovered = sensorsCovered | mask(:,jbest);
        d = vecnorm(cand - seedR(rr,:), 2, 2);
        candValidR = candValidR & (d >= minSepR);
        candValidR(jbest) = false;
    end
    if any(~sensorsCovered)
        missing = seedS(~sensorsCovered,:);
        for rr = 1:nR
            if isempty(missing), break; end
            d = vecnorm(missing - seedR(rr,:),2,2);
            [~,ix]=min(d);
            seedR(rr,:) = 0.75*seedR(rr,:) + 0.25*missing(ix,:);
        end
    end
end

function [GX,GY,GZ,vox] = buildVoxelGrid(area, obs, sx, sy, sz)
    xs = 1:sx:area(1); ys = 1:sy:area(2); zs = 1:sz:area(3);
    [GX,GY,GZ] = ndgrid(xs,ys,zs);
    vox = [GX(:) GY(:) GZ(:)];
    keep = ~insideAnyObstacle(vox, obs);
    vox  = vox(keep,:);
end

function tf = insideAnyObstacle(P, obs)
    if isempty(obs)
        tf = false(size(P,1),1);
        return;
    end
    tf = false(size(P,1),1);
    for k=1:size(obs,1)
        o = obs(k,:);
        inside = P(:,1)>=o(1) & P(:,1)<=o(4) & ...
                 P(:,2)>=o(2) & P(:,2)<=o(5) & ...
                 P(:,3)>=o(3) & P(:,3)<=o(6);
        tf = tf | inside;
    end
end

% ------------------------ SAVE ALL FIGURES -------------------------------
function saveAllFigures(outDir)
    figs = findobj('Type','figure');
    for i=1:numel(figs)
        f = figs(i);
        if isempty(f.Name), base = sprintf('figure_%d', f.Number);
        else, base = regexprep(f.Name,'[^\w\-\s]',''); base = strtrim(strrep(base,' ','_')); end
        pngFile = fullfile(outDir, [base '.png']);
        figFile = fullfile(outDir, [base '.fig']);
        try
            exportgraphics(f, pngFile, 'Resolution', 200);
        catch
            print(f, pngFile, '-dpng','-r200');
        end
        try
            savefig(f, figFile);
        catch
        end
    end
    fprintf('Saved %d figure(s) to: %s\n', numel(figs), outDir);
end

function adaptF = calculateAdaptiveForce(pos,radius,areaSize,obstacles)
    numPts = 120;
    pts    = rand(numPts,3).*areaSize;
    cov    = false(numPts,1);
    for i = 1:numPts
        point  = pts(i,:);
        inside = false;
        for k = 1:size(obstacles,1)
            ob = obstacles(k,:);
            if point(1)>=ob(1)&&point(1)<=ob(4)&&...
               point(2)>=ob(2)&&point(2)<=ob(5)&&...
               point(3)>=ob(3)&&point(3)<=ob(6)
                inside = true; break;
            end
        end
        if ~inside && norm(point-pos)<=radius
            cov(i)=true;
        end
    end
    if any(cov)
        centroid = mean(pts(cov,:),1);
    else
        centroid = pos;
    end
    d = norm(pos-centroid);
    if d<=radius
        forceMag = (1 - d/radius);
    else
        forceMag = 0;
    end
    adaptF = forceMag*(centroid-pos);
end
