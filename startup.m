%% startup.m
% Project bootstrap script for the Inverted Pendulum LQI+RBF project.
%
% Run this FIRST, every session, before anything else. It:
%   1. Resolves the project root (the folder this file lives in)
%   2. Adds every subfolder to the MATLAB path
%   3. Clears the workspace / command window / figures
%   4. Loads parameters() into the base workspace as `params` so it is
%      immediately available for quick interactive work
%
% Usage:
%   >> startup
%
% Author: Project datasheet — Inverted Pendulum (LQI + RBF + Estimator)

clc;
close all;
clearvars -except params; %#ok<NASGU> (keep clean, params reloaded below)

% ---- 1. Resolve project root -------------------------------------------
projectRoot = fileparts(mfilename('fullpath'));

% ---- 2. Add all subfolders to path -------------------------------------
% genpath grabs every nested folder (Models, Controllers, Observers, RBF,
% Simulations, Utilities, Visualizations, Results, etc.) in one shot.
addpath(genpath(projectRoot));

% Avoid polluting the path with version-control / hidden folders if any
% ever appear (e.g. .git). This is a no-op if none exist.
pathCell = strsplit(path, pathsep);
badIdx = contains(pathCell, [filesep '.']);
if any(badIdx)
    rmpath(strjoin(pathCell(badIdx), pathsep));
end

% ---- 3. Confirm and load default parameters ----------------------------
fprintf('Project root added to path:\n  %s\n', projectRoot);

params = parameters(); %#ok<NASGU> default config is set inside parameters.m
fprintf('Loaded default configuration: %s\n', params.config);
fprintf('Type "edit parameters" to change configuration (A or B) or any value.\n');

clear projectRoot pathCell badIdx
