% Run this in MATLAB AFTER the Stereo Camera Calibrator has exported
% `stereoParams` to the workspace (Export Camera Parameters -> Export to Workspace).
%
% It flattens the stereoParameters OBJECT into a plain numeric struct and saves
% it as -v7. Both details matter: a saved MATLAB object is opaque outside MATLAB,
% and the default -v7.3 format is HDF5, which needs different tooling to read.
% A plain struct in -v7 loads anywhere.
%
% Copy the resulting stereoParams_final.mat back to the Mac, into
%   ~/Programming/Raisen/stereo_calibration/matlab_out/

if ~exist('stereoParams', 'var')
    error(['No `stereoParams` in the workspace. In the Stereo Camera ' ...
           'Calibrator: Export Camera Parameters -> Export to Workspace.']);
end

c1 = stereoParams.CameraParameters1;
c2 = stereoParams.CameraParameters2;

S = struct();

% Intrinsics. R2022b renamed IntrinsicMatrix (transposed, 1-indexed) to K
% (conventional row-major). Handle both rather than guessing the version.
if isprop(c1, 'K')
    S.K1 = c1.K;
    S.K2 = c2.K;
    S.k_convention = 'K (R2022b+, already row-major)';
else
    S.K1 = c1.IntrinsicMatrix';
    S.K2 = c2.IntrinsicMatrix';
    S.k_convention = 'IntrinsicMatrix transposed to row-major';
end

S.radial1     = c1.RadialDistortion;
S.radial2     = c2.RadialDistortion;
S.tangential1 = c1.TangentialDistortion;
S.tangential2 = c2.TangentialDistortion;
S.imageSize   = c1.ImageSize;          % [rows cols] = [600 960]

% Extrinsics, left -> right. R2022b+ exposes PoseCamera2 (a rigidtform3d);
% older releases expose RotationOfCamera2 / TranslationOfCamera2.
if isprop(stereoParams, 'PoseCamera2')
    S.R = stereoParams.PoseCamera2.R;
    S.T = stereoParams.PoseCamera2.Translation;
    S.pose_convention = 'PoseCamera2 (R2022b+)';
else
    S.R = stereoParams.RotationOfCamera2;
    S.T = stereoParams.TranslationOfCamera2;
    S.pose_convention = 'RotationOfCamera2 / TranslationOfCamera2';
end

S.meanReprojError = stereoParams.MeanReprojectionError;
S.numPairs        = size(c1.ReprojectionErrors, 3);
S.squareSizeMM    = 25.6;   % MEASURED with a ruler, not the 25 mm nominal

% per-pair error, so the Mac side can see which pairs were kept and how they sit
e = c1.ReprojectionErrors;
S.perPairError = squeeze(sqrt(mean(sum(e.^2, 2), 1)))';

save('stereoParams_final.mat', 'S', '-v7');

fprintf('\n=== saved stereoParams_final.mat to %s ===\n', pwd);
fprintf('mean reprojection error : %.4f px over %d pairs\n', ...
        S.meanReprojError, S.numPairs);
fprintf('baseline norm(T)        : %.2f mm   (old calibration was 148.23)\n', norm(S.T));
fprintf('image size              : %d x %d (rows x cols)\n', S.imageSize(1), S.imageSize(2));
fprintf('K1 =\n'); disp(S.K1);
fprintf('radial1 = '); disp(S.radial1);
fprintf('T = '); disp(S.T);
fprintf('\nCopy stereoParams_final.mat back to the Mac.\n');
