classdef DroneRCSModel < SwerlingIITarget
    % DroneRCSModel - UAV-specific RCS model with optional extended scatterer support
    %
    % Extends SwerlingIITarget to model:
    % - UAV body resonance effects
    % - Multi-scatterer spatial distribution (optional)
    % - Elevation angle dependency (optional)
    %
    % This model is suitable for small UAVs (e.g., DJI Mavic class) in the
    % resonance region where significant RCS fluctuations occur.
    %
    % Usage:
    %   % Simple drone target (Swerling II only)
    %   drone = DroneRCSModel(-17, 512, 5.8e9);
    %   [alpha, rcs] = drone.generateFluctuatingRCS();
    %
    %   % Extended drone with scatterers
    %   drone = DroneRCSModel(-17, 512, 5.8e9, 'shuffle', 'NumScatterers', 5);
    %   drone.setupExtendedScatterers();
    %   drone.plotExtendedScattererPattern();
    %
    % Reference:
    %   UAV RCS characteristics in resonance region (5-30 GHz)
    %
    % Author: Radar Fusion Research Project
    % Date: February 2026
    
    properties
        DroneType           % Drone type identifier (e.g., 'generic', 'quadrotor')
        NumScatterers       % Number of scattering centers
        ScattererPositions  % [N×3] positions relative to drone center (m)
        ScattererRCS        % [N×1] individual scatterer RCS values (m²)
        ElevationAngles     % Array of elevation angles for pattern (degrees)
        UseExtendedModel    % Flag for multi-scatterer model
    end
    
    properties (SetAccess = private)
        RCSvsElevation      % [num_angles×1] RCS pattern vs elevation
        ExtendedAlpha       % Extended scatterer complex amplitudes
    end
    
    methods
        function obj = DroneRCSModel(meanRCS_dBsm, numChirps, fc, seedMode, varargin)
            % Constructor for DroneRCSModel
            %
            % Inputs:
            %   meanRCS_dBsm - Mean RCS in dBsm (e.g., -17 for 5.8 GHz UAV)
            %   numChirps    - Number of chirps/pulses (e.g., 512)
            %   fc           - Operating frequency in Hz (e.g., 5.8e9)
            %   seedMode     - 'shuffle' or integer seed for reproducibility
            %
            % Optional Name-Value pairs:
            %   'DroneType'      - 'generic', 'quadrotor', 'hexarotor' (default: 'generic')
            %   'NumScatterers'  - Number of scattering centers (default: 1)
            %   'ElevationRange' - [min max] elevation angles in degrees (default: [0 90])
            
            % Handle default arguments
            if nargin < 4
                seedMode = 'shuffle';
            end
            if nargin < 3
                fc = 5.8e9;
            end
            if nargin < 2
                numChirps = 512;
            end
            if nargin < 1
                meanRCS_dBsm = -17;
            end
            
            % Call parent constructor
            obj@SwerlingIITarget(meanRCS_dBsm, numChirps, fc, seedMode);
            
            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'DroneType', 'generic', @ischar);
            addParameter(p, 'NumScatterers', 1, @isnumeric);
            addParameter(p, 'ElevationRange', [0, 90], @isnumeric);
            parse(p, varargin{:});
            
            obj.DroneType = p.Results.DroneType;
            obj.NumScatterers = p.Results.NumScatterers;
            obj.ElevationAngles = linspace(p.Results.ElevationRange(1), ...
                                           p.Results.ElevationRange(2), 91);
            obj.UseExtendedModel = (obj.NumScatterers > 1);
            
            % Initialize scatterer positions based on drone type
            if obj.UseExtendedModel
                obj.setupExtendedScatterers();
            end
        end
        
        function setupExtendedScatterers(obj)
            % Setup extended scatterer model with spatial distribution
            %
            % Models the UAV as multiple point scatterers distributed
            % across the airframe (body, arms, motors, etc.)
            
            N = obj.NumScatterers;
            c = 3e8;
            lambda = c / obj.OperatingFreq;
            
            switch lower(obj.DroneType)
                case 'quadrotor'
                    % Quadrotor: body center + 4 motor positions
                    arm_length = 0.15;  % 15 cm arm length (Mavic-class)
                    angles = [0, 90, 180, 270] * pi/180;
                    
                    obj.ScattererPositions = zeros(min(N, 5), 3);
                    obj.ScattererPositions(1, :) = [0, 0, 0];  % Body center
                    
                    for k = 2:min(N, 5)
                        a = angles(k-1);
                        obj.ScattererPositions(k, :) = [arm_length*cos(a), arm_length*sin(a), 0];
                    end
                    
                    % Distribute RCS (body gets 50%, motors split 50%)
                    obj.ScattererRCS = zeros(min(N, 5), 1);
                    obj.ScattererRCS(1) = obj.MeanRCS_m2 * 0.5;
                    motor_rcs = obj.MeanRCS_m2 * 0.5 / (min(N, 5) - 1);
                    obj.ScattererRCS(2:end) = motor_rcs;
                    
                case 'hexarotor'
                    % Hexarotor: body center + 6 motor positions
                    arm_length = 0.25;  % 25 cm arm length
                    angles = (0:5) * 60 * pi/180;
                    
                    obj.ScattererPositions = zeros(min(N, 7), 3);
                    obj.ScattererPositions(1, :) = [0, 0, 0];
                    
                    for k = 2:min(N, 7)
                        a = angles(k-1);
                        obj.ScattererPositions(k, :) = [arm_length*cos(a), arm_length*sin(a), 0];
                    end
                    
                    obj.ScattererRCS = zeros(min(N, 7), 1);
                    obj.ScattererRCS(1) = obj.MeanRCS_m2 * 0.4;
                    motor_rcs = obj.MeanRCS_m2 * 0.6 / (min(N, 7) - 1);
                    obj.ScattererRCS(2:end) = motor_rcs;
                    
                otherwise  % 'generic'
                    % Generic: random scatterer distribution within UAV envelope
                    envelope_size = 0.3;  % 30 cm envelope
                    
                    % Random positions within envelope
                    obj.ScattererPositions = (rand(N, 3) - 0.5) * envelope_size;
                    obj.ScattererPositions(1, :) = [0, 0, 0];  % First scatterer at center
                    
                    % Random RCS distribution (normalized to mean)
                    raw_rcs = rand(N, 1) + 0.5;  % 0.5 to 1.5 relative
                    obj.ScattererRCS = raw_rcs / sum(raw_rcs) * N * obj.MeanRCS_m2;
            end
            
            fprintf('Extended scatterer model: %d scatterers (%s)\n', ...
                size(obj.ScattererPositions, 1), obj.DroneType);
        end
        
        function [alpha, rcs_m2] = generateFluctuatingRCS(obj)
            % Generate fluctuating RCS using extended scatterer model if enabled
            %
            % For extended model: coherently sum contributions from all scatterers
            % Each scatterer fluctuates independently (Swerling II)
            
            if ~obj.UseExtendedModel
                % Use parent class method for simple single-scatterer model
                [alpha, rcs_m2] = generateFluctuatingRCS@SwerlingIITarget(obj);
                return;
            end
            
            M = obj.NumChirps;
            N = size(obj.ScattererPositions, 1);
            
            % Generate independent Swerling II fluctuations for each scatterer
            alpha_all = zeros(M, N);
            for k = 1:N
                mean_sigma_k = obj.ScattererRCS(k);
                std_k = sqrt(mean_sigma_k / 2);
                X = std_k * randn(M, 1);
                Y = std_k * randn(M, 1);
                alpha_all(:, k) = X + 1j * Y;
            end
            
            % Coherent sum of all scatterers (assuming far-field, same range)
            % In a more advanced model, we could add phase shifts based on geometry
            alpha = sum(alpha_all, 2);
            rcs_m2 = abs(alpha).^2;
            
            % Store results
            obj.ComplexAmplitudes = alpha;
            obj.RCS_m2 = rcs_m2;
            obj.RCS_dB = 10*log10(rcs_m2 / obj.MeanRCS_m2);
            obj.LocalSNR_dB = obj.RCS_dB;
            obj.IsGenerated = true;
            obj.ExtendedAlpha = alpha_all;
        end
        
        function rcs = getRCSAtElevation(obj, elevation_deg)
            % Get RCS at a specific elevation angle
            %
            % Simple model: RCS varies with elevation due to projected area
            % Maximum RCS at broadside (0°), reduced at high elevation
            %
            % Input:
            %   elevation_deg - Elevation angle in degrees (0° = horizontal)
            %
            % Output:
            %   rcs - RCS value in m² at this elevation
            
            % Empirical model based on projected area
            % RCS(el) = RCS_mean * cos²(el) + RCS_mean * 0.2 * sin²(el)
            % This gives ~80% reduction at 90° elevation
            el_rad = elevation_deg * pi / 180;
            rcs = obj.MeanRCS_m2 * (cos(el_rad)^2 + 0.2 * sin(el_rad)^2);
        end
        
        function computeElevationPattern(obj)
            % Compute RCS pattern over elevation angles
            
            obj.RCSvsElevation = zeros(length(obj.ElevationAngles), 1);
            for k = 1:length(obj.ElevationAngles)
                obj.RCSvsElevation(k) = obj.getRCSAtElevation(obj.ElevationAngles(k));
            end
        end
        
        function fig = plotRCSvsElevation(obj)
            % Plot RCS pattern vs elevation angle
            
            if isempty(obj.RCSvsElevation)
                obj.computeElevationPattern();
            end
            
            fig = figure('Name', 'RCS vs Elevation', 'Color', 'w');
            
            % Linear plot
            subplot(2,1,1);
            plot(obj.ElevationAngles, obj.RCSvsElevation * 1e3, 'b-', 'LineWidth', 1.5);
            grid on;
            xlabel('Elevation Angle (degrees)');
            ylabel('RCS (mm²)');
            title(sprintf('RCS vs Elevation (%.1f GHz, %s drone)', ...
                obj.OperatingFreq/1e9, obj.DroneType));
            
            % dB plot
            subplot(2,1,2);
            rcs_dBsm = 10*log10(obj.RCSvsElevation);
            plot(obj.ElevationAngles, rcs_dBsm, 'b-', 'LineWidth', 1.5);
            hold on;
            yline(obj.MeanRCS_dBsm, 'r--', 'LineWidth', 1.5, 'Label', 'Mean RCS');
            hold off;
            grid on;
            xlabel('Elevation Angle (degrees)');
            ylabel('RCS (dBsm)');
            title('RCS vs Elevation (dB scale)');
        end
        
        function fig = plotExtendedScattererPattern(obj)
            % Plot the spatial distribution of scatterers
            
            if ~obj.UseExtendedModel
                warning('Extended scatterer model not enabled');
                fig = [];
                return;
            end
            
            fig = figure('Name', 'Extended Scatterer Pattern', 'Color', 'w');
            
            pos = obj.ScattererPositions;
            rcs = obj.ScattererRCS;
            
            % Scale marker size by RCS
            marker_sizes = 50 + 500 * rcs / max(rcs);
            
            % 3D scatter plot
            subplot(1,2,1);
            scatter3(pos(:,1)*100, pos(:,2)*100, pos(:,3)*100, marker_sizes, ...
                10*log10(rcs), 'filled');
            colorbar;
            xlabel('X (cm)');
            ylabel('Y (cm)');
            zlabel('Z (cm)');
            title(sprintf('Scatterer Positions (%s)', obj.DroneType));
            grid on;
            axis equal;
            view(45, 30);
            
            % Top-down view
            subplot(1,2,2);
            scatter(pos(:,1)*100, pos(:,2)*100, marker_sizes, 10*log10(rcs), 'filled');
            colorbar;
            xlabel('X (cm)');
            ylabel('Y (cm)');
            title('Top View (color = RCS in dBsm)');
            grid on;
            axis equal;
            
            % Draw drone outline for quadrotor
            if strcmpi(obj.DroneType, 'quadrotor') && size(pos, 1) >= 5
                hold on;
                arm_pos = pos(2:5, 1:2) * 100;
                for k = 1:4
                    plot([0, arm_pos(k,1)], [0, arm_pos(k,2)], 'k-', 'LineWidth', 2);
                end
                hold off;
            end
            
            sgtitle(sprintf('Extended Drone Model: %d Scatterers (Mean RCS = %.1f dBsm)', ...
                size(pos, 1), obj.MeanRCS_dBsm));
        end
        
        function printSummary(obj)
            % Print summary including drone-specific information
            
            % Call parent summary
            printSummary@SwerlingIITarget(obj);
            
            % Add drone-specific info
            fprintf('--- Drone Model Info ---\n');
            fprintf('Drone Type: %s\n', obj.DroneType);
            fprintf('Extended Model: %s\n', string(obj.UseExtendedModel));
            
            if obj.UseExtendedModel
                fprintf('Number of Scatterers: %d\n', size(obj.ScattererPositions, 1));
                fprintf('Scatterer RCS range: %.2f to %.2f dBsm\n', ...
                    10*log10(min(obj.ScattererRCS)), 10*log10(max(obj.ScattererRCS)));
            end
            fprintf('=======================================\n\n');
        end
    end
    
    methods (Static)
        function demo()
            % Static demo method showing DroneRCSModel capabilities
            fprintf('=== Drone RCS Model Demo ===\n\n');
            
            % Simple drone (Swerling II only)
            fprintf('--- Simple Drone Model (5.8 GHz) ---\n');
            drone1 = DroneRCSModel(-17, 512, 5.8e9, 42);
            drone1.generateFluctuatingRCS();
            drone1.printSummary();
            drone1.plotRCSPattern();
            drone1.plotHistogram();
            
            % Extended quadrotor model
            fprintf('\n--- Extended Quadrotor Model (24 GHz) ---\n');
            drone2 = DroneRCSModel(-9.5, 512, 24e9, 42, ...
                'DroneType', 'quadrotor', 'NumScatterers', 5);
            drone2.generateFluctuatingRCS();
            drone2.printSummary();
            drone2.plotExtendedScattererPattern();
            drone2.plotRCSvsElevation();
            
            % Validate distributions
            fprintf('\n--- Distribution Validation ---\n');
            drone1.validateDistribution();
            drone2.validateDistribution();
        end
    end
end
