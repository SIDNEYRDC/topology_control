%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw Robot Function
%
% Maintainer: Sidney Carvalho - sydney.rdc@gmail.com
% Last Change: 2016 Jun 09 23:10:32
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function draw_robot(x, theta, length, varargin)
% help
% draw_robot() draw a triangle robot shape centered in a point x
%
% draw_robot(x, theta, length) draw the robot with default parameters
% draw_robot(x, theta, length, RobotLabel, LineColor...) draw the robot with
% extra parameters
%
% Input arguments:
% ----------------------
% x: Coordinates of the robot's center point (meters)
% theta: Robot orientation (radians)
% length: Robot length (meters)
%
% Optional parameters (passed as parameter/value pairs):
% ------------------------------------------------------
%
% 'RobotLabel'       :   Name to be printed into the robot's shape
%                       default: ''
% 'FillColor'       :   Colour of the shape (vector of colour or char)
%                       default: 'w'
%
% Example:
% --------
% draw_robot([0 0], 0, ...
%           'RobotLabel', 'robot', ...
%           'FillColor', 'r')
%

% parameter check
if ~exist('x', 'var')
    disp('Not input point.')
    return
%elseif length(x) ~= 2
    %disp('The origin point must be a 2D coordinate.')
    %return
elseif ~exist('theta', 'var')
    disp('Not robot angle inserted.')
    return
elseif ~exist('length', 'var')
    disp('Not robot length inserted.')
    return
end

% extra parameter name
properties = {'RobotLabel', ...
              'FillColor'};

% default values
values.RobotLabel       = '';
values.FillColor        = 'w';

% get extra parameters name
given_property = varargin(1 : 2 : end);

% get extra parameters values
propertyValue = varargin(2 : 2 : end);

% relates each value with its parameter name
for i = 1 : size(given_property, 2)
    validInput = sum(cell2mat(strfind(properties, given_property{i})));

    if validInput
        values.(sprintf('%s',given_property{i})) = propertyValue{i};
    end
end

% rotation matrix
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];

% axis x length
b = 2/7*length;

% robot shape points
x1 = rot*[0; 2/3*length];
x2 = rot*[b; - length/3];
x3 = rot*[- b; - length/3];

% keep the figure
hold on;

% plot robot shape
%plot([x1(1) x2(1) x3(1) x1(1)], [x1(2) x2(2) x3(2) x1(2)], 'Color', 'k');
fill(x(1) + [x1(1) x2(1) x3(1)], x(2) + [x1(2) x2(2) x3(2)], values.FillColor);

% plot robot label
text(x(1), x(2), values.RobotLabel, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

end
