function draw_line2(p1, p2, varargin)
% help
% draw_line2 draw directional vector points in 2D with directional arrows
%
% draw_line2(p1, p2) draw line with defulat optional parmeters;
% draw_line2(p1, p2 , param1, val1, ...)
% The line with stretching(<-->) and compressive(>---<) arrows or without
% arrows (----) can be plotted.
%
% Input arguments:
% ----------------------
% p1, p2  : Cordinates of the two points in 2D
%
% Optional parameters ( passed as parameter/value pairs):
%--------------------------------------------------------
%
% 'LineColor'       :   Colour of line ( vector of colour or char)
%                       default : % 'b'
% 'LineStyle        :   Style of line 
%                       default: '-'
% 'LineWidth'       :   Width of the line (Scaler values)
%                       default :  5)
% 'ArrowDirection'  :   (0,1,2) (0 : p1--- p2, 1: p1 --> p2 2: and P1 <-> p2)
%                       default : 1
% 'ArrowLength'     :   Length of the arrow ( scaler values)
%                       default : 3
% 'ArrowIntend'     :   Intended arrow ( scaler value)
%                       default : 1
% 'ArrowAngle'      :   Angle of arrow with the main line in degree
%                       default : 30
% 'ArrowColor'      :   Arrow face clour ( in color vector or char)
%                       default : 'r'
% 'ArrowEdgeColor   :   arrow edge colour ( in colour vector or char or
%                       default : 'k'
% Example:
%
%   draw_line2([0 0]', [100 100]',...
%                     'Linecolor', 'b',...
%                     'LineWidth', 5 ,...
%                     'ArrowDirection, 2,...
%                     'ArrowLength, 10,....
%                     'ArrowIntend, 3,...
%                     'ArrowAngle', 45,...
%                     'ArrowColor, 'r')
%                
%  will draw the line with the specified optional parameters in the current plot               
%
% -------------------------------------------------------------------------
% Copyright 2012 Sabesan Sivapalan  
% Last edited: 25/09/2012           
% email: sabeshuom@gmail.com
%
% LineStyle added by SIDNEY RDC.
% Last edited: 23/01/2015
% email: sydney_rdc@hotmail.com
% -------------------------------------------------------------------------
%% check the given points are column format
clc;
% p1 = [ 45 56]';
% p2 = [ 3 67]';

hold_status = ishold;
if ~exist('p1', 'var')  || ~exist('p2', 'var')
    disp('No input points.')
    return
end
if length(p1) ~=2  || length(p1) ~=2
    disp('Not proper 2D cordinates given for the input points.')
    return
end

if ~hold_status
    subplot(1,1,1,'replace');
    hold on;
end
if size(p1 , 1) == 1
    p1 = p1';
end

if size(p2 , 1) == 1
    p2 = p2';
end

properties = {'LineColor',...
              'LineStyle',...
              'LineWidth',...
              'ArrowDirection',...
              'ArrowLength',...
              'ArrowIntend',...
              'ArrowAngle',...
              'ArrowColor',...
              'ArrowEdgeColor'};

% default values
values.LineColor        = 'b';
values.LineStyle        = '-';
values.LineWidth        = 5;
values.ArrowDirection   = 1;
values.ArrowLength      = 3;
values.ArrowIntend      = 0.1;
values.ArrowAngle       = 15;
values.ArrowColor       = 'r';
values.ArrowEdgeColor   = 'k';

given_property = varargin(1:2:end);
propertyValue = varargin(2:2:end);

for i=1:size(given_property, 2)
validInput = sum(cell2mat(strfind(properties, given_property{i})));

if validInput
    values.(sprintf('%s',given_property{i})) = propertyValue{i};
end

end

%plot bidirectional arrow
if values.ArrowDirection == 2
    % compute points for arrows
    [pnt1_1 pnt2_1 pnt3_1] = getArrowPoints(p1, p2, values);
    [pnt1_2 pnt2_2 pnt3_2] = getArrowPoints(p2, p1, values);
    % plot the line
    plot([pnt3_1(1) pnt3_2(1)], [pnt3_1(2) pnt3_2(2)], 'color', values.LineColor, 'LineWidth', values.LineWidth, 'LineStyle', values.LineStyle);
    % patch arrows
    patch([pnt3_1(1) pnt1_1(1) p2(1) pnt2_1(1)], [pnt3_1(2) pnt1_1(2) p2(2) pnt2_1(2)], values.ArrowColor , 'EdgeColor', values.ArrowEdgeColor );
    patch([pnt3_2(1) pnt1_2(1) p1(1) pnt2_2(1)], [pnt3_2(2) pnt1_2(2) p1(2) pnt2_2(2)], values.ArrowColor , 'EdgeColor', values.ArrowEdgeColor );
else
    [pnt1 pnt2 pnt3] = getArrowPoints(p1, p2, values);
    %plot the line
    plot([p1(1) pnt3(1)], [p1(2) pnt3(2)], 'color', values.LineColor, 'LineWidth', values.LineWidth, 'LineStyle', values.LineStyle);

    if (values.ArrowDirection == 1)
    % patch arrow
        patch([pnt3(1) pnt1(1) p2(1) pnt2(1)], [pnt3(2) pnt1(2) p2(2) pnt2(2)], values.ArrowColor , 'EdgeColor', values.ArrowEdgeColor );
    end
end    
if ~hold_status
    hold off;
end
axis equal;

end
function [pnt1 pnt2 pnt3] = getArrowPoints(p1, p2, values)

rz = atan2((p2(2) - p1(2)), (p2(1) - p1(1))) ;
r = [ cos(rz) -sin(rz) ;   sin(rz)  cos(rz) ];

if values.ArrowAngle > 90
    values.ArrowLength= -values.ArrowLength;
    values.ArrowIntend = -values.ArrowIntend;
end
pnt1 = p2 + r * [-values.ArrowLength values.ArrowLength  * tan(values.ArrowAngle * pi / 180)]';
pnt2 = p2 + r * [-values.ArrowLength -values.ArrowLength * tan(values.ArrowAngle * pi / 180)]';
pnt3 = p2 + r * [-(values.ArrowLength - values.ArrowIntend)  0]';
end