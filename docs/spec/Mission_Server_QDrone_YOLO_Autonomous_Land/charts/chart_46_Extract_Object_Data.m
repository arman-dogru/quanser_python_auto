function [position, orientation, tracking] = fcn(data, index)

n = length(data)/7;

if (index<=0)||(index>n)
    position = zeros(1,3); orientation = zeros(1,3); tracking = 0;
else
    position = [data(index*3-2), data(index*3-1), data(index*3)];
    orientation = [data((index+n)*3 - 2), data((index+n)*3 - 1), data((index+n)*3)];
    tracking = data(index + (n*6));
end
