function [outputV] = test(inputV)

figure
outputV = 2*inputV;
plot(outputV,inputV);
axis equal;

end
