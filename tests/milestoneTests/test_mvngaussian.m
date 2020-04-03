% plot gammax
clear all;
x = 2; y=2;
mu = [x y];
step = 60;
% n = 20;
sigma = 0.3;
Sigma = [sigma 0;0 sigma];
x1 = linspace(0,9.2,step); x2 = linspace(0,9.2,step);
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([0  9.2   0  9.2   0   1]);
xlabel('x deviation(pixel)'); ylabel('y deviation(pixel)'); zlabel('gamma x');
title('pdf of gammax');