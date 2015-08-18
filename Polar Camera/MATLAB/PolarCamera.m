clear;
close all;
filename = 'lena512.bmp';
A = imread(filename);
% A = rgb2gray(X);
A_d = im2double(A); 
figure('Name', 'Original Image');
imshow(A);

w_A = length(A(1,:));
h_A = length(A(:,1));

% Create x,y,v arrays as row vectors
x = repmat(1:w_A, 1, h_A);
for i=1:h_A,
    y((i-1)*w_A+1:i*w_A) = i*ones([1 w_A]);
end
for i=1:h_A,
    A_rv((i-1)*w_A+1:i*w_A) = A_d(i,1:w_A);
end

% Change row vectors to column vectors
x = x(:);
y = y(:);
A_cv = A_rv(:);

% Interpolate to create what the sensor would see
F = scatteredInterpolant(x, y, A_cv, 'natural');

% Number of samples in a circle
num_sam = 200;
d_theta = 2*pi/num_sam;

% Create the radial camera sampling pattern with mfg defects: x,y-offsets
num_sensors = 128;
% pix_offset = min([h_A/2 w_A/2])/4;
y_pix_offset = 0;
x_pix_offset = 0;
for i=1:num_sensors,
   [x_t, y_t] = ...
       circle(w_A/2, h_A/2, ...
       sqrt((i*min([h_A/2 w_A/2])/num_sensors)^2+x_pix_offset^2)+y_pix_offset,...
       d_theta); 
   x_sam(:,i) = x_t;
   y_sam(:,i) = y_t;
end

% Arrange the vectors in a single row
for i=1:length(x_sam(:,1)),
   x_sam_rv((i-1)*length(x_sam(1,:))+1:i*length(x_sam(1,:))) = ...
       x_sam(i,:);
   y_sam_rv((i-1)*length(y_sam(1,:))+1:i*length(y_sam(1,:))) = ...
       y_sam(i,:);
end

% Row->Column
x_sam_cv = x_sam_rv(:);
y_sam_cv = y_sam_rv(:);

% Sample the image at the actual radial sampling points
A_sam = F(x_sam_cv, y_sam_cv);

% Set up the theoretically sampled points (assuming no mfg defects)
for i=1:num_sensors,
   [x_t, y_t] = ...
       circle(w_A/2, h_A/2, i*min([h_A/2 w_A/2])/num_sensors, d_theta); 
   x_sam_th(:,i) = x_t;
   y_sam_th(:,i) = y_t;
end

% Arrange the vectors in a single row
for i=1:length(x_sam_th(:,1)),
   x_sam_th_rv((i-1)*length(x_sam_th(1,:))+1:i*length(x_sam_th(1,:))) = ...
       x_sam_th(i,:);
   y_sam_th_rv((i-1)*length(y_sam_th(1,:))+1:i*length(y_sam_th(1,:))) = ...
       y_sam_th(i,:);
end

% Row->Column
x_sam_th_cv = x_sam_th_rv(:);
y_sam_th_cv = y_sam_th_rv(:);

% Interpolate the sampled points as the theoretical points
F2 = scatteredInterpolant(x_sam_th_cv, y_sam_th_cv, A_sam, 'linear');

% Desired width and height
des_w = 4*num_sensors;
des_h = 4*num_sensors;

x = repmat(0:w_A/des_w:w_A*(1-1/des_w), 1, des_h);
clear y;
for i=1:des_h,
    y((i-1)*des_w+1:i*des_w) = i*h_A/des_h*ones([1 des_w]);
end

% Row->Column
x = x(:);
y = y(:);

% Recovered image
A_rec_cv = F2(x, y);

for i=1:des_h,
   temp = A_rec_cv((i-1)*des_w+1:des_w*i);
   A_rec(i,:) = temp(:);
end

figure('Name', 'Recovered Image');
imshow(A_rec);

% A_f = fft2(A);
% A_f = fftshift(A_f);
% A_f_min = min(min(abs(A_f)));
% A_f_max = max(max(abs(A_f)));
% figure('Name', 'FFT Magnitude Original');
% imshow(abs(A_f), [A_f_min A_f_max]);
% 
% A_rec_f = fft2(A_rec);
% A_rec_f = fftshift(A_rec_f);
% A_rec_f_min = min(min(abs(A_rec_f)));
% A_rec_f_max = max(max(abs(A_rec_f)));
% figure('Name', 'FFT Magnitude Recovered');
% imshow(abs(A_rec_f), [A_rec_f_min A_rec_f_max]);
 





