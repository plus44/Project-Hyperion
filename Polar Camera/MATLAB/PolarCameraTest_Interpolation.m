clear all;
% Our data is stored in the RAW folder as .mat files
filename = 'raw_pix_6';
load(strcat('RAW/',filename,'.mat'));

% % % % % % % % % % % % % % INTERPOLATION % % % % % % % % % % % % % % % % 
w_A = 128;
h_A = 128;

% Number of samples in a circle
num_sam = 200;
d_theta = 2*pi/num_sam;

% Create the radial camera sampling pattern with mfg defects: x,y-offsets
num_sensors = 64;
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

% Arrange the vectors in a single column
% NOTE: This sampling pattern progresses from the center outward, for every
% angle theta
for i=1:length(x_sam(:,1)),
   x_sam_cv((i-1)*length(x_sam(1,:))+1:i*length(x_sam(1,:)),1) = ...
       x_sam(i,:);
   y_sam_cv((i-1)*length(y_sam(1,:))+1:i*length(y_sam(1,:)),1) = ...
       y_sam(i,:);
end

% Arrange the image data in a single column with the same coordinates as
% our sampling pattern
for i=1:length(raw_pix_data(1,:)),
   raw_pix_cv((i-1)*num_sensors+1:i*num_sensors,1) = ...
       double(flipud(raw_pix_data(1:num_sensors, i)));  
end

% Interpolate the sampled data points
F = scatteredInterpolant(x_sam_cv, y_sam_cv, raw_pix_cv, 'natural');

% Desired width and height
des_w = 4*num_sensors;
des_h = 4*num_sensors;

x = repmat(0:w_A/des_w:w_A*(1-1/des_w), 1, des_h);
for i=1:des_h,
    y((i-1)*des_w+1:i*des_w) = i*h_A/des_h*ones([1 des_w]);
end

% Row->Column
x = x(:);
y = y(:);

% Reconstructed image
A_rec_cv = F(x,y);

% Arrange it s.t. we can display it as a matrix
for i=1:des_h,
   temp = A_rec_cv((i-1)*des_w+1:des_w*i);
   A_rec(i,:) = temp(:)./255;
end

figure('Name', 'Recovered Image');
imshow(A_rec);
imwrite(A_rec, strcat('OUTPUT/',filename,'.jpg'));
