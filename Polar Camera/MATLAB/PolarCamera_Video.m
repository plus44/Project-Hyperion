% Procedure:
% 1. Load video
% 2. Read a frame
% 3. Construct an interpolating function
% 4. Sample a line of pixels in the frame at the correct angle
% 5. Construct an interpolating function from one revolution of pixel line
%    reading
% 6. Sample at the integer valued x-y coordinates of the original video
% 7. Reconstruct a frame
% 8. Generate a video from the reconstructed frames

clear;
% % % % % % % % % % % % % % % % STEP 1 % % % % % % % % % % % % % % 
% Load input video file
in_video = VideoReader('front_flip_2500fps.mp4');
w_A = in_video.Width;
h_A = in_video.Height;
num_sam = 75;
d_theta = 2*pi/num_sam;
theta = 0;
num_pix = 128;
k = 1;

% Create an array of the x sampling points passed to the interpolating f-n
x = repmat((1:w_A)', h_A, 1);
% Create an array of the y sampling points passed to the interpolating f-n
for i=1:h_A,
    y((i-1)*w_A+1:i*w_A,1) = i*ones([w_A 1]);
end

while hasFrame(in_video),
    % % % % % % % % % % % % % % % % STEP 2 % % % % % % % % % % % % % % 
    % Read a single frame from this video and convert to grayscale and double
    A = im2double(rgb2gray(readFrame(in_video)));
   
    % Arrange the frame values as a column vector with each entry corresponding 
    % to the x and y coordinates we defined above
    for i=1:h_A,
        A_cv((i-1)*w_A+1:i*w_A,1) = A(i,1:w_A)';
    end

    % % % % % % % % % % % % % % % % STEP 3 % % % % % % % % % % % % % % 
    % Construct an interpolating function
    F = scatteredInterpolant(x, y, A_cv, 'natural');
   
    % % % % % % % % % % % % % % % % STEP 4 % % % % % % % % % % % % % % 
    % Find the sampling points for the current angle
    [x_sam, y_sam] = sampleLine(w_A/2, h_A/2, min(w_A/2, h_A/2), theta, num_pix);
    % Col 1 stores the x sampling pts, Col 2 the y sampling pts, Col 3 the
    % interpolated values
    frame_sampled(:,1,k) = x_sam; 
    frame_sampled(:,2,k) = y_sam;
    frame_sampled(:,3,k) = F(x_sam, y_sam);
    
    % Increment theta and k for the next frame
    theta = theta + d_theta;
    k = k + 1;
    disp(['K: ' num2str(k)]);
    pause(0.01);
end

% % % % % % % % % % % % % % % % STEP 5 % % % % % % % % % % % % % % 
% Loop over every revolution of samples and construct an interpolating
% function based on that revolution 
for i=1:floor((k-1)/num_sam),
    % Arrange all the sampling points/values in the revolution as a column
    for j=1:num_sam,
        x_int((j-1)*num_pix+1:j*num_pix,1) = ...
            frame_sampled(:,1,j+(i-1)*num_sam);
        y_int((j-1)*num_pix+1:j*num_pix,1) = ...
            frame_sampled(:,2,j+(i-1)*num_sam);
        F_int((j-1)*num_pix+1:j*num_pix,1) = ...
            frame_sampled(:,3,j+(i-1)*num_sam);
    end
    
    % Interpolate the sampled values over this revolution
    F_sam = scatteredInterpolant(x_int, y_int, F_int, 'linear');
    
    % % % % % % % % % % % % % % % % STEP 6 % % % % % % % % % % % % % % 
    % Sample the new interpolant at discrete x, y values
    for p=1:h_A,
        y_disc((p-1)*h_A+1:p*h_A,1) = p*ones([h_A 1]);
    end
    x_disc = repmat(((w_A-h_A)/2:(w_A+h_A)/2 - 1)', h_A, 1);
    rec_frame_cv = F_sam(x_disc, y_disc);
    
    % % % % % % % % % % % % % % % % STEP 7 % % % % % % % % % % % % % %
    % Reconstruct a frame 
    for l=1:h_A,
       temp = rec_frame_cv((l-1)*h_A+1:l*h_A);
       rec_vid(l,:,i) = temp(:); 
    end
end


% % % % % % % % % % % % % % % % STEP 8 % % % % % % % % % % % % % %
% Generate a video from reconstructed frames
out_video = VideoWriter('out_video.avi');
out_video.FrameRate = in_video.FrameRate;
open(out_video);
for i=1:length(rec_vid(1,1,:)),
   rec_vid(:,:,i)
   writeVideo(out_video, im2frame(rec_vid(:,:,i))); 
end
close(out_video);
