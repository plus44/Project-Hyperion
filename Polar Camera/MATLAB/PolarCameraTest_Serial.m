clear all;
portName = '/dev/tty.SLAB_USBtoUART';
delete(instrfind);
s = serial(portName);
set(s,'BaudRate',921600,'Parity','none','DataBits',8, 'StopBits', 1);
fopen(s);

num_steps = 200;

for i=1:num_steps,
   fprintf(s, '%c', 's'); % Ask for a line (128 bytes) of data
   raw_pix_data(:,i) = uint8(fread(s, 128, 'uint8')); % Receive the data
end

fclose(s);

save('RAW/raw_pix_6.mat','raw_pix_data');