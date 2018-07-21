sample_rate=4096;
frequency=60;
num_samples = 512;
x = 1:num_samples;
t = 0:1/sample_rate:1;
N=length(t); %/number of samples

s=sin(2*pi*frequency*t);

s_trim = s(1:num_samples);
plot(s_trim);

% y = (fft(s_trim));
% plot(x(1:num_samples/2),y(1:num_samples/2));
% csvwrite('sine-wave',s_trim);

%% 2 freq

sample_rate=4096;
frequency=60;
num_samples = 512;
x = 1:num_samples;
t = 0:1/sample_rate:1;
N=length(t); %/number of samples

s= 120*sin(2*pi*frequency*t) + 20*sin(2*pi*frequency*3*t);

s_trim = s(1:num_samples);
plot(s_trim)