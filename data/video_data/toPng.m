function [ ] = toPng( filename, foldername )
%toPng by Vaibhav Viswanathan (vviswanathan@hmc.edu)
%   This function converts an mov file to pngs
%   Inputs: .mov filename, foldername

v = VideoReader(filename); %initialize video reader
f = 0.25; %video sample frequency
T = round(v.FrameRate*f);
i = 0; %initialize counter
mkdir(foldername); %initialize folder
path = pwd;


while hasFrame(v)
    video = readFrame(v);
    name = strcat(path,'/',foldername,'/',string(i),'.png');
    name = char(name);
    if mod(i, T) ==0
        imwrite(video, name)
    end
    i = i+1;
end

end

