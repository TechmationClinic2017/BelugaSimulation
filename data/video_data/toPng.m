function [ ] = toPng( filename, foldername )
%toPng by Vaibhav Viswanathan (vviswanathan@hmc.edu)
%   This function converts an mov file to pngs
%   Inputs: .mov filename, foldername

v = VideoReader(filename); %initialize video reader
f = 1; %video sample frequency
T = round(v.FrameRate*f);
i = 0; %initialize counter
mkdir(foldername); %initialize folder
path = pwd;


while hasFrame(v)
    if mod(i,T) ==0
        video = readFrame(v);
        name = strcat(path,'/',foldername,'/',string(i),'.png');
        name = char(name);
        imwrite(video, name)
    end
    
    i = i+1;
end

end
