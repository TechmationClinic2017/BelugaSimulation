files = dir();
filenames = {files.name};
pat = '(forward.*)\.bag';

matches = regexp(filenames,pat,'tokens');

for match = matches
    token = match{1};
    if ~isempty(token)
        matlab_cells_are_dumb = token{1};
        prefix = matlab_cells_are_dumb{1};
        [t,y] = plot_ekf_data([prefix '_ekf.dat'], [prefix '_input.dat']);
    end
end
        
