function [obj] = yaml_read_file(filePath, toMatrix)

if nargin < 2
	toMatrix = [];
end

assert(exist(filePath, 'file') ~= 0, 'MATLAB:MATYAML:FileNotFound', 'No such file to read: ',filePath);
[dirPath, ~, ~] = fileparts(filePath);
raw = fileread(filePath);
obj = yaml_load(raw, toMatrix, dirPath);

end