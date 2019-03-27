function [catted] = horzcat_fields(structs)
% Take a struct array and return one structure with all fields horizcontall
% concatenated.

catted = blank_struct(structs, 1, 1);
fields = fieldnames(catted);

assert(isvector(structs), 'Can only handle vector struct arrays');

for i = 1:length(fields)
	field = fields{i};
	if length(structs) > 1 && size(structs(1).(field), 2) > 1
		% If it's not a vector, then concat with cells
		% Or make it three dimensional???
		catted.(field) = {structs.(field)};
	else
		catted.(field) = horzcat(structs.(field));
	end
end

end

function [blank, fields] = blank_struct(elem, rows, cols)
%STRUCT_BLANK Simple function to copy struture (fields, no data) of given structure
%For parsing into struct arrays. Will make a struct array of (ros, cols),
%or same size as elem if no args specified.
if nargin < 2
	rows = size(elem, 1);
end
if nargin < 3
	cols = size(elem, 2);
end
fields = fieldnames(elem);
setup = cell(length(fields) * 2, 1);
setup(1:2:end) = fields;
blank = repmat(struct(setup{:}), rows, cols);
end
