function [out] = horzcat_fields_domains(s)

out = struct();
fields = fieldnames(s);
count = length(fields);
for i = 1:count
    field = fields{i};
    if ismatrix(s(1).(field))
        out.(field) = horzcat_domains(s.(field));
    end
end

end
