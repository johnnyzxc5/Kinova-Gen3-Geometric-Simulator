function write_json(path,value)
%WRITE_JSON Persist a readable result, preserving errors on failed open.
parent=fileparts(path);if ~isfolder(parent),mkdir(parent);end
fid=fopen(path,'w');assert(fid~=-1,'kinova:io','Cannot open %s',path);
close=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
