function [faces,vertices]=read_stl(path)
%READ_STL Minimal binary/ASCII STL reader, no toolbox requirement.
fid=fopen(path,'rb');assert(fid~=-1,'kinova:mesh','Cannot open %s',path);
close=onCleanup(@()fclose(fid));fseek(fid,0,'eof');bytes=ftell(fid);fseek(fid,80,'bof');
count=fread(fid,1,'uint32=>double',0,'ieee-le');
if ~isempty(count)&&bytes==84+50*count
    raw=fread(fid,[50,count],'uint8=>uint8');
    coordinates=typecast(reshape(raw(13:48,:),[],1),'single');
    [~,~,endian]=computer;if endian=='B',coordinates=swapbytes(coordinates);end
    vertices=double(reshape(coordinates,3,[])');faces=reshape(1:3*count,3,[])';
else
    text=fileread(path);matches=regexp(text,'vertex\s+([\d.eE+\-]+)\s+([\d.eE+\-]+)\s+([\d.eE+\-]+)','tokens');
    assert(mod(numel(matches),3)==0&&~isempty(matches),'kinova:mesh','Unsupported STL');
    vertices=zeros(numel(matches),3);
    for i=1:numel(matches),vertices(i,:)=str2double(matches{i});end
    faces=reshape(1:numel(matches),3,[])';
end
end
