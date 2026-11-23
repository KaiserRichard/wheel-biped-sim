function TT = n1_decode_log(inputFile, dbcFile, mappingFile)
% Decode strict candump text to a MATLAB timetable with VNT signal scaling.
% ASC and BLF are normalized by the Python decoder to the same candump form.
db=canDatabase(dbcFile);
mapping=jsondecode(fileread(mappingFile));
lines=readlines(inputFile); lines=lines(strlength(lines)>0 & ~startsWith(lines,"#"));
n=numel(lines); time=zeros(n,1); Bus=zeros(n,1); ID=zeros(n,1);
Axis=strings(n,1); Kind=repmat("unknown",n,1); Hex=strings(n,1);
Validation=repmat("unknown_id",n,1); data=nan(n,9);
names=["Position" "Velocity" "Kp" "Kd" "Torque" "MotorID" "State" "MOS_Temperature" "Rotor_Temperature"];
for j=1:n
    tok=regexp(lines(j),'^\((\d+(?:\.\d+)?)\) can([123]) ([0-9A-Fa-f]{1,3})#([0-9A-Fa-f]{16})$','tokens','once');
    assert(~isempty(tok),'N1:Log','Malformed line or invalid DLC');
    time(j)=str2double(tok{1}); Bus(j)=str2double(tok{2}); ID(j)=hex2dec(tok{3}); Hex(j)=upper(tok{4});
    bytes=uint8(sscanf(tok{4},'%2x').');
    index=find([mapping.bus]==Bus(j) & ([mapping.study_command_id]==ID(j) | [mapping.study_feedback_id]==ID(j)),1);
    if isempty(index), continue; end
    m=mapping(index); Axis(j)=string(m.axis); Validation(j)="ok";
    if ID(j)==m.study_command_id
        if all(bytes(1:7)==255) && ismember(bytes(8),251:254)
            labels=["clear_error" "enable" "disable" "zero"];
            Kind(j)=labels(double(bytes(8))-250); continue
        end
        Kind(j)="command"; message=canMessage(db,[m.axis '_Command']);
    else
        Kind(j)="feedback"; message=canMessage(db,[m.axis '_Feedback']);
    end
    message.Data=bytes;
    decoded=message.Signals;
    for k=1:numel(names)
        if isfield(decoded,names(k)), data(j,k)=decoded.(names(k)); end
    end
    if Kind(j)=="feedback" && decoded.MotorID~=m.study_command_id
        Validation(j)="payload_id_mismatch";
    end
end
T=table(seconds(time),Bus,ID,Axis,Kind,Hex,Validation,'VariableNames',...
    {'Time','Bus','ID','Axis','Kind','Hex','Validation'});
T=[T array2table(data,'VariableNames',cellstr(names))];
TT=table2timetable(T,'RowTimes','Time');
end
