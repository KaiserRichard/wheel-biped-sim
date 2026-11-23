function run_n1()
% Offline VNT experiments; no physical CAN channels and no plant simulation.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab'));
% Caller sets TMP/TEMP to the programme-approved E: scratch directory.
Simulink.sdi.setAutoArchiveMode(false); Simulink.sdi.clear;
diary(fullfile(root,'out','matlab_tests.log')); cleanup=onCleanup(@()diary('off')); %#ok<NASGU>
timer=tic;
assert(~startsWith(lower(string(tempdir)),"c:"),'N1:Temp','Temporary directory must be off C:');
fprintf('MATLAB %s; Vehicle Network Toolbox %s; tempdir on E: = %d\n',version('-release'),ver('vnt').Version,startsWith(lower(string(tempdir)),"e:"));
files=[dir(fullfile(root,'*.dbc'));dir(fullfile(root,'dbcs','*.dbc'))];
for j=1:numel(files)
    database=canDatabase(fullfile(files(j).folder,files(j).name));
    fprintf('Database %s: %d messages validated\n',files(j).name,numel(database.Messages));
end
db=canDatabase(fullfile(root,'wbr_dm.dbc'));
fw=canDatabase(fullfile(root,'dbcs','codec_firmware_wheel.dbc'));
vectors=readtable(fullfile(root,'tests','vectors.csv'),'TextType','string','VariableNamingRule','preserve');
oracle=readtable(fullfile(root,'out','c_oracle.csv'),'TextType','string','VariableNamingRule','preserve');
assert(height(vectors)==height(oracle));
fields=["Position" "Velocity" "Kp" "Kd" "Torque"];
names=["L_front" "L_rear" "L_wheel" "R_front" "R_rear" "R_wheel"];
count=0; maxDecode=0; result=strings(height(vectors),1);
for j=1:height(vectors)
    p=vectors.profile(j);
    if p=="hip", axes=[1 2 4 5]; use=db;
    elseif p=="wheel", axes=[3 6]; use=db;
    else, axes=3; use=fw; end
    for axis=axes
        if p=="firmware_wheel", name="Firmware_Wheel_Command"; else, name=names(axis)+"_Command"; end
        message=n1_encode(use,name,vectors{j,fields},p);
        hex=string(reshape(dec2hex(message.Data,2).',1,[]));
        assert(hex==vectors.hex(j) && hex==oracle.hex(j),'N1:ByteParity','Command payload mismatch');
        decoded=message.Signals;
        info=signalInfo(use,char(name));
        for k=1:5
            si=info(strcmp({info.Name},fields(k)));
            code=round((decoded.(fields(k))-si.Offset)/si.Factor);
            assert(code==vectors{j,"q"+string(k-1)},'N1:RawParity','Raw code mismatch');
        end
        err=max(abs([decoded.Position decoded.Velocity decoded.Torque]-oracle{j,{'position','velocity','torque'}}));
        maxDecode=max(maxDecode,err);assert(err<2e-5);
        count=count+1;
    end
    result(j)="PASS";
end
writetable(table(vectors.case,vectors.profile,result,'VariableNames',{'Case','Profile','Result'}),fullfile(root,'out','matlab_vector_results.csv'));
fb=readtable(fullfile(root,'tests','feedback_vectors.csv'),'TextType','string');
for j=1:height(fb)
    axis=fb.id(j)-16;message=canMessage(db,char(names(axis)+"_Feedback"));
    message.Data=uint8(sscanf(fb.hex(j),'%2x').'); d=message.Signals;
    qFields=["Position" "Velocity" "Torque" "MotorID" "State" "MOS_Temperature" "Rotor_Temperature"];
    info=signalInfo(db,char(names(axis)+"_Feedback"));
    for field=qFields
        si=info(strcmp({info.Name},field));q=round((d.(field)-si.Offset)/si.Factor);
        assert(q==fb{j,field});
    end
end
sp=canDatabase(fullfile(root,'dbcs','wbr_dm_special.dbc'));
for axis=1:6
    for opcode=251:254
        message=canMessage(sp,char(names(axis)+"_Special"));
        for j=0:6, message.Signals.("Prefix"+j)=255; end
        message.Signals.Opcode=opcode; assert(isequal(message.Data,uint8([255*ones(1,7) opcode])));
    end
end
TT=n1_decode_log(fullfile(root,'out','synthetic.log'),fullfile(root,'wbr_dm.dbc'),fullfile(root,'reference','motor_map.json'));
assert(height(TT)==12024 && all(TT.Validation=="ok"));
py=readtable(fullfile(root,'out','decoded_python.csv'),'TextType','string');
assert(all(abs(seconds(TT.Time)-py.time_s)<1e-9));assert(all(TT.Hex==py.hex));
assert(all(TT.Kind==py.kind) && all(TT.Axis==py.axis) && all(TT.Bus==py.bus));
maxLog=0;
for field=[fields "MotorID" "State" "MOS_Temperature" "Rotor_Temperature"]
    a=TT.(field);b=py.(field);assert(isequal(isnan(a),isnan(b)));
    err=max(abs(a-b),[],'omitnan'); if ~isempty(err), maxLog=max(maxLog,err); end
end
assert(maxLog<1e-10);
writetimetable(TT,fullfile(root,'out','decoded_timetable.csv'));
save(fullfile(root,'out','decoded_timetable.mat'),'TT');
neg=n1_decode_log(fullfile(root,'tests','negative.log'),fullfile(root,'wbr_dm.dbc'),fullfile(root,'reference','motor_map.json'));
assert(isequal(neg.Validation,["unknown_id";"payload_id_mismatch"]));
try
    n1_decode_log(fullfile(root,'tests','bad_dlc.log'),fullfile(root,'wbr_dm.dbc'),fullfile(root,'reference','motor_map.json'));
    error('N1:Test','Invalid DLC accepted');
catch ex
    assert(strcmp(ex.identifier,'N1:Log'));
end
% VNT independently imports generated standard Vector ASC and BLF logs.
asc=canMessageImport(fullfile(root,'out','synthetic.asc'),'Vector','OutputFormat','timetable');
blf=blfread(fullfile(root,'out','synthetic.blf'));
assert(totalRows(asc)==height(TT) && totalRows(blf)==height(TT));
fprintf('VNT ASC and BLF import: %d frames each\n',height(TT));
plots(root,TT,names);
summary=struct('Result','PASS','SourceVectors',height(vectors),'VNTCommandChecks',count,...
    'FeedbackVectors',height(fb),'LifecycleChecks',24,'DecodedFrames',height(TT),...
    'MaxCDecodeError',maxDecode,'MaxPythonLogDifference',maxLog,'WallSeconds',toc(timer),...
    'TempDrive','E:','PlantSimulatedSeconds',0);
writestruct(summary,fullfile(root,'out','matlab_summary.json'));
fprintf('PASS: %d command payloads, %d feedback, 24 lifecycle; max log difference %.3g\n',count,height(fb),maxLog);
fprintf('MATLAB wall time %.3f s; plant simulated time 0 s\n',summary.WallSeconds);
Simulink.sdi.clear;
end

function n=totalRows(x)
if iscell(x), n=sum(cellfun(@height,x)); else, n=height(x); end
end

function plots(root,TT,names)
set(groot,'defaultAxesFontSize',11);
f=figure('Visible','off','Theme','light','Color','white','Position',[80 80 1200 850]);
tl=tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for a=1:6
    nexttile; c=TT.Axis==names(a) & TT.Kind=="command"; b=TT.Axis==names(a) & TT.Kind=="feedback";
    plot(seconds(TT.Time(c))-.01,TT.Torque(c),'LineWidth',1.3); hold on
    plot(seconds(TT.Time(b))-.01,TT.Torque(b),'--','LineWidth',1.1);
    title(strrep(names(a),'_',' '));xlabel('Replay time (s)');ylabel('Torque (N m)');grid on
    if a==1,legend('Command','Synthetic feedback','Location','best');end
end
title(tl,'S11 aggregate trace mapped to a synthetic six-axis CAN fixture; feedback delay = 2 ms');
exportgraphics(f,fullfile(root,'out','01_joint_torque.png'),'Resolution',140,'BackgroundColor','white');close(f);
f=figure('Visible','off','Theme','light','Color','white','Position',[80 80 1100 650]);
tl=tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
periodRows=[];nexttile;hold on
for a=1:6
    for kind=["command" "feedback"]
        sel=TT.Axis==names(a) & TT.Kind==kind; t=seconds(TT.Time(sel));jitter=(diff(t)-.002)*1e6;
        if kind=="command", plot(t(2:end)-.01,jitter,'DisplayName',strrep(names(a),'_',' '));end
        periodRows=[periodRows; table(names(a),kind,min(jitter),max(jitter),sqrt(mean(jitter.^2)),...
            'VariableNames',{'Axis','Kind','MinJitter_us','MaxJitter_us','RMSJitter_us'})]; %#ok<AGROW>
    end
end
xlabel('Replay time (s)');ylabel('Period minus 2 ms (us)');title('Command start timestamp jitter from synthetic release/arbitration');grid on
legend('Location','eastoutside');nexttile;
bar(categorical(names,names),reshape(periodRows.RMSJitter_us,2,6).');xlabel('Joint (axis)');ylabel('RMS period jitter (us)');
title('Command and feedback start period jitter');legend('Command','Feedback');grid on
exportgraphics(f,fullfile(root,'out','02_frame_jitter.png'),'Resolution',140,'BackgroundColor','white');close(f);
writetable(periodRows,fullfile(root,'out','frame_jitter.csv'));
f=figure('Visible','off','Theme','light','Color','white','Position',[80 80 1200 520]);
tl=tiledlayout(1,2,'TileSpacing','compact');load=readtable(fullfile(root,'out','bus_load.csv'));
nexttile;bar(load.buses,[load.unstuffed_pct load.worst_pct]);xlabel('Independent buses (count)');ylabel('Load per bus (%)');
title('6 motors, 500 Hz request/reply, 1 Mbit/s');legend('No stuffing','Worst stuffing');ylim([0 100]);grid on
replay=readtable(fullfile(root,'out','replay_bus_load.csv'));nexttile;
bar(replay.bus,[replay.actual_stuffed_pct replay.worst_pct]);xlabel('S12 study bus (index)');ylabel('Load per bus (%)');
title('Two-second payload replay, excluding lifecycle setup');legend('Actual CRC + stuffing','Worst stuffing');ylim([0 40]);grid on
exportgraphics(f,fullfile(root,'out','03_bus_load.png'),'Resolution',140,'BackgroundColor','white');close(f);
end
