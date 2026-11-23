function run_all(firmwareRoot)
% Reproduce offline N1 tests through MATLAB MCP. Optional read-only rebuild.
root=fileparts(fileparts(mfilename('fullpath')));old=pwd;
cleanup=onCleanup(@()cd(old)); %#ok<NASGU>
cd(root);addpath(fullfile(root,'matlab'));
Simulink.sdi.setAutoArchiveMode(false);Simulink.sdi.clear;
assert(~startsWith(lower(string(tempdir)),"c:"),'N1:Temp','Set TMP/TEMP to approved E: scratch first');
pyexe=fullfile(root,'.venv','Scripts','python.exe');
assert(isfile(pyexe),'N1:Python','Create the study-local cantools venv first; see RUN_ME.md');
if nargin>0
    [s,o]=system(sprintf('"%s" python/build.py "%s"',pyexe,firmwareRoot));disp(o);assert(s==0);
end
cc=mex.getCompilerConfigurations('C','Installed');
assert(~isempty(cc),'N1:Compiler','No installed C compiler detected');
[s,o]=system(sprintf('"%s" -std=c11 -O0 -ffp-contract=off tests/dm_oracle.c -o tests/dm_oracle.exe',cc(1).LinkerName));
assert(s==0,'N1:Compile','%s',o);
writelines(["PASS: existing MinGW C compiler; -std=c11 -O0 -ffp-contract=off";...
    "Six source-derived functions; host CAN send stub; explicit input clamp wrapper"],fullfile(root,'out','c_tests.log'));
[s,~]=system('tests\dm_oracle.exe < tests\oracle_input.txt > out\c_oracle.csv');assert(s==0);
[s,~]=system(sprintf('"%s" python/run_tests.py > out/python_tests.log 2>&1',pyexe));
if s~=0,disp(fileread(fullfile(root,'out','python_tests.log')));end
assert(s==0,'N1:Python','Python tests failed; inspect out/python_tests.log');
disp(fileread(fullfile(root,'out','python_tests.log')));
% Retain prior MATLAB attempts while making the main test log a fresh run.
log=fullfile(root,'out','matlab_tests.log');
if isfile(log)
    copyfile(log,fullfile(root,'out','matlab_previous_attempts.log'));
    writelines("",log);
end
run_n1;
[s,o]=system(sprintf('"%s" -m pip --disable-pip-version-check freeze',pyexe));assert(s==0);
writelines(string(o),fullfile(root,'python','requirements-lock.txt'));
end
