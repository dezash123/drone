echo off
set LOCALHOST=%COMPUTERNAME%
set KILL_CMD="C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v231\fluent/ntbin/win64/winkill.exe"

"C:\PROGRA~1\ANSYSI~1\ANSYSS~1\v231\fluent\ntbin\win64\tell.exe" DesmondWindowsBlade14 50802 CLEANUP_EXITING
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 3964) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 9900) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 8172) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 11780) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 15636) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 3468) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 17296) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 1648) 
if /i "%LOCALHOST%"=="DesmondWindowsBlade14" (%KILL_CMD% 15616)
del "D:\drone\CAD\Model\cfd\cleanup-fluent-DesmondWindowsBlade14-1648.bat"
