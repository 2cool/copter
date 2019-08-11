@ECHO OFF


:choice
set /P c=Are you sure you want to continue[Y/N]?
if /I "%c%" EQU "Y" goto :upload
if /I "%c%" EQU "N" goto :somewhere_else
goto :choice


:upload
@ECHO ON
git fetch origin && git reset --hard origin/master

exit

:somewhere_else

echo "You typed N"
pause 
exit