set datetimef=%date:~-4%_%date:~3,2%_%date:~0,2%__%time:~0,2%_%time:~3,2%_%time:~6,2%
git add . && git commit -m "%datetimef%" 

:choice
set /P c=Want to push 2 git[Y/N]?
if /I "%c%" EQU "Y" goto :push
if /I "%c%" EQU "N" goto :somewhere_else
goto :choice

:push
@ECHO ON
git push origin master

exit

:somewhere_else

echo "You typed N"
pause 
exit