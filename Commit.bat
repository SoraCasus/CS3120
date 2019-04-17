@echo off
git add *
set /p commitMessage="Commit Message: "
git commit -m commitMessage
git push origin master
pause
