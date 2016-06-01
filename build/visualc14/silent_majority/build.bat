call "%VS140COMNTOOLS%\..\..\VC\vcvarsall.bat" amd64

for %%P in (.) do set NAME=%%~nP%%~xP
MSBuild /t:clean;build /p:configuration=release;WholeProgramOptimization=PGInstrument %NAME%.vcxproj
"x64\Release\%NAME%.exe" bench
MSBuild /p:configuration=release;WholeProgramOptimization=PGOptimize;arch=AVX %NAME%.vcxproj
pgomgr /summary "x64\Release\%NAME%.pgd" > profile.txt

rem profile.txt
copy /Y x64\Release\silent_majority.exe SILENT_MAJORITY.exe
pause

rem AVX2: nodes 8458135 time 9507
rem SSE42: nodes 8549070 time 9501
rem SSE2: nodes 8128418 time 9523
