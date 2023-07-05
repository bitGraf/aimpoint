set dir="ver-0.2.0"

mkdir %dir%
mkdir %dir%\data

xcopy "..\bin\Release\aimpoint.exe" "%dir%" /y /S
xcopy "..\data" "%dir%\data" /y /S