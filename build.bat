@echo off
pushd source

call vcvarsall x86_amd64
cl -Zi -Od -Feauto_editor_win32 auto_editor_win32.cpp user32.lib gdi32.lib opengl32.lib ws2_32.lib shell32.lib
copy auto_editor_win32.exe "../build/auto_editor_win32.exe"
copy auto_editor_win32.pdb "../build/auto_editor_win32.pdb"
del *.obj *.pdb *.ilk *.exe

popd
pause
exit