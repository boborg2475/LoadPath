@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d C:\Users\bobby\AppData\Local\Temp\claude\C--Users-bobby-Documents-Unreal-Projects-DestructionGame\1c20c3b7-b4e9-40fb-a43d-64c43f1388e4\scratchpad\stress313
cl /nologo /c /std:c++20 /O2 /MD /EHs-c- /GR- /fp:precise /D_HAS_EXCEPTIONS=0 /DNOMINMAX /DNDEBUG /DEIGEN_MPL2_ONLY /utf-8 /permissive- /I C:\Users\bobby\Documents\LoadPath\include /I C:\Users\bobby\Documents\LoadPath\src /I C:\Users\bobby\Documents\LoadPath\third_party\eigen /I . RbsmDiag.cpp
cl /nologo /std:c++20 /O2 /MD /EHs-c- /GR- /D_HAS_EXCEPTIONS=0 /DNOMINMAX /DNDEBUG /utf-8 /permissive- /I C:\Users\bobby\Documents\LoadPath\include /I . rbsm_probe.cpp RbsmDiag.obj C:\Users\bobby\Documents\LoadPath\build\msvc\Release\loadpath.lib /Fe:rbsm_probe.exe
