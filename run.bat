if not exist "build" mkdir build
if not exist "bin" mkdir bin
cd build
cmake ../src 
cmake --build . --config Release 
cmake --install .