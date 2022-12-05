if not exist "build" mkdir build
if not exist "bin" mkdir bin
cd build
cmake ../src 
cmake --build . 
cmake --install .