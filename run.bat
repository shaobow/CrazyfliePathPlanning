if exist "build" rm build
mkdir build
if exist "bin" rm bin
mkdir bin
cd build
cmake ../src 
cmake --build . --config Release 
cmake --install .