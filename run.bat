rm build
mkdir build
rm bin
mkdir bin
cd build
cmake ../src 
cmake --build . --config Release 
cmake --install .