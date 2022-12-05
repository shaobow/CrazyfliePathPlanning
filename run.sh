mkdir -p build
mkdir -p bin
cd build
cmake ../src
cmake --build . 
cmake --install .