#build the project

cd `dirname $0`
mkdir -p build
cd build
cmake ..
make -j `nproc` $*
echo "the project build"
