basepath=$(cd `dirname $0`; pwd)
cd $basepath/protocol && mkdir -p ../src_protocol && protoc-c --c_out ../src_protocol *.proto
cd $basepath/protocol && mkdir -p ../src_protocol && protoc --python_out ../src_protocol *.proto
