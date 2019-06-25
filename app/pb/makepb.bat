protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./Define.proto
protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./IdentityMsg.proto
protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./router_execsta.proto
protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./router_setting.proto
protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./router_env.proto
protoc --proto_path=. --plugin=protoc-gen-nanopb=.\generator\protoc-gen-nanopb.bat --nanopb_out=. ./router_gps.proto