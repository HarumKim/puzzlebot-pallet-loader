
//sudo chown robotics:robotics /usr/local/go
//go install google.golang.org/protobuf/cmd/protoc-gen-go@v1.28
//go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@v1.2
//go install github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-grpc-gateway
//go install github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-openapiv2
//go install google.golang.org/protobuf/cmd/protoc-gen-go 
//go install google.golang.org/grpc/cmd/protoc-gen-go-grpc

//sudo apt install protobuf-compiler

//protoc -I ./protos -I /home/robotics/tools/googleapis --go_out ./protos --go_opt paths=source_relative --go-grpc_out ./protos --go-grpc_opt paths=source_relative rpc-demo-gw.proto
//protoc -I ./protos -I /home/robotics/tools/googleapis --plugin=protoc-gen-grpc-gateway=/home/robotics/go/bin/protoc-gen-grpc-gateway --grpc-gateway_out ./protos --grpc-gateway_opt logtostderr=true --grpc-gateway_opt paths=source_relative rpc-demo-gw.proto

//go mod init example.com/rest-gateway-demo
//go mod tidy
//go build go-gateway.go

//https://github.com/grpc-ecosystem/grpc-gateway/releases/tag/v2.15.2