import grpc
import sys
import time

# Importamos los archivos generados por gRPC (asegúrate de que existan en la carpeta protos)
sys.path.insert(1, './protos')
import rpc_demo_pb2
import rpc_demo_pb2_grpc
from google.protobuf import empty_pb2

def run():
    print("Conectando al servidor gRPC en localhost:7042...")
    # 1. Creamos un canal de comunicación inseguro (sin SSL) hacia el puerto 7042
    with grpc.insecure_channel('localhost:7042') as channel:
        # 2. Creamos el "Stub" (el cliente) usando el canal
        stub = rpc_demo_pb2_grpc.RPCDemoStub(channel)
        
        # 3. Hacemos la llamada al servidor.
        # NOTA: En gRPC siempre debes enviar un mensaje de Request.
        request = empty_pb2.Empty()
        
        print("Solicitando coordenadas multiplicadas...")
        response = stub.GetMultCoords(request)
        
        print(f"¡Datos recibidos! X: {response.values[0]}, Y: {response.values[1]}, Timestamp: {response.values[2]}")

if __name__ == '__main__':
    run()