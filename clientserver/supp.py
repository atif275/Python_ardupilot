import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = '0.0.0.0'  # Listen on all available network interfaces
port = 12345  # Use the same port as the C++ client
server_socket.bind((host, port))
server_socket.listen(1)

print(f"Server is listening on {host}:{port}")

client_socket, client_address = server_socket.accept()
print(f"Accepted connection from {client_address}")

data = client_socket.recv(1024)
print(f"Received data: {data.decode('utf-8')}")

client_socket.close()
server_socket.close()
