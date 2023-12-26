import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("127.0.0.1", 12345))  # Server IP and port
server_socket.listen(1)  # Listen for incoming connections

print("Server is listening...")

connection, address = server_socket.accept()
print(f"Connection from {address}")

received_data = b""  # Initialize an empty byte string to store received data

while True:
    data = connection.recv(1024)
    if not data:
        break
    print(f"Received from client: {data.decode()}")
    received_data += data  # Append the received data to the existing data

# Now you can use the received_data for other purposes
print("Received data from the client: ", received_data)

connection.close()
