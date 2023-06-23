import socket
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("192.168.0.2", 6969))  # ip address dan port

degree = "45"
mode_tembak = "1"
kecepatan = "100"

task1 = degree
task2 = mode_tembak
task3 = kecepatan

while True:
    server.sendto(task1.encode(), (("192.168.000.111"),6969))
    server.sendto(task2.encode(), (("192.168.000.111"),6969))
    server.sendto(task3.encode(), (("192.168.000.111"),6969))
    message, address = server.recvfrom(8)
    balasan = message.decode()
    print(f'reply from micro = {balasan}')
