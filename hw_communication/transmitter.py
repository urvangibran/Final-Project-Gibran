import socket
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("192.168.000.111", 6969))  # ip address dan port

degree = "45"
mode_tembak = "1"
kecepatan = "100"
password = "its"

task1 = password + degree
task2 = password + mode_tembak
task3 = password + kecepatan

while True:
    server.sendto(task1.encode(), (("192.168.000.111"),6969))
    server.sendto(task2.encode(), (("192.168.000.111"),6969))
    server.sendto(task3.encode(), (("192.168.000.111"),6969))
    print("data terkirim")

    message, address = server.recvfrom(8)
    balasan = message.decode()
    if balasan[:3] == password:
        pesan = balasan [3:]
        print(f'reply from micro = {pesan}')