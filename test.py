#!/usr/bin/env python
import socket
def main():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    localaddr = ("192.168.1.5",2522)
    udp_socket.bind(localaddr)
    print("inited")
    
    while True:
    	print("looping")
    	# udp_socket.sendto(b'aaa',("192.168.1.15",2523))
        recv_data = udp_socket.recvfrom(1024)
        
        recv_msg = recv_data[0]
        send_addr = recv_data[1]

        print(recv_data)
        print("message:%s from:%s" %(str(send_addr),recv_msg.decode("gbk")))
    udp_socket.close()
if __name__ == "__main__":
    main()