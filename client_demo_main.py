from Car_Client_udp_class import CarClient
from time import time, sleep


def reciever_server():
    myClient = CarClient("192.168.1.38", 13000) #local ip+port

    car_UDPClientSocket = myClient.connSetup()

    while(True):
        g_id_car = 'C'
        recieve_msg = myClient.rx(car_UDPClientSocket)
        car_stop, car_ev_sign = recieve_msg.decode(g_id_car)
        sleep(2)

    # while(True):


    #     # custom params
    #     g_id_car = 'A'
    #     g_id_track_sector = 1
    #     ga_Command = 0x00
    #     g_id_inFront_car = 'Z'
    #     g_speed = 0
    #     g_car_err = 0

    #     myClient.encode_packet(g_id_car, g_id_track_sector, g_id_inFront_car, g_speed, g_car_err)
    #     myClient.tx(car_UDPClientSocket)
