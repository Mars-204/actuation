# Project Name: Car_Client_udp_class.py
#
# From Software Departement 
# Date: 29/08/2023
#
import socket
import time
from time import time, sleep
import datetime
import json


######################################################
# Global variables
serverAddressPort   = ("192.168.1.148", 13001)
ListenServerAddressPort   = ("192.168.1.38", 13000) #("192.168.1.52", 13000)#("127.0.0.1", 13000) #inserire ip locale
g_id_server = 'S'
bufferSize          = 1024
######################################################

class CarClient(): 
    def __init__(self, ip, port): 
        self.clientIp = None
        self.clientPort = None
        self.dataCar = None
        self.Pkt_from_Server = None

    def connSetup(self): 
        # Create a UDP socket at client side
        UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        UDPClientSocket.bind(ListenServerAddressPort) #socket locale
        UDPClientSocket.connect(serverAddressPort)
        print("connection on \n")
        return UDPClientSocket

    def rx(self,UDPClientSocket): 
        print("rx client")
        # # dictionary init
        # self.Pkt_from_Server["ID_Client"] = 'Z'
        # self.Pkt_from_Server["ID_Server"] = 'Z' 
        # self.Pkt_from_Server["Type_of_pckt"] = 0x01
        # self.Pkt_from_Server["Cmd_Code"] = 0x00
        # self.Pkt_from_Server["Cmd_Field1"] = 0xFF 
        # self.Pkt_from_Server["Cmd_Field2"] = 0xFF
        # self.Pkt_from_Server["Cmd_Field3"] = 0xFF
        # self.Pkt_from_Server["Cmd_Field4"] = 0xFF
        # self.Pkt_from_Server["Cmd_Field5"] = 0xFF
        self.Pkt_from_Server = {
            #"ID_Client": g_id_car,
            "ID_Server": g_id_server, 
            "Type_of_pckt": 0x01,
            "Cmd_Code": 0x00,
            "Cmd_Field1": 0xFF, 
            "Cmd_Field2": 0xFF,
            "Cmd_Field3": 0xFF,
            "Cmd_Field4": 0xFF,
            "Cmd_Field5": 0xFF
            }

        #Rx from server
        msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    
        rx_msg = msgFromServer[0].decode() # rx_msg = complete message from server
        print(rx_msg)

        self.Pkt_from_Server = json.loads(rx_msg) #json oggetto importato in python 
        return self.Pkt_from_Server

    def tx(self,UDPClientSocket): 
        UDPClientSocket.send(bytes(json.dumps(self.dataCar), 'UTF-8'))
        # print("message sent\n")
        

    def encode_packet(self, g_id_car, g_id_track_sector, g_id_inFront_car, g_speed, g_car_err):

        # ct stores current time
        ct = datetime.datetime.now()
        # ts store timestamp of current time
        ts = ct.timestamp()
        print("timestamp: ", ts)
  
        self.dataCar =  {
            "ID_Server": g_id_server,
            "ID_Client": g_id_car, 
            "Time_Stamp": ts,
            "ID_Track_Sector": g_id_track_sector,
            "ID_Car_in_front": g_id_inFront_car, 
            "Car_Speed": g_speed, 
            "Car_Error": g_car_err,
            "Empty_Field": 0xFF
        }


    def decode_packet(self, g_id_car):
        # dictionary 
        # field cmd declaration: <ID_Client><ID_Server><Type_of_packet><cmd_code><cmd_info1><cmd_info2><cmd_info3><cmd_info4><cmd_info5>
        if (self.Pkt_from_Server['Type_of_pckt'] == 0x06):
            print("ACK from server\n")
        else : #
            if (self.Pkt_from_Server['Type_of_pckt'] == 0x01) : #cmd in packet
                # <cmd_code>=0x01 => break
                # <cmd_code>=0x02 => stop charge station
                # <cmd_code>=0x03 => accident on track
                # <cmd_code>=0x04 => s
                # k (the number must be inserted in next field)
                # <cmd_code>=0x05 => wet track
                # .....
                car_stop = False
                car_ev_sign = False
                if self.Pkt_from_Server['ID_Client'] == g_id_car:
                    g_cmd_code = self.Pkt_from_Server['Cmd_Code'] 
                    if (g_cmd_code == 0x01) : 
                        print('break\n')
                        # stop the car
                    elif (g_cmd_code == 0x02): 
                        print('stop charge station\n')
                        car_ev_sign = True
                        # stop when detects charging station
                    elif (g_cmd_code == 0x03): 
                        print('accident on track\n')
                    elif (g_cmd_code == 0x04): 
                        print('stop car\n')
                        car_stop = True
                        # stop the car at g_id_track sector
                    elif (g_cmd_code == 0x05): 
                        print('wet track\n')
                        # reduce the speed to 30% 
        return car_stop, car_ev_sign

