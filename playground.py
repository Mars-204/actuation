import ipdb

pkt = {'ID_Client': 'S',
       'ID_Server': 'C',
       'Type_of_packet': 1,
       'cmd_code':00,
       'cmd_info1': 'break',
       'cmd_info2': 'stop after charging',
       'cmd_info3':'wet road',
       'cmd_info4':'accident on track',
       'cmd_info5':'STOP'}
ipdb.set_trace()
if 'STOP' in pkt:
    print('yes')