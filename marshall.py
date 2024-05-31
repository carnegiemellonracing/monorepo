#!/usr/bin/python
path = "can_fmt.rawh"
with open('can_fmt.json', 'r') as file_in, open ('Gen/can_fmt.rawh', 'w+') as file_out:
    data = file_in.read().replace('\n', '').replace('\"','\\\"').replace(' ','')
    data = '"' + data + '"'
    file_out.write(data)