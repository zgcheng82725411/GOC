import numpy as np
import time
import copy
import csv

def read_data(file_num, cust_num, charge_num):
    consumer_constraint = {0: [1, 0, 0, 480, 1440]}  # {cust_id: [cust_type, weight, volume, first_receive, last_receive], []}
    time_mat = np.zeros([cust_num + charge_num, cust_num + charge_num])
    dist_mat = np.zeros([cust_num + charge_num, cust_num + charge_num])
    file_code = 1
    with open(r'data\inputnode_%s.csv' % file_num, 'r') as fc:
        reader = csv.reader(fc)
        next(reader)
        next(reader)
        for v in reader:
            # 充电站
            if v[1] == '4':
                consumer_constraint[int(v[0]) - file_code * 10000] = [4, 0, 0, 480, 1440]
            else:
                first_t = v[6].split(':')
                last_t = v[7].split(':')
                consumer_constraint[int(v[0]) - file_code*10000] = \
                    [int(v[1]), float(v[4]), float(v[5]),
                     int(first_t[0])*60 + int(first_t[1]),
                     int(last_t[0])*60 + int(last_t[1])]

    with open(r'data\inputdistancetime_%s.txt' % file_num, 'r') as fd:
        next(fd)
        for row in fd:
            to_list = row.strip('\n').split(',')
            if int(to_list[1]) > 0:
                from_id = int(to_list[1]) - file_code*10000
            else:
                from_id = int(to_list[1])
            if int(to_list[2]) > 0:
                to_id = int(to_list[2]) - file_code*10000
            else:
                to_id = int(to_list[2])
            dist_mat[from_id, to_id] = int(to_list[3])
            time_mat[from_id, to_id] = int(to_list[4])
    return consumer_constraint, time_mat, dist_mat