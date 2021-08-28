import csv
import numpy as np
import time
import copy
import random
import matplotlib.pyplot as plt

import readfile
from greedsolve import GreedSolve
from simulatedannealing import Simulatedannealing

def get_cost(solution, all_vehicle_type, if_write, run_t=289.3):
    """Given the solution saved in list, calculate the total cost of the solution.
    Write the solution to local in the required format."""

    file_code = 1
    result = [['trans_code', 'vehicle_type', 'dist_seq', 'distribute_lea_tm', 'distribute_arr_tm', 'distance', 'trans_cost', 'charge_cost', 'wait_cost', 'fixed_use_cost', 'total_cost', 'charge_cnt']]
    total_cost = 0
    vehicle_code = 0
    for k, vehicle in enumerate(solution):
        if all_vehicle_type[k] == 1:
            trans0 = small_car_capacity[4]
            fix0 = small_car_capacity[5]
        else:
            trans0 = big_car_capacity[4]
            fix0 = big_car_capacity[5]

        # get the output format
        route = [0] * len(result[0])
        vehicle_code += 1
        route[0] = 'DP' + str(vehicle_code ).zfill(4)  # vehicle name
        route[1] = all_vehicle_type[k]  # vehicle type
        route_ele = []
        for ele in vehicle:
            if ele == 0:
                route_ele.append(str(ele))
            else:
                route_ele.append(str(ele + file_code * 10000))
        route[2] = ';'.join(route_ele)  # route

        total_cost += fix0
        total_cost += distance_matrix[0, vehicle[1]] * trans0
        if time_matrix[0, vehicle[1]] + earliest_start_time <= consumer_constraint[vehicle[1]][3]:
            t = consumer_constraint[vehicle[1]][3] + operation_time
            time_out = int(consumer_constraint[vehicle[1]][3] - time_matrix[0, vehicle[1]])
            route[3] = str(time_out / 60) + ':' + str(time_out % 60).zfill(2)  # vehicle out time
        else:

            t = time_matrix[0, vehicle[1]] + earliest_start_time + operation_time
            route[3] = str(earliest_start_time / 60) + ':' + str(earliest_start_time % 60).zfill(2)  # vehicle out time
        total_wait_cost = 0
        for i in range(2, len(vehicle)-1):  # can not wait at the first 2 points
            total_cost += (distance_matrix[vehicle[i - 1], vehicle[i]] * trans0)
            if consumer_constraint[vehicle[i]][0] == 4:
                total_cost += charge_one_cost
            wait_t = consumer_constraint[vehicle[i]][3] - (t + time_matrix[vehicle[i - 1], vehicle[i]])
            if wait_t > 0:
                # print veh[i-1], veh[i], wait_t
                total_cost += (wait_t / 60. * wait_one_hour_cost)
                total_wait_cost += (wait_t / 60. * wait_one_hour_cost)
                t = consumer_constraint[vehicle[i]][3] + operation_time
            else:
                if vehicle[i] == 0:
                    t += (time_matrix[vehicle[i - 1], vehicle[i]] + every_round_stay_cost)
                else:
                    t += (time_matrix[vehicle[i - 1], vehicle[i]] + operation_time)
            if vehicle[i] == 0:  # get back to the depot and will depart again, wait cost is 1hour
                total_cost += wait_one_hour_cost
                total_wait_cost += wait_one_hour_cost
            # print veh[i], str(int(t) / 60) + ':' + str(int(t) % 60).zfill(2)


        in_time = int(t + time_matrix[vehicle[-2], 0])
        route[4] = str(in_time / 60) + ':' + str(in_time % 60).zfill(2)  # vehicle back time
        total_dist = 0
        total_charg_cost = 0
        total_charg_cnt = 0
        for j in range(len(vehicle) - 1):
            total_dist += distance_matrix[vehicle[j], vehicle[j + 1]]
            if vehicle[j] >= consumer_number:
                total_charg_cost += charge_one_cost
                total_charg_cnt += 1
        route[5] = int(total_dist)  # total distance
        route[6] = round(route[5] * trans0, 2)  # transfer cost
        route[7] = total_charg_cost  # total charge cost
        route[8] = total_wait_cost
        route[9] = fix0  # vehicle fixed cost
        route[10] = route[6] + route[7] + route[8] + route[9]  # total cost
        route[11] = total_charg_cnt  # charge count

        result.append(route)
        # print route
        total_cost += distance_matrix[vehicle[-2], 0] * trans0
        # print 'Last leave time: ', int(t) / 60, ':', int(t) % 60
        # print 'total distances: ', route[5]


    if if_write:
        with open('d:/Result_%s_%s.csv' % (file_code, run_t), 'w', newline = '') as fw:
            writer = csv.writer(fw)
            for v in result:
                writer.writerow(v)
    return total_cost

def vehicle_type_adjust(solution):
    """Given a solution only using large truck, check if if we can replace with a small vehicle to save cost."""
    type_list = []
    for veh in solution:
        typ = 2
        wei_shou = [0]  # pickup weight
        wei_song = [0]  # delivery weight
        vol_shou = [0]
        vol_song = [0]
        distance = []  # distance at each point
        for i in range(len(veh) - 1):
            if 2 <= consumer_constraint[veh[i]][0] <= 3:
                distance0 = distance[-1] + distance_matrix[veh[i], veh[i + 1]]
                distance.append(distance0)
            else:
                distance0 = distance_matrix[veh[i], veh[i + 1]]
                distance.append(distance0)

            wei_song0, wei_shou0, vol_song0, vol_shou0 = 0, 0, 0, 0
            if consumer_constraint[veh[i + 1]][0] == 2:
                wei_song0 = wei_song[-1] + consumer_constraint[veh[i + 1]][1]
                vol_song0 = vol_song[-1] + consumer_constraint[veh[i + 1]][2]
            elif consumer_constraint[veh[i + 1]][0] == 3:
                wei_shou0 = wei_shou[-1] + consumer_constraint[veh[i + 1]][1]
                vol_shou0 = vol_shou[-1] + consumer_constraint[veh[i + 1]][2]
            elif veh[i+1] == 0:  # go back to the depot initialize vehicle resources
                wei_song0, wei_shou0, vol_song0, vol_shou0 = 0, 0, 0, 0
            else:
                continue
            wei_song.append(wei_song0)
            wei_shou.append(wei_shou0)
            vol_song.append(vol_song0)
            vol_shou.append(vol_shou0)

        if max(wei_song) > 2.5 or max(wei_shou) > 2.5 or max(vol_song) > 16 or max(vol_shou) > 16 or max(distance) > 120000:
            print( 'Shit!!!')
            print ('Error route: ', veh)
            print ('wei_song wei_shou vol_song vol_shou distance: ', max(wei_song), max(wei_shou), max(vol_song), max(vol_shou), max(distance))
        if max(wei_song) <= small_car_capacity[1] and max(wei_shou) <= small_car_capacity[1] and max(vol_song) <= small_car_capacity[2] and max(vol_shou) <= small_car_capacity[2] and max(distance) <= small_car_capacity[3]:
            typ = 1

        type_list.append(typ)

    return type_list

if __name__ == '__main__':
    start_time = time.time()
    max_run_time = 300

    file_name = '1_1601'
    consumer_number = 1501
    charge_number = 100

    consumer_constraint, time_matrix, distance_matrix = \
        readfile.read_data(file_name, consumer_number, charge_number)

    # 最近充电桩
    consumer_nearest_charge = []
    for i in range(consumer_number):
        charge_distance = copy.deepcopy(distance_matrix[i, :])
        charge_distance[1:consumer_number] = 100000
        min_distance = np.where(charge_distance == np.min(charge_distance))
        near_charge_id = min_distance[0][0]
        consumer_nearest_charge.append(near_charge_id)

    # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    small_car_capacity = [1, 2.0, 12, 100000, 0.012, 200]

    # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    big_car_capacity = [2, 2.5, 16, 120000, 0.014, 300]

    charge_one_cost = 0.5 * 100  # charge cost
    wait_one_hour_cost = 24.0  # waiting cost: 24 yuan/h
    charge_time = 30  # charge time at charge station
    every_round_stay_cost = 60  # stay time at depot after every round
    operation_time = 30  # operation time at each customer
    earliest_start_time = 60 * 8  # earliest start time of a vehicle

    small_vehicle_number = 255
    greedsolve = GreedSolve(small_vehicle_number, consumer_number, time_matrix
                            , distance_matrix, operation_time, charge_time
                            , copy, consumer_constraint, every_round_stay_cost,
                            consumer_nearest_charge)
    greed_output = greedsolve.greed_solve()
    vehicle_type = vehicle_type_adjust(greed_output)
    greed_cost = get_cost(solution=greed_output, all_vehicle_type=vehicle_type, if_write=True)

    simulatedannealing = Simulatedannealing(small_vehicle_number, consumer_number, time_matrix
                                            , distance_matrix, operation_time, charge_time
                                            , copy, consumer_constraint, every_round_stay_cost
                                            , consumer_nearest_charge)
    new_output, cost_t = simulatedannealing.simulated_annealing(sol_in=greed_output,
                                                                veh_type_in=vehicle_type,
                                                                cost_in=greed_cost)
    veh_typ1 = vehicle_type_adjust(new_output)

    new_cost = get_cost(solution=new_output,
                        all_vehicle_type=veh_typ1,
                        if_write=True)
    print('\nTotal used vehicle number: ', len(new_output))
    print('Total cost is: ', new_cost)
    print ('\nTotal used vehicle number: ', len(greed_output))
    print ('Total cost is: ', greed_cost)