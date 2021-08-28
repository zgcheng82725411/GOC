class GreedSolve(object):
    trans_cost = 0
    wait_cost = 0
    small_car_capacity = [1, 2.0, 12, 100000, 0.012, 200]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    big_car_capacity = [2, 2.5, 16, 120000, 0.014, 300]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    charge_one_cost = 0.5 * 100  # charge cost
    wait_one_hour_cost = 24.0  # waiting cost: 24 yuan/h
    charge_time = 30  # charge time at charge station
    every_round_stay_cost = 60  # stay time at depot after every round
    operation_time = 30  # operation time at each customer
    earliest_start_time = 60 * 8  # earliest start time of a vehicle

    small_vehicle_number = 255

    def __init__(self, small_vehicle_number, consumer_number, time_matrix
                 , distance_matrix, operation_time, charge_time
                 , copy, consumer_constraint, every_round_stay_cost
                 , consumer_nearest_charge):
        self.small_vehicle_number = small_vehicle_number
        self.consumer_number = consumer_number
        self.time_matrix = time_matrix
        self.distance_matrix = distance_matrix
        self.operation_time = operation_time
        self.charge_time = charge_time
        self.copy = copy
        self.consumer_constraint = consumer_constraint
        self.every_round_stay_cost = every_round_stay_cost
        self.consumer_nearest_charge = consumer_nearest_charge

    def greed_solve(self):
        """
        This is a Large Neighbour Search algorithm for VRPTW,
        we choose the next visiting customer based on the least waiting time.
        """
        greed_solution = []  # [[0;2;5;0;4;6;0],[],...]
        have_visited_point = []
        will_visit_point = [i + 1 for i in range(self.consumer_number - 1)]  # [1,5,8,...]
        iteration_count = 0
        while len(will_visit_point) > 0:
            iteration_count += 1
            if iteration_count < self.small_vehicle_number:
                vehicle_type = 1
            else:
                vehicle_type = 2

            weight_volume_distance_leavetime = [0, 0, 0, 480]  # used weight, volume and distance of the vehicle, leave time
            vehicle_route = [0]
            # print '\nA new vehicle will be used.'
            while True:
                current_consumer = vehicle_route[-1]
                if len(vehicle_route) == 1:
                    last_consumer = 0
                else:
                    last_consumer = vehicle_route[-2]
                # print used_res
                next_one_point = self.get_next_one_point(last_consumer, current_consumer, will_visit_point, weight_volume_distance_leavetime, len(vehicle_route), vehicle_type)
                next_consumer, next_start_time = next_one_point[0], next_one_point[1]

                if next_consumer == 0:  # next visiting customer is depot
                    if current_consumer == 0:
                        # print 'The current vehicle ends.'
                        break
                    else:
                        weight_volume_distance_leavetime[:3] = [0, 0, 0]
                        weight_volume_distance_leavetime[3] += (self.time_matrix[current_consumer, next_consumer] + self.every_round_stay_cost)
                        # print 'Get back to the depot, and ready for a new round.'

                elif self.consumer_constraint[next_consumer][0] == 2:  # next visiting customer is delivery customer
                    weight_volume_distance_leavetime[0] += self.consumer_constraint[next_consumer][1]
                    weight_volume_distance_leavetime[1] += self.consumer_constraint[next_consumer][2]
                    weight_volume_distance_leavetime[2] += self.distance_matrix[current_consumer, next_consumer]
                    weight_volume_distance_leavetime[3] = (next_start_time + self.operation_time)

                elif self.consumer_constraint[next_consumer][0] == 3:  # next visiting customer is pickup customer
                    if current_consumer == 0:  # current customer is depot
                        weight_volume_distance_leavetime[0] = self.consumer_constraint[next_consumer][1]
                        weight_volume_distance_leavetime[1] = self.consumer_constraint[next_consumer][2]
                        weight_volume_distance_leavetime[2] = self.distance_matrix[current_consumer, next_consumer]
                        weight_volume_distance_leavetime[3] = (next_start_time + self.operation_time)
                    elif self.consumer_constraint[current_consumer][0] <= 2:  # current customer is delivery customer
                        weight_volume_distance_leavetime[0] = self.consumer_constraint[next_consumer][1]
                        weight_volume_distance_leavetime[1] = self.consumer_constraint[next_consumer][2]
                        weight_volume_distance_leavetime[2] += self.distance_matrix[current_consumer, next_consumer]
                        weight_volume_distance_leavetime[3] = (next_start_time + self.operation_time)
                    else:  # current customer is pickup customer or charge station
                        weight_volume_distance_leavetime[0] += self.consumer_constraint[next_consumer][1]
                        weight_volume_distance_leavetime[1] += self.consumer_constraint[next_consumer][2]
                        weight_volume_distance_leavetime[2] += self.distance_matrix[current_consumer, next_consumer]
                        weight_volume_distance_leavetime[3] = (next_start_time + self.operation_time)

                else:  # next visiting customer is a charge station
                    weight_volume_distance_leavetime[2] = 0
                    weight_volume_distance_leavetime[3] += (self.time_matrix[current_consumer, next_consumer] + self.charge_time)

                vehicle_route.append(next_consumer)
                # print 'Vehicle used resource: ', used_res
                if self.consumer_constraint[next_consumer][0] == 2 or self.consumer_constraint[next_consumer][0] == 3:
                    # print 'visited customer', next_cust
                    will_visit_point.remove(next_consumer)
                    if self.consumer_constraint[next_consumer][0] == 3:
                        have_visited_point.append(next_consumer)

            if self.consumer_constraint[vehicle_route[-2]][
                0] == 4:  # the last visit was a charge station, to judge if this charge station is necessary
                veh_rout_test = self.copy.deepcopy(vehicle_route)
                veh_rout_test.pop(-2)
                if self.check_violation(veh_rout_test, 1)[0]:
                    if self.check_violation(veh_rout_test, 1)[1] < self.check_violation(vehicle_route, 1)[1]:
                        greed_solution.append(veh_rout_test)
                        continue
                elif self.check_violation(veh_rout_test, 2)[0]:
                    if self.check_violation(veh_rout_test, 2)[1] < self.check_violation(vehicle_route, 1)[1]:
                        greed_solution.append(veh_rout_test)
                        continue
            greed_solution.append(vehicle_route)
            # print 'Last point 0 earliest leave time: ', int(used_res[-1]) / 60, ':', int(used_res[-1]) % 60
            # print 'Route %s is: ' % itr, veh_rout
            # print '*'*10, 'Iteration:', itr, '*'*10
        return greed_solution

    def get_next_one_point(self, last_consumer, current_consumer, will_visit_point, weight_volume_distance_leavetime, route_length, vehicle_type):
        """Given a vehicle and its current visiting customer, return the next visiting customer.
        Here we use the Time-oriented Nearest-Neighborhood Heuristic proposed by Solomon(1987).
        The closeness between customer i and j: C_ij = alp1*t_ij + alp2*h_ij + alp3*v_ij,
        t_ij: travel time between i and j (distance);
        h_ij = b_j - (b_i + s_i), b is start service time, and s is service time (idle time);
        v_ij = l_j - (b_i + s_i + t_ij), l is the latest admissible service time, t_ij is the travel time (urgency)
        The low value of C_ij, the better.
        """
        distance_weight, bet, gam = 0.64, 0.23, 0.13  # weight of t_ij, h_ij and v_ij of Time-oriented Nearest-Neighbor
        # alp, bet, gam = nn_para[f_i]

        if vehicle_type == 1:
            veh_cap = self.small_car_capacity
        else:
            veh_cap = self.big_car_capacity

        id_nextstart_distance_closeness = [-1, 100000, 600000, 1000]  # [cust_id, next_start, distance, closeness]
        if route_length > 6:
            pass
        # current customer type is a delivery
        #当前是送货点或者当前是充电桩但是上一个是送货点
        elif self.consumer_constraint[current_consumer][0] <= 2 \
                or \
                (self.consumer_constraint[current_consumer][0] == 4
                 # 充电站
                 and
                # 上一个为delivery
                 self.consumer_constraint[last_consumer][0] <= 2):

            for remain_point in will_visit_point:
                near_charge_id = self.consumer_nearest_charge[remain_point]

                # next customer is a delivery
                if self.consumer_constraint[remain_point][0] == 2:
                    if weight_volume_distance_leavetime[2] + self.distance_matrix[current_consumer, remain_point] + self.distance_matrix[remain_point, near_charge_id] > veh_cap[3]:
                        # print 'd out of battery 1'
                        continue  # run out of battery before getting to the nearest charge station of the visiting customer
                    elif self.distance_matrix[current_consumer, remain_point] > veh_cap[3] - weight_volume_distance_leavetime[2]:
                        # print 'd out of battery 2'
                        continue  # run out of battery before getting to the visiting customer
                    elif weight_volume_distance_leavetime[0] + self.consumer_constraint[remain_point][1] > veh_cap[1]:
                        # print 'd weight overload'
                        continue  # weight overload
                    elif weight_volume_distance_leavetime[1] + self.consumer_constraint[remain_point][2] > veh_cap[2]:
                        # print 'd volume overload'
                        continue  # volume overload
                    elif weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point] > self.consumer_constraint[remain_point][4]:
                        # print 'd last receive time'
                        continue  # can not arrive before last receive time

                    else:
                        wait_time = self.consumer_constraint[remain_point][3] - (weight_volume_distance_leavetime[3] +
                                                                                 self.time_matrix[current_consumer, remain_point])
                        if wait_time < 0:
                            next_start = weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point]
                            h_ij = self.time_matrix[current_consumer, remain_point]
                        else:
                            next_start = self.consumer_constraint[remain_point][3]
                            h_ij = next_start - weight_volume_distance_leavetime[3]
                        v_ij = self.consumer_constraint[remain_point][4] - (weight_volume_distance_leavetime[3] +
                                                                            self.time_matrix[current_consumer, remain_point])
                        close_ij = distance_weight * self.time_matrix[
                            current_consumer, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                        if close_ij < id_nextstart_distance_closeness[3]:
                            id_nextstart_distance_closeness[0] = remain_point
                            id_nextstart_distance_closeness[1] = next_start
                            id_nextstart_distance_closeness[2] = self.distance_matrix[current_consumer, remain_point]
                            id_nextstart_distance_closeness[3] = close_ij
                        else:
                            continue

                else:  # next customer is a pickup
                    if weight_volume_distance_leavetime[2] + self.distance_matrix[current_consumer, remain_point] + self.distance_matrix[remain_point, near_charge_id] > veh_cap[3]:
                        # print 'p out of battery 1'
                        continue  # run out of battery before getting to the nearest charge station of the visiting customer
                    elif self.distance_matrix[current_consumer, remain_point] > veh_cap[3] - weight_volume_distance_leavetime[2]:
                        # print 'p out of battery 2'
                        continue  # run out of battery before getting to the visiting customer
                    elif weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point] > self.consumer_constraint[remain_point][4]:
                        # print 'p last receive time'
                        continue  # can not arrive before last receive time

                    else:
                        wait_time = self.consumer_constraint[remain_point][3] - (weight_volume_distance_leavetime[3] +
                                                                                 self.time_matrix[current_consumer, remain_point])
                        if wait_time < 0:
                            next_start = weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point]
                            h_ij = self.time_matrix[current_consumer, remain_point]
                        else:
                            next_start = self.consumer_constraint[remain_point][3]
                            h_ij = next_start - weight_volume_distance_leavetime[3]
                        v_ij = self.consumer_constraint[remain_point][4] - (weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point])
                        close_ij = distance_weight * self.time_matrix[
                            current_consumer, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                        if close_ij < id_nextstart_distance_closeness[3]:
                            id_nextstart_distance_closeness[0] = remain_point
                            id_nextstart_distance_closeness[1] = next_start
                            id_nextstart_distance_closeness[2] = self.distance_matrix[current_consumer, remain_point]
                            id_nextstart_distance_closeness[3] = close_ij
                        else:
                            continue


        else:  # current customer is a pickup
            for remain_point in will_visit_point:
                near_charge_id = self.consumer_nearest_charge[remain_point]
                if self.consumer_constraint[remain_point][0] == 2:
                    continue  # not delivery customer any more
                elif weight_volume_distance_leavetime[2] + self.distance_matrix[current_consumer, remain_point] + \
                        self.distance_matrix[remain_point, near_charge_id] > veh_cap[3]:
                    continue  # run out of battery before getting to the nearest charge station of the visiting customer
                elif self.distance_matrix[current_consumer, remain_point] > veh_cap[3] - weight_volume_distance_leavetime[2]:
                    continue  # run out of battery before getting to the visiting customer
                elif weight_volume_distance_leavetime[0] + self.consumer_constraint[remain_point][1] > veh_cap[1]:
                    continue  # weight overload
                elif weight_volume_distance_leavetime[1] + self.consumer_constraint[remain_point][2] > veh_cap[2]:
                    continue  # volume overload
                elif weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point] > self.consumer_constraint[remain_point][4]:
                    continue  # can not arrive before last receive time

                else:
                    wait_time = self.consumer_constraint[remain_point][3] - (weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point])
                    if wait_time < 0:
                        next_start = weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point]
                        h_ij = self.time_matrix[current_consumer, remain_point]
                    else:
                        next_start = self.consumer_constraint[remain_point][3]
                        h_ij = next_start - weight_volume_distance_leavetime[3]
                    v_ij = self.consumer_constraint[remain_point][4] - (weight_volume_distance_leavetime[3] + self.time_matrix[current_consumer, remain_point])
                    close_ij = distance_weight * self.time_matrix[current_consumer, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                    if close_ij < id_nextstart_distance_closeness[3]:
                        id_nextstart_distance_closeness[0] = remain_point
                        id_nextstart_distance_closeness[1] = next_start
                        id_nextstart_distance_closeness[2] = self.distance_matrix[current_consumer, remain_point]
                        id_nextstart_distance_closeness[3] = close_ij
                    else:
                        continue

        if id_nextstart_distance_closeness[0] == -1:  # no customer to visit
            if 2 <= self.consumer_constraint[current_consumer][0] <= 3:  # for customers, first choose to get charged if no customers to visit
                id_nextstart_distance_closeness[0] = self.consumer_nearest_charge[current_consumer]  # get to the nearest charge station
                id_nextstart_distance_closeness[1] = weight_volume_distance_leavetime[-1] + self.time_matrix[current_consumer, id_nextstart_distance_closeness[0]]
            else:  # for charge stations and depot, go back to depot if no customers to visit
                id_nextstart_distance_closeness[0] = 0
                id_nextstart_distance_closeness[1] = weight_volume_distance_leavetime[-1] + self.time_matrix[current_consumer, 0]
        return id_nextstart_distance_closeness

    def check_violation(self, route, vehicle_type):
        """To check if a route is feasible using large vehicle(truck), and return check result and route cost."""
        if len(route) == 2:  # [0, 0] route
            return True, 0
        elif len(route) == 3 and self.consumer_constraint[route[1]][0] == 4:  # [0, charge, 0] route
            return True, 0
        else:
            # 0the leaving time, 1accumulated distance, 2accumulated weight_song, 3accumulated volume_song,
            # 4accumulated weight_shou, 5accumulated volume_shou, when arriving at a customer
            accu_res = [480, 0, 0, 0, 0, 0]
            if vehicle_type == 1:
                veh_cap = self.small_car_capacity
            elif vehicle_type == 2:
                veh_cap = self.big_car_capacity
            else:
                print('Input wrong vehicle type!', vehicle_type)

            fixed_cost = veh_cap[5]
            trans_cost = 0
            wait_cost = 0
            charge_cost = 0
            if self.time_matrix[0, route[1]] + 480 < self.consumer_constraint[route[1]][3]:
                accu_res[0] = self.consumer_constraint[route[1]][3] - self.time_matrix[0, route[1]]
            for i in range(len(route) - 1):
                last_cust = route[i]
                curr_cust = route[i + 1]

                # checking leaving time
                arr_time = accu_res[0] + self.time_matrix[last_cust, curr_cust]
                if arr_time < self.consumer_constraint[curr_cust][3]:
                    accu_res[0] = self.consumer_constraint[curr_cust][3] + self.operation_time
                    wait_time = self.consumer_constraint[curr_cust][3] - arr_time
                    wait_cost += (wait_time / 60. * self.wait_one_hour_cost)
                elif arr_time <= self.consumer_constraint[curr_cust][4]:
                    accu_res[0] = arr_time + self.operation_time
                else:
                    # print 'Infeasible route!(Service Time Error.)'
                    return False, 1000000

                # checking vehicle max distance
                trans_cost += (self.distance_matrix[last_cust, curr_cust] * veh_cap[4])

                if 2 <= self.consumer_constraint[last_cust][0] <= 3:
                    accu_res[1] += self.distance_matrix[last_cust, curr_cust]
                else:
                    accu_res[1] = self.distance_matrix[last_cust, curr_cust]
                if accu_res[1] > veh_cap[3]:
                    # print 'Infeasible route!(Max Distance Error.)'
                    return False, 1000000

                # checking vehicle max weight and volume
                if self.consumer_constraint[curr_cust][0] == 1:
                    accu_res[2:] = [0, 0, 0, 0]
                elif self.consumer_constraint[curr_cust][0] == 2:
                    accu_res[2] += self.consumer_constraint[curr_cust][1]
                    accu_res[3] += self.consumer_constraint[curr_cust][2]
                elif self.consumer_constraint[curr_cust][0] == 3:
                    accu_res[4] += self.consumer_constraint[curr_cust][1]
                    accu_res[5] += self.consumer_constraint[curr_cust][2]
                else:
                    pass

                if accu_res[2] > veh_cap[1] or accu_res[3] > veh_cap[2] or accu_res[4] > veh_cap[1] or accu_res[5] > \
                        veh_cap[2]:
                    # print 'Infeasible route!(Max Weight/Volume Error.)'
                    return False, 1000000

                if self.consumer_constraint[last_cust][0] == 4:
                    charge_cost += self.charge_one_cost

        # print trans_cost, wait_cost, charge_cost, fixed_cost
        return True, trans_cost + wait_cost + charge_cost + fixed_cost
