class GreedSolve(object):
    trans_cost = 0
    wait_cost = 0
    iveco_cap = [1, 2.0, 12, 100000, 0.012, 200]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    truck_cap = [2, 2.5, 16, 120000, 0.014, 300]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    charg_cost0 = 0.5 * 100  # charge cost
    wait_cost0 = 24.0  # waiting cost: 24 yuan/h
    charg_t = 30  # charge time at charge station
    depot_t = 60  # stay time at depot after every round
    oper_t = 30  # operation time at each customer
    start_t = 60 * 8  # earliest start time of a vehicle
    alp, bet, gam = 0.64, 0.23, 0.13  # weight of t_ij, h_ij and v_ij of Time-oriented Nearest-Neighbor
    # alp, bet, gam = nn_para[f_i]
    small_car_number = 255

    def __init__(self, small_car_number, cust_num, time_mat
                 , dist_mat, oper_t, charg_t
                 , copy, cust_need, depot_t
                 , cust_charge):
        self.small_car_number = small_car_number
        self.cust_num = cust_num
        self.time_mat = time_mat
        self.dist_mat = dist_mat
        self.oper_t = oper_t
        self.charg_t = charg_t
        self.copy = copy
        self.cust_need = cust_need
        self.depot_t = depot_t
        self.cust_charge = cust_charge

    def lns_initial(self):
        """
        This is a Large Neighbour Search algorithm for VRPTW,
        we choose the next visiting customer based on the least waiting time.
        """
        sol = []  # [[0;2;5;0;4;6;0],[],...]
        visited_p = []
        to_vist = [i + 1 for i in range(self.cust_num - 1)]  # [1,5,8,...]
        itr = 0

        while len(to_vist) > 0:
            itr += 1
            if itr < self.small_car_number:
                vehicle_type0 = 1
            else:
                vehicle_type0 = 2
            used_res = [0, 0, 0, 480]  # used weight, volume and distance of the vehicle, leave time
            veh_rout = [0]

            # print '\nA new vehicle will be used.'
            while True:
                curr_cust = veh_rout[-1]
                if len(veh_rout) == 1:
                    last_cust = 0
                else:
                    last_cust = veh_rout[-2]
                # print used_res
                next_one = self.get_next_one_point(last_cust, curr_cust, to_vist, used_res, len(veh_rout), vehicle_type0)
                next_cust, next_start = next_one[0], next_one[1]

                if next_cust == 0:  # next visiting customer is depot
                    if curr_cust == 0:
                        # print 'The current vehicle ends.'
                        break
                    else:
                        used_res[:3] = [0, 0, 0]
                        used_res[3] += (self.time_mat[curr_cust, next_cust] + self.depot_t)
                        # print 'Get back to the depot, and ready for a new round.'

                elif self.cust_need[next_cust][0] == 2:  # next visiting customer is delivery customer
                    used_res[0] += self.cust_need[next_cust][1]
                    used_res[1] += self.cust_need[next_cust][2]
                    used_res[2] += self.dist_mat[curr_cust, next_cust]
                    used_res[3] = (next_start + self.oper_t)

                elif self.cust_need[next_cust][0] == 3:  # next visiting customer is pickup customer
                    if curr_cust == 0:  # current customer is depot
                        used_res[0] = self.cust_need[next_cust][1]
                        used_res[1] = self.cust_need[next_cust][2]
                        used_res[2] = self.dist_mat[curr_cust, next_cust]
                        used_res[3] = (next_start + self.oper_t)
                    elif self.cust_need[curr_cust][0] <= 2:  # current customer is delivery customer
                        used_res[0] = self.cust_need[next_cust][1]
                        used_res[1] = self.cust_need[next_cust][2]
                        used_res[2] += self.dist_mat[curr_cust, next_cust]
                        used_res[3] = (next_start + self.oper_t)
                    else:  # current customer is pickup customer or charge station
                        used_res[0] += self.cust_need[next_cust][1]
                        used_res[1] += self.cust_need[next_cust][2]
                        used_res[2] += self.dist_mat[curr_cust, next_cust]
                        used_res[3] = (next_start + self.oper_t)

                else:  # next visiting customer is a charge station
                    used_res[2] = 0
                    used_res[3] += (self.time_mat[curr_cust, next_cust] + self.charg_t)

                veh_rout.append(next_cust)
                # print 'Vehicle used resource: ', used_res
                if self.cust_need[next_cust][0] == 2 or self.cust_need[next_cust][0] == 3:
                    # print 'visited customer', next_cust
                    to_vist.remove(next_cust)
                    if self.cust_need[next_cust][0] == 3:
                        visited_p.append(next_cust)

            if self.cust_need[veh_rout[-2]][
                0] == 4:  # the last visit was a charge station, to judge if this charge station is necessary
                veh_rout_test = self.copy.deepcopy(veh_rout)
                veh_rout_test.pop(-2)
                if self.check_violation(veh_rout_test, 1)[0]:
                    if self.check_violation(veh_rout_test, 1)[1] < self.check_violation(veh_rout, 1)[1]:
                        sol.append(veh_rout_test)
                        continue
                elif self.check_violation(veh_rout_test, 2)[0]:
                    if self.check_violation(veh_rout_test, 2)[1] < self.check_violation(veh_rout, 1)[1]:
                        sol.append(veh_rout_test)
                        continue
            sol.append(veh_rout)
            # print 'Last point 0 earliest leave time: ', int(used_res[-1]) / 60, ':', int(used_res[-1]) % 60
            # print 'Route %s is: ' % itr, veh_rout
            # print '*'*10, 'Iteration:', itr, '*'*10
        return sol

    def get_next_one_point(self, last_cust, curr_cust, remain_list, used_resource, rout_len, vehicle_type):
        """Given a vehicle and its current visiting customer, return the next visiting customer.
        Here we use the Time-oriented Nearest-Neighborhood Heuristic proposed by Solomon(1987).
        The closeness between customer i and j: C_ij = alp1*t_ij + alp2*h_ij + alp3*v_ij,
        t_ij: travel time between i and j (distance);
        h_ij = b_j - (b_i + s_i), b is start service time, and s is service time (idle time);
        v_ij = l_j - (b_i + s_i + t_ij), l is the latest admissible service time, t_ij is the travel time (urgency)
        The low value of C_ij, the better.
        """
        charg_cost0 = 0.5 * 100  # charge cost
        wait_cost0 = 24.0  # waiting cost: 24 yuan/h
        charg_t = 30  # charge time at charge station
        depot_t = 60  # stay time at depot after every round
        oper_t = 30  # operation time at each customer
        start_t = 60 * 8  # earliest start time of a vehicle
        alp, bet, gam = 0.64, 0.23, 0.13  # weight of t_ij, h_ij and v_ij of Time-oriented Nearest-Neighbor
        # alp, bet, gam = nn_para[f_i]

        if vehicle_type == 1:
            veh_cap = self.iveco_cap
        else:
            veh_cap = self.truck_cap

        visit_cust = [-1, 100000, 600000, 1000]  # [cust_id, next_start, distance, closeness]
        if rout_len > 6:
            pass

        # current customer type is a delivery
        #当前是送货点或者当前是充电桩但是上一个是送货点
        elif self.cust_need[curr_cust][0] <= 2 \
                or \
                (self.cust_need[curr_cust][0] == 4
                 # 充电站
                and
                # 上一个为delivery
                self.cust_need[last_cust][0] <= 2):

            for remain_point in remain_list:
                near_charg_id = self.cust_charge[remain_point]

                # next customer is a delivery
                if self.cust_need[remain_point][0] == 2:
                    if used_resource[2] + self.dist_mat[curr_cust, remain_point] + self.dist_mat[remain_point, near_charg_id] > veh_cap[3]:
                        # print 'd out of battery 1'
                        continue  # run out of battery before getting to the nearest charge station of the visiting customer
                    elif self.dist_mat[curr_cust, remain_point] > veh_cap[3] - used_resource[2]:
                        # print 'd out of battery 2'
                        continue  # run out of battery before getting to the visiting customer
                    elif used_resource[0] + self.cust_need[remain_point][1] > veh_cap[1]:
                        # print 'd weight overload'
                        continue  # weight overload
                    elif used_resource[1] + self.cust_need[remain_point][2] > veh_cap[2]:
                        # print 'd volume overload'
                        continue  # volume overload
                    elif used_resource[3] + self.time_mat[curr_cust, remain_point] > self.cust_need[remain_point][4]:
                        # print 'd last receive time'
                        continue  # can not arrive before last receive time

                    else:
                        wait_time = self.cust_need[remain_point][3] - (used_resource[3] +
                                                               self.time_mat[curr_cust, remain_point])
                        if wait_time < 0:
                            next_start = used_resource[3] + self.time_mat[curr_cust, remain_point]
                            h_ij = self.time_mat[curr_cust, remain_point]
                        else:
                            next_start = self.cust_need[remain_point][3]
                            h_ij = next_start - used_resource[3]
                        v_ij = self.cust_need[remain_point][4] - (used_resource[3] +
                                                          self.time_mat[curr_cust, remain_point])
                        close_ij = alp * self.time_mat[
                            curr_cust, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                        if close_ij < visit_cust[3]:
                            visit_cust[0] = remain_point
                            visit_cust[1] = next_start
                            visit_cust[2] = self.dist_mat[curr_cust, remain_point]
                            visit_cust[3] = close_ij
                        else:
                            continue

                else:  # next customer is a pickup
                    if used_resource[2] + self.dist_mat[curr_cust, remain_point] + self.dist_mat[remain_point, near_charg_id] > veh_cap[3]:
                        # print 'p out of battery 1'
                        continue  # run out of battery before getting to the nearest charge station of the visiting customer
                    elif self.dist_mat[curr_cust, remain_point] > veh_cap[3] - used_resource[2]:
                        # print 'p out of battery 2'
                        continue  # run out of battery before getting to the visiting customer
                    elif used_resource[3] + self.time_mat[curr_cust, remain_point] > self.cust_need[remain_point][4]:
                        # print 'p last receive time'
                        continue  # can not arrive before last receive time

                    else:
                        wait_time = self.cust_need[remain_point][3] - (used_resource[3] +
                                                               self.time_mat[curr_cust, remain_point])
                        if wait_time < 0:
                            next_start = used_resource[3] + self.time_mat[curr_cust, remain_point]
                            h_ij = self.time_mat[curr_cust, remain_point]
                        else:
                            next_start = self.cust_need[remain_point][3]
                            h_ij = next_start - used_resource[3]
                        v_ij = self.cust_need[remain_point][4] - (used_resource[3] + self.time_mat[curr_cust, remain_point])
                        close_ij = alp * self.time_mat[
                            curr_cust, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                        if close_ij < visit_cust[3]:
                            visit_cust[0] = remain_point
                            visit_cust[1] = next_start
                            visit_cust[2] = self.dist_mat[curr_cust, remain_point]
                            visit_cust[3] = close_ij
                        else:
                            continue


        else:  # current customer is a pickup
            for remain_point in remain_list:
                near_charg_id = self.cust_charge[remain_point]
                if self.cust_need[remain_point][0] == 2:
                    continue  # not delivery customer any more
                elif used_resource[2] + self.dist_mat[curr_cust, remain_point] + \
                        self.dist_mat[remain_point, near_charg_id] > veh_cap[3]:
                    continue  # run out of battery before getting to the nearest charge station of the visiting customer
                elif self.dist_mat[curr_cust, remain_point] > veh_cap[3] - used_resource[2]:
                    continue  # run out of battery before getting to the visiting customer
                elif used_resource[0] + self.cust_need[remain_point][1] > veh_cap[1]:
                    continue  # weight overload
                elif used_resource[1] + self.cust_need[remain_point][2] > veh_cap[2]:
                    continue  # volume overload
                elif used_resource[3] + self.time_mat[curr_cust, remain_point] > self.cust_need[remain_point][4]:
                    continue  # can not arrive before last receive time

                else:
                    wait_time = self.cust_need[remain_point][3] - (used_resource[3] + self.time_mat[curr_cust, remain_point])
                    if wait_time < 0:
                        next_start = used_resource[3] + self.time_mat[curr_cust, remain_point]
                        h_ij = self.time_mat[curr_cust, remain_point]
                    else:
                        next_start = self.cust_need[remain_point][3]
                        h_ij = next_start - used_resource[3]
                    v_ij = self.cust_need[remain_point][4] - (used_resource[3] + self.time_mat[curr_cust, remain_point])
                    close_ij = alp * self.time_mat[curr_cust, remain_point] + bet * h_ij + gam * v_ij  # closeness between i and j
                    if close_ij < visit_cust[3]:
                        visit_cust[0] = remain_point
                        visit_cust[1] = next_start
                        visit_cust[2] = self.dist_mat[curr_cust, remain_point]
                        visit_cust[3] = close_ij
                    else:
                        continue

        if visit_cust[0] == -1:  # no customer to visit
            if 2 <= self.cust_need[curr_cust][0] <= 3:  # for customers, first choose to get charged if no customers to visit
                visit_cust[0] = self.cust_charge[curr_cust]  # get to the nearest charge station
                visit_cust[1] = used_resource[-1] + self.time_mat[curr_cust, visit_cust[0]]
            else:  # for charge stations and depot, go back to depot if no customers to visit
                visit_cust[0] = 0
                visit_cust[1] = used_resource[-1] + self.time_mat[curr_cust, 0]
        return visit_cust

    def check_violation(self, route, vehicle_type):
        """To check if a route is feasible using large vehicle(truck), and return check result and route cost."""
        if len(route) == 2:  # [0, 0] route
            return True, 0
        elif len(route) == 3 and self.cust_need[route[1]][0] == 4:  # [0, charge, 0] route
            return True, 0
        else:
            # 0the leaving time, 1accumulated distance, 2accumulated weight_song, 3accumulated volume_song,
            # 4accumulated weight_shou, 5accumulated volume_shou, when arriving at a customer
            accu_res = [480, 0, 0, 0, 0, 0]
            if vehicle_type == 1:
                veh_cap = self.iveco_cap
            elif vehicle_type == 2:
                veh_cap = self.truck_cap
            else:
                print('Input wrong vehicle type!', vehicle_type)

            fixed_cost = veh_cap[5]
            trans_cost = 0
            wait_cost = 0
            charge_cost = 0
            if self.time_mat[0, route[1]] + 480 < self.cust_need[route[1]][3]:
                accu_res[0] = self.cust_need[route[1]][3] - self.time_mat[0, route[1]]
            for i in range(len(route) - 1):
                last_cust = route[i]
                curr_cust = route[i + 1]

                # checking leaving time
                arr_time = accu_res[0] + self.time_mat[last_cust, curr_cust]
                if arr_time < self.cust_need[curr_cust][3]:
                    accu_res[0] = self.cust_need[curr_cust][3] + self.oper_t
                    wait_time = self.cust_need[curr_cust][3] - arr_time
                    wait_cost += (wait_time / 60. * self.wait_cost0)
                elif arr_time <= self.cust_need[curr_cust][4]:
                    accu_res[0] = arr_time + self.oper_t
                else:
                    # print 'Infeasible route!(Service Time Error.)'
                    return False, 1000000

                # checking vehicle max distance
                trans_cost += (self.dist_mat[last_cust, curr_cust] * veh_cap[4])

                if 2 <= self.cust_need[last_cust][0] <= 3:
                    accu_res[1] += self.dist_mat[last_cust, curr_cust]
                else:
                    accu_res[1] = self.dist_mat[last_cust, curr_cust]
                if accu_res[1] > veh_cap[3]:
                    # print 'Infeasible route!(Max Distance Error.)'
                    return False, 1000000

                # checking vehicle max weight and volume
                if self.cust_need[curr_cust][0] == 1:
                    accu_res[2:] = [0, 0, 0, 0]
                elif self.cust_need[curr_cust][0] == 2:
                    accu_res[2] += self.cust_need[curr_cust][1]
                    accu_res[3] += self.cust_need[curr_cust][2]
                elif self.cust_need[curr_cust][0] == 3:
                    accu_res[4] += self.cust_need[curr_cust][1]
                    accu_res[5] += self.cust_need[curr_cust][2]
                else:
                    pass

                if accu_res[2] > veh_cap[1] or accu_res[3] > veh_cap[2] or accu_res[4] > veh_cap[1] or accu_res[5] > \
                        veh_cap[2]:
                    # print 'Infeasible route!(Max Weight/Volume Error.)'
                    return False, 1000000

                if self.cust_need[last_cust][0] == 4:
                    charge_cost += self.charg_cost0

        # print trans_cost, wait_cost, charge_cost, fixed_cost
        return True, trans_cost + wait_cost + charge_cost + fixed_cost
