import csv
import numpy as np
import time
import copy
import random

class Simulatedannealing(object):
    initial_temp, stop_temp, delta = 40., 10., 30. / 6000

    iveco_cap = [1, 2.0, 12, 100000, 0.012, 200]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    truck_cap = [2, 2.5, 16, 120000, 0.014, 300]  # [veh_type, weight, volume, max_distance, trs_cost, fix_cost]
    charg_cost0 = 0.5 * 100  # charge cost
    wait_cost0 = 24.0  # waiting cost: 24 yuan/h
    charg_t = 30  # charge time at charge station
    depot_t = 60  # stay time at depot after every round
    oper_t = 30  # operation time at each customer
    start_t = 60 * 8  # earliest start time of a vehicle

    file_code = 1

    def __init__(self, small_veh, cust_num, time_mat
                 , dist_mat, oper_t, charg_t
                 , copy, cust_need, depot_t
                 ,cust_charge):
        self.small_veh = small_veh
        self.cust_num = cust_num
        self.time_mat = time_mat
        self.dist_mat = dist_mat
        self.oper_t = oper_t
        self.charg_t = charg_t
        self.copy = copy
        self.cust_need = cust_need
        self.depot_t = depot_t
        self.cust_charge = cust_charge

    def simulated_annealing(self, sol_in, veh_type_in, cost_in):
        """Neighborhood search based on 5 operators. In each iteration, select one operator randomly."""

        t_str = time.time()
        max_run_time = 3600.
        itr_cost = []
        solu = self.copy.deepcopy(sol_in)
        solu_type = self.copy.deepcopy(veh_type_in)
        best_solu = sol_in
        best_val = cost_in
        tabu_list = []
        random.seed(10)
        itr = 0
        temp = self.initial_temp
        t_run = time.time()
        while temp > self.stop_temp and t_run - t_str < max_run_time:
            itr += 1
            print(itr)
            if itr >= 6000:
                sa_lns = True  # use sa or lns
            else:
                sa_lns = False
            c = random.randint(1, self.cust_num - 1)  # randomly generated moving customer
            while c in tabu_list:
                c = random.randint(1, self.cust_num - 1)
            c_loc = self.cust_loc(solu, c)

            if len(solu[c_loc[0]]) < 4:  # customer number less than 2, can only implement shift1 and exchange1 operator
                wheel_value1 = random.uniform(0, 1)
                if wheel_value1 < 0.45:
                    self.shift_1_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                elif wheel_value1 < 0.9:
                    self.exchange_1_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                else:
                    self.two_opt(solu, c, c_loc, temp, solu_type, sa_lns)
            elif len(solu[c_loc[0]]) >= 4 and c_loc[1] <= len(
                    solu[c_loc[0]]) - 3:  # customer number more than 2, can implement all operators
                wheel_value2 = random.uniform(0, 1)
                if wheel_value2 < 0.2:
                    self.shift_1_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                elif wheel_value2 < 0.4:
                    self.shift_2_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                elif wheel_value2 < 0.6:
                    self.exchange_1_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                elif wheel_value2 < 0.8:
                    self.exchange_2_cust(solu, c, c_loc, temp, solu_type, sa_lns)
                else:
                    self.two_opt(solu, c, c_loc, temp, solu_type, sa_lns)

            if itr % 150 == 0:  # implement two-exchange operator every 200 iteration
                self.two_exchange_sol(solu, temp, solu_type, sa_lns)

            temp -= self.delta
            tabu_list.append(c)
            if len(tabu_list) > 200:
                tabu_list.pop(0)

            cost_i = self.get_cost(solu, solu_type, False)
            itr_cost.append(cost_i)
            if cost_i < best_val:
                best_solu = solu
                best_val = cost_i

            t_run = time.time()

        # Adjust0: delete consecutive depots(0)
        adjust_sol0 = []
        for route0 in best_solu:
            if len(route0) <= 2:  # [0, 0] route
                continue
            elif len(route0) == 3 and self.cust_need[route0[1]][0] == 4:  # [0, charge, 0] route
                continue
            else:
                new_route0 = [0]
                for j in range(1, len(route0)):
                    if new_route0[-1] == 0 and route0[j] == 0:  # delete consecutive depots
                        continue
                    elif self.cust_need[new_route0[-1]][0] == 4 and self.cust_need[route0[j]][
                        0] == 4:  # delete consecutive charges
                        continue
                    else:
                        new_route0.append(route0[j])
                adjust_sol0.append(new_route0)

        # Adjust1: delete the last charge station if unnecessary
        adjust_sol1 = []
        for route1 in adjust_sol0:
            if self.cust_need[route1[-2]][
                0] == 4:  # the last was a charge station, to judge if this charge station is necessary
                type1 = self.route_type(route1)
                route1_cost = self.check_violation(route1, type1)[1]
                veh_rout_test = copy.deepcopy(route1)
                veh_rout_test.pop(-2)
                if self.check_violation(veh_rout_test, 1)[0]:
                    if self.check_violation(veh_rout_test, 1)[1] < route1_cost:
                        adjust_sol1.append(veh_rout_test)
                    else:
                        adjust_sol1.append(route1)
                elif self.check_violation(veh_rout_test, 2)[0]:
                    if self.check_violation(veh_rout_test, 2)[1] < route1_cost:
                        adjust_sol1.append(veh_rout_test)
                    else:
                        adjust_sol1.append(route1)
                else:
                    adjust_sol1.append(route1)
            else:
                adjust_sol1.append(route1)
        return adjust_sol1, itr_cost

    def cust_loc(self, sol, cust):
        """Get the route location and customer location of a customer."""
        cust_ind = []  # [route_loc, cust_loc]
        for i, rt in enumerate(sol):
            if cust in rt:
                cust_ind.append(i)
                cust_ind.append(rt.index(cust))
                return cust_ind

        print('Costomer not in the solution: ', cust)

    def shift_1_cust(self, sol_in1, cust, c_loc, curr_temp, sol_type1, sa_lns):
        """Try to move 1 customer to anywhere it can be put, and see if the move can cut the total cost."""

        route_ing = copy.deepcopy(sol_in1[c_loc[0]])
        route_new = route_ing
        move_to_route = c_loc[0]
        new_type = 2
        origin_cost1 = self.check_violation(route_ing, sol_type1[c_loc[0]])[1]
        route_ing.remove(cust)  # move c in the current route
        adjust_cost1 = min(self.check_violation(route_ing, 1)[1], self.check_violation(route_ing, 2)[1])
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        for j, rou in enumerate(sol_in1):
            origin_cost2 = self.check_violation(rou, sol_type1[j])[1]
            if j == c_loc[0]:  # moving in the same route
                for k in range(1, len(route_ing)):
                    if k == c_loc[1]:
                        continue  # do not put it at the original position
                    rou_test = route_ing[:k] + [cust] + route_ing[k:]
                    if self.check_violation(rou_test, 1)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 1)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 1
                    elif self.check_violation(rou_test, 2)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 2)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 2
            else:  # moving to a different route
                for k in range(1, len(rou)):
                    rou_test = rou[:k] + [cust] + rou[k:]
                    if self.check_violation(rou_test, 1)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 1)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 1
                    elif self.check_violation(rou_test, 2)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 2)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 2

        if best_cut_cost > 1e-5:
            print('shift1 good', best_cut_cost)
            sol_in1[move_to_route] = route_new
            sol_type1[move_to_route] = new_type
            if move_to_route != c_loc[0]:  # moving to a different route
                sol_in1[c_loc[0]] = route_ing
                sol_type1[c_loc[0]] = self.route_type(route_ing)
        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('shift1', best_cut_cost)
                sol_in1[move_to_route] = route_new
                sol_type1[move_to_route] = new_type
                if move_to_route != c_loc[0]:  # moving to a different route
                    sol_in1[c_loc[0]] = route_ing
                    sol_type1[c_loc[0]] = self.route_type(route_ing)

        # return sol_in1

    def shift_2_cust(self,sol_in2, cust, c_loc, curr_temp, sol_type2, sa_lns):
        """Try to move 2 consecutive customers to anywhere they can be put, see if they move can cut the total cost."""

        route_ing = copy.deepcopy(sol_in2[c_loc[0]])
        route_new = route_ing
        move_to_route = c_loc[0]
        new_type = 2
        cust_folw = route_ing[c_loc[1] + 1]
        origin_cost1 = self.check_violation(route_ing, sol_type2[c_loc[0]])[1]
        route_ing.remove(cust)  # remove c in the current route
        del route_ing[c_loc[1]]  # remove customer following c
        adjust_cost1 = min(self.check_violation(route_ing, 1)[1], self.check_violation(route_ing, 2)[1])
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        for j, rou in enumerate(sol_in2):
            origin_cost2 = self.check_violation(rou, sol_type2[j])[1]
            if j == c_loc[0]:  # moving in the same route
                for k in range(1, len(route_ing)):
                    if k == c_loc[1]:
                        continue
                    rou_test = route_ing[:k] + [cust, cust_folw] + route_ing[k:]
                    if self.check_violation(rou_test, 1)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 1)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 1
                    elif self.check_violation(rou_test, 2)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 2)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 2
            else:  # moving to a different route
                for k in range(1, len(rou)):
                    rou_test = rou[:k] + [cust, cust_folw] + rou[k:]
                    if self.check_violation(rou_test, 1)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 1)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 1
                    elif self.check_violation(rou_test, 2)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 2)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new = rou_test
                            move_to_route = j
                            new_type = 2

        if best_cut_cost > 1e-5:
            print('shift2 good', best_cut_cost)
            sol_in2[move_to_route] = route_new
            sol_type2[move_to_route] = new_type
            if move_to_route != c_loc[0]:  # moving to a different route
                sol_in2[c_loc[0]] = route_ing
                sol_type2[c_loc[0]] = self.route_type(route_ing)

        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('shift2', best_cut_cost)
                sol_in2[move_to_route] = route_new
                sol_type2[move_to_route] = new_type
                if move_to_route != c_loc[0]:  # moving to a different route
                    sol_in2[c_loc[0]] = route_ing
                    sol_type2[c_loc[0]] = self.route_type(route_ing)

        # return sol_in2

    def exchange_1_cust(self,sol_in3, cust, c_loc, curr_temp, sol_type3, sa_lns):
        """Exchange the position of two customers(same route or not) if feasible, and see if it can cut the total cost."""

        route_ing = copy.deepcopy(sol_in3[c_loc[0]])

        route_new_1 = route_ing
        route_new_2 = route_ing
        exch_to_route = c_loc[0]
        origin_cost1 = self.check_violation(route_ing, sol_type3[c_loc[0]])[1]
        # route_ing.remove(cust)  # move c in the current route
        # adjust_cost1 = check_violation(route_ing)[1]
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        for j, rou in enumerate(sol_in3):
            origin_cost2 = self.check_violation(rou, sol_type3[j])[1]
            if j == c_loc[0]:  # exchange in the same route
                for k in range(1, len(rou) - 1):
                    if k == c_loc[1]:
                        continue
                    rou_test = copy.deepcopy(sol_in3[c_loc[0]])
                    rou_test[k], rou_test[c_loc[1]] = rou_test[c_loc[1]], rou_test[k]
                    if self.check_violation(rou_test, 1)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 1)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test
                            route_new_2 = rou_test
                            exch_to_route = j

                    elif self.check_violation(rou_test, 2)[0]:
                        adjust_cost2 = self.check_violation(rou_test, 2)[1]
                        cost_cut_test = origin_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test
                            route_new_2 = rou_test
                            exch_to_route = j

            else:  # exchange to a different route
                for k in range(1, len(rou) - 1):
                    rou_test_1 = copy.deepcopy(sol_in3[c_loc[0]])
                    rou_test_2 = copy.deepcopy(rou)
                    rou_test_1[c_loc[1]] = rou[k]
                    rou_test_2[k] = cust
                    if self.check_violation(rou_test_1, 1)[0] and self.check_violation(rou_test_2, 1)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 1)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 1)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

                    elif self.check_violation(rou_test_1, 2)[0] and self.check_violation(rou_test_2, 2)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 2)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 2)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

        if best_cut_cost > 1e-5:
            print('exchange1 good', best_cut_cost)
            sol_in3[c_loc[0]] = route_new_1
            sol_in3[exch_to_route] = route_new_2
            sol_type3[c_loc[0]] = self.route_type(route_new_1)
            sol_type3[exch_to_route] = self.route_type(route_new_2)

        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('exchange1', best_cut_cost)
                sol_in3[c_loc[0]] = route_new_1
                sol_in3[exch_to_route] = route_new_2
                sol_type3[c_loc[0]] = self.route_type(route_new_1)
                sol_type3[exch_to_route] = self.route_type(route_new_2)

        # return sol_in3

    def exchange_2_cust(self, sol_in4, cust, c_loc, curr_temp, sol_type4, sa_lns):
        """Exchange 2 consecutive customers' position with another 2 customers' position, and see if it can cut cost."""

        route_ing = copy.deepcopy(sol_in4[c_loc[0]])
        route_new_1 = route_ing
        route_new_2 = route_ing
        cust_folw = route_ing[c_loc[1] + 1]
        exch_to_route = c_loc[0]
        origin_cost1 = self.check_violation(route_ing, sol_type4[c_loc[0]])[1]
        # route_ing.remove(cust)  # move c in the current route
        # adjust_cost1 = check_violation(route_ing)[1]
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        for j, rou in enumerate(sol_in4):
            origin_cost2 = self.check_violation(rou, sol_type4[j])[1]
            if j != c_loc[0] and len(rou) >= 4:  # exchange to a different route
                for k in range(1, len(rou) - 2):
                    rou_test_1 = copy.deepcopy(sol_in4[c_loc[0]])
                    rou_test_2 = copy.deepcopy(rou)
                    rou_test_1[c_loc[1]], rou_test_1[c_loc[1] + 1] = rou[k], rou[k + 1]
                    rou_test_2[k], rou_test_2[k + 1] = cust, cust_folw
                    if self.check_violation(rou_test_1, 1)[0] and self.check_violation(rou_test_2, 1)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 1)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 1)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

                    if self.check_violation(rou_test_1, 2)[0] and self.check_violation(rou_test_2, 2)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 2)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 2)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

        if best_cut_cost > 1e-5:
            print('exchange2 good', best_cut_cost)
            sol_in4[c_loc[0]] = route_new_1
            sol_in4[exch_to_route] = route_new_2
            sol_type4[c_loc[0]] = self.route_type(route_new_1)
            sol_type4[exch_to_route] = self.route_type(route_new_2)

        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('exchange2', best_cut_cost)
                sol_in4[c_loc[0]] = route_new_1
                sol_in4[exch_to_route] = route_new_2
                sol_type4[c_loc[0]] = self.route_type(route_new_1)
                sol_type4[exch_to_route] = self.route_type(route_new_2)

        # return sol_in4

    def two_exchange_sol(self, sol_in5, curr_temp, sol_type5, sa_lns):
        """Two-Exchange operator: For two customers i and j on the same route where i is visited before j,
        remove arcs (i,i+),(j,j+); add arcs (i,j),(i+,j+); and reverse the orientation of the arcs between i+ and j.
        Given a solution, check all possible neighborhood.
        """
        solu = copy.deepcopy(sol_in5)
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        adjust_rou_ind = 0
        route_new = sol_in5[adjust_rou_ind]
        for i, rou in enumerate(solu):
            if len(rou) >= 6:
                origin_cost = self.check_violation(rou, sol_type5[i])[1]
                for k in range(1, len(rou) - 4):
                    for l in range(k + 3, len(rou) - 1):
                        route_test = copy.deepcopy(rou)
                        route_test[k], route_test[l] = route_test[l], route_test[k]
                        route_test[k + 1: l] = route_test[l - 1:k:-1]  # middle reverse
                        if self.check_violation(route_test, 1)[0]:
                            adjust_cost = self.check_violation(route_test, 1)[1]
                            if origin_cost - adjust_cost > best_cut_cost:
                                best_cut_cost = origin_cost - adjust_cost
                                adjust_rou_ind = i
                                route_new = route_test

                        elif self.check_violation(route_test, 2)[0]:
                            adjust_cost = self.check_violation(route_test, 2)[1]
                            if origin_cost - adjust_cost > best_cut_cost:
                                best_cut_cost = origin_cost - adjust_cost
                                adjust_rou_ind = i
                                route_new = route_test

        if best_cut_cost > 1e-5:
            print('2exchange good', best_cut_cost)
            sol_in5[adjust_rou_ind] = route_new
            sol_type5[adjust_rou_ind] = self.route_type(route_new)

        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('2exchange', best_cut_cost)
                sol_in5[adjust_rou_ind] = route_new
                sol_type5[adjust_rou_ind] = self.route_type(route_new)

        # return sol_in5

    def two_opt(self, sol_in7, cust, c_loc, curr_temp, sol_type7, sa_lns):
        """2-opt*: given customer i in route a and customer j in route b, exchange the following sequences of i and j.
        for example, initial route a: ...-i-i1-i2-..., initial route b: ...-j-j1-j2-...
        New route a: ...-i-j1-j2-..., new route b: ...-j-i1-i2-..."""

        route_ing = copy.deepcopy(sol_in7[c_loc[0]])

        route_new_1 = route_ing
        route_new_2 = route_ing
        exch_to_route = c_loc[0]
        origin_cost1 = self.check_violation(route_ing, sol_type7[c_loc[0]])[1]
        # route_ing.remove(cust)  # move c in the current route
        # adjust_cost1 = check_violation(route_ing)[1]
        best_cut_cost0 = -1000
        best_cut_cost = best_cut_cost0  # best cost cut of moving this customer
        for j, rou in enumerate(sol_in7):
            origin_cost2 = self.check_violation(rou, sol_type7[j])[1]
            if j != c_loc[0]:  # 2-opt* operator has to be implemented in 2 different routes
                for k in range(1, len(rou) - 1):
                    rou_test_1 = sol_in7[c_loc[0]][:c_loc[1]] + rou[k:]
                    rou_test_2 = rou[:k] + sol_in7[c_loc[0]][c_loc[1]:]
                    if self.check_violation(rou_test_1, 1)[0] and self.check_violation(rou_test_2, 1)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 1)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 1)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

                    elif self.check_violation(rou_test_1, 2)[0] and self.check_violation(rou_test_2, 2)[0]:
                        adjust_cost1 = self.check_violation(rou_test_1, 2)[1]
                        adjust_cost2 = self.check_violation(rou_test_2, 2)[1]
                        cost_cut_test = origin_cost1 + origin_cost2 - adjust_cost1 - adjust_cost2
                        if cost_cut_test > best_cut_cost:
                            best_cut_cost = cost_cut_test
                            route_new_1 = rou_test_1
                            route_new_2 = rou_test_2
                            exch_to_route = j

        if best_cut_cost > 1e-5:
            print('2opt* good', best_cut_cost)
            sol_in7[c_loc[0]] = route_new_1
            sol_in7[exch_to_route] = route_new_2
            sol_type7[c_loc[0]] = self.route_type(route_new_1)
            sol_type7[exch_to_route] = self.route_type(route_new_2)

        elif sa_lns and best_cut_cost < -1e-5:
            prb = random.uniform(0, 1)
            if np.exp(best_cut_cost / curr_temp) > prb:
                print('2opt*', best_cut_cost)
                sol_in7[c_loc[0]] = route_new_1
                sol_in7[exch_to_route] = route_new_2
                sol_type7[c_loc[0]] = self.route_type(route_new_1)
                sol_type7[exch_to_route] = self.route_type(route_new_2)

    def get_cost(self, solution, veh_type, if_write, run_t=289.3):
        """Given the solution saved in list, calculate the total cost of the solution.
        Write the solution to local in the required format."""

        result = [['trans_code', 'vehicle_type', 'dist_seq', 'distribute_lea_tm', 'distribute_arr_tm', 'distance',
                   'trans_cost', 'charge_cost', 'wait_cost', 'fixed_use_cost', 'total_cost', 'charge_cnt']]
        total_cost = 0
        veh_code = 0
        for k, veh in enumerate(solution):
            if veh_type[k] == 1:
                trans0 = self.iveco_cap[4]
                fix0 = self.iveco_cap[5]
            else:
                trans0 = self.truck_cap[4]
                fix0 = self.truck_cap[5]

            # get the output format
            route = [0] * len(result[0])
            veh_code += 1
            route[0] = 'DP' + str(veh_code).zfill(4)  # vehicle name
            route[1] = veh_type[k]  # vehicle type
            route_ele = []
            for ele in veh:
                if ele == 0:
                    route_ele.append(str(ele))
                else:
                    route_ele.append(str(ele + self.file_code * 10000))
            route[2] = ';'.join(route_ele)  # route

            total_cost += fix0
            total_cost += self.dist_mat[0, veh[1]] * trans0
            if self.time_mat[0, veh[1]] + self.start_t <= self.cust_need[veh[1]][3]:
                t = self.cust_need[veh[1]][3] + self.oper_t
                time_out = int(self.cust_need[veh[1]][3] - self.time_mat[0, veh[1]])
                route[3] = str(time_out / 60) + ':' + str(time_out % 60).zfill(2)  # vehicle out time
            else:

                t = self.time_mat[0, veh[1]] + self.start_t + self.oper_t
                route[3] = str(self.start_t / 60) + ':' + str(self.start_t % 60).zfill(2)  # vehicle out time
            total_wait_cost = 0
            for i in range(2, len(veh) - 1):  # can not wait at the first 2 points
                total_cost += (self.dist_mat[veh[i - 1], veh[i]] * trans0)
                if self.cust_need[veh[i]][0] == 4:
                    total_cost += self.charg_cost0
                wait_t = self.cust_need[veh[i]][3] - (t + self.time_mat[veh[i - 1], veh[i]])
                if wait_t > 0:
                    # print veh[i-1], veh[i], wait_t
                    total_cost += (wait_t / 60. * self.wait_cost0)
                    total_wait_cost += (wait_t / 60. * self.wait_cost0)
                    t = self.cust_need[veh[i]][3] + self.oper_t
                else:
                    if veh[i] == 0:
                        t += (self.time_mat[veh[i - 1], veh[i]] + self.depot_t)
                    else:
                        t += (self.time_mat[veh[i - 1], veh[i]] + self.oper_t)
                if veh[i] == 0:  # get back to the depot and will depart again, wait cost is 1hour
                    total_cost += self.wait_cost0
                    total_wait_cost += self.wait_cost0
                # print veh[i], str(int(t) / 60) + ':' + str(int(t) % 60).zfill(2)

            in_time = int(t + self.time_mat[veh[-2], 0])
            route[4] = str(in_time / 60) + ':' + str(in_time % 60).zfill(2)  # vehicle back time
            total_dist = 0
            total_charg_cost = 0
            total_charg_cnt = 0
            for j in range(len(veh) - 1):
                total_dist += self.dist_mat[veh[j], veh[j + 1]]
                if veh[j] >= self.cust_num:
                    total_charg_cost += self.charg_cost0
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
            total_cost += self.dist_mat[veh[-2], 0] * trans0
            # print 'Last leave time: ', int(t) / 60, ':', int(t) % 60
            # print 'total distances: ', route[5]

        if if_write:
            with open(r'd:/Result_%s_%s1.csv' % (self.file_code, run_t), 'w') as fw:
                writer = csv.writer(fw)
                for v in result:
                    writer.writerow(v)

        return total_cost

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

    def route_type(self,route):
        """Given a route, return the vehicle type of the route. Samll vehicle first, large vehicle second."""
        typ = 2
        wei_shou = [0]  # pickup weight
        wei_song = [0]  # delivery weight
        vol_shou = [0]
        vol_song = [0]
        distance = []  # distance at each point
        for i in range(len(route) - 1):
            if 2 <= self.cust_need[route[i]][0] <= 3:
                distance0 = distance[-1] + self.dist_mat[route[i], route[i + 1]]
                distance.append(distance0)
            else:
                distance0 = self.dist_mat[route[i], route[i + 1]]
                distance.append(distance0)

            wei_song0, wei_shou0, vol_song0, vol_shou0 = 0, 0, 0, 0
            if self.cust_need[route[i + 1]][0] == 2:
                wei_song0 = wei_song[-1] + self.cust_need[route[i + 1]][1]
                vol_song0 = vol_song[-1] + self.cust_need[route[i + 1]][2]
            elif self.cust_need[route[i + 1]][0] == 3:
                wei_shou0 = wei_shou[-1] + self.cust_need[route[i + 1]][1]
                vol_shou0 = vol_shou[-1] + self.cust_need[route[i + 1]][2]
            elif route[i + 1] == 0:  # go back to the depot initialize vehicle resources
                wei_song0, wei_shou0, vol_song0, vol_shou0 = 0, 0, 0, 0
            else:
                continue
            wei_song.append(wei_song0)
            wei_shou.append(wei_shou0)
            vol_song.append(vol_song0)
            vol_shou.append(vol_shou0)

        if max(wei_song) > 2.5 or max(wei_shou) > 2.5 or max(vol_song) > 16 or max(vol_shou) > 16 or max(
                distance) > 120000:
            print('Shit!!!')
            print('Error route: ', route)
            print('wei_song wei_shou vol_song vol_shou distance: ', max(wei_song), max(wei_shou), max(vol_song), max(
                vol_shou), max(distance))
        if max(wei_song) <= self.iveco_cap[1] and max(wei_shou) <= self.iveco_cap[1] and max(vol_song) <= self.iveco_cap[2] and max(
                vol_shou) <= self.iveco_cap[2] and max(distance) <= self.iveco_cap[3]:
            typ = 1

        return typ