import os
import sys
import math
import traci
import json


class ConflictPoint:
    def __init__(self, point_id, x, y):
        self.id = point_id
        self.x, self.y = x, y
        self.ApproachCar = {}
        self.PassedCars = set()
        self.last_exit_time = 0

        self.safety_gap = 1.2
        self.occ_length = 5.0

    def UpdateCar(self, car_id, car_x, car_y, current_sim_time, orig_speed):
        if car_id in self.PassedCars: return

        # 1. 예약용 거리: 실제 도로 경로 기준
        try:
            route_dist = traci.vehicle.getDrivingDistance2D(car_id, self.x, self.y)
            if route_dist < 0 or route_dist > 1000:
                route_dist = math.dist((self.x, self.y), (car_x, car_y))
        except:
            route_dist = math.dist((self.x, self.y), (car_x, car_y))

        # 2. 물리적 판정용 거리
        euc_dist = math.dist((self.x, self.y), (car_x, car_y))

        if not self.ApproachCar and current_sim_time > self.last_exit_time:
            self.last_exit_time = current_sim_time

        # 현재 흘러간 시간과 줄어든 거리의 비율을 통해 물리적인 지연 시간을 산출
        earliest_arrival = current_sim_time + (route_dist / orig_speed)

        # [신규 예약 단계]
        if car_id not in self.ApproachCar:
            start_time = max(earliest_arrival, self.last_exit_time + self.safety_gap)
            exit_time = start_time + 1.5

            self.last_exit_time = exit_time
            self.ApproachCar[car_id] = {
                'start': start_time,
                'exit': exit_time,
                'dist': route_dist,
                'euc_dist': euc_dist,
                'prev_euc': euc_dist
            }

        # [업데이트 및 예약 연기 로직]
        else:
            car_list = list(self.ApproachCar.keys())
            try:
                my_idx = car_list.index(car_id)
            except ValueError:
                return

            # 🚨 핵심 수정: 1등 차량은 자기 자신의 예약만 신경 씁니다.
            if my_idx == 0:
                possible_start = max(earliest_arrival, self.ApproachCar[car_id]['start'])
            else:
                front_car = car_list[my_idx - 1]
                reference_time = self.ApproachCar[front_car]['exit']
                possible_start = max(earliest_arrival, reference_time + self.safety_gap)

            current_start = self.ApproachCar[car_id]['start']

            if abs(possible_start - current_start) > 0.2:
                time_diff = possible_start - current_start
                self.ApproachCar[car_id]['start'] = possible_start
                self.ApproachCar[car_id]['exit'] = possible_start + 1.8

                # 지연 연쇄 전파 (Delay Propagation)
                if time_diff > 0:
                    for i in range(my_idx + 1, len(car_list)):
                        rear_car = car_list[i]
                        self.ApproachCar[rear_car]['start'] += time_diff
                        self.ApproachCar[rear_car]['exit'] += time_diff

                self.last_exit_time = self.ApproachCar[car_list[-1]]['exit']

        # [물리 기반 해제 단계]
        if car_id in self.ApproachCar:
            prev_euc = self.ApproachCar[car_id]['prev_euc']

            is_passed = (euc_dist > prev_euc and euc_dist > 6.0)

            if is_passed:
                self.PassedCars.add(car_id)
                del self.ApproachCar[car_id]

                if not self.ApproachCar:
                    self.last_exit_time = current_sim_time
            else:
                self.ApproachCar[car_id]['prev_euc'] = euc_dist
                self.ApproachCar[car_id]['dist'] = route_dist

    def GetRequiredSpeed(self, car_id, current_time, orig_speed):
        if car_id not in self.ApproachCar: return orig_speed

        # =========================================================
        # 🎯 [핵심 해결책] 내가 대기열 1등인가?
        # 1등이라는 건 물리적으로 CP가 완전히 비어있다는 뜻입니다.
        # 예약 시간(start)을 기다리며 멈칫할 필요 없이 무조건 제 속도로 달립니다!
        # =========================================================
        car_list = list(self.ApproachCar.keys())
        if car_list and car_list[0] == car_id:
            return orig_speed

        # --- 이 아래는 2등부터 적용되는 로직 (기존과 동일) ---
        data = self.ApproachCar[car_id]
        time_left = data['start'] - current_time

        if time_left <= 0: return orig_speed

        req_v = data['dist'] / max(time_left, 0.1)

        # 거리가 멀 때는 최소 30% 유지
        req_v = max(req_v, orig_speed * 0.3)

        # CP 코앞(5m 이내)에서는 멈칫하지 않게 40% 유지
        if data['dist'] < 5.0:
            req_v = max(req_v, orig_speed * 0.4)

        return min(orig_speed, max(0.1, req_v))

# =========================================================
# ⚙️ SUMO 및 시나리오 초기화
# =========================================================
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("환경 변수 'SUMO_HOME'을 설정해주세요.")

sumoCmd = ["sumo-gui", "-c", "intersection.sumocfg", "--collision.action", "none", "--step-length", "0.1"]
traci.start(sumoCmd)

routes = {
    "route_N_to_S": ["n_in", "s_out"], "route_S_to_N": ["s_in", "n_out"],
    "route_E_to_W": ["e_in", "w_out"], "route_W_to_E": ["w_in", "e_out"],
    "route_N_to_E": ["n_in", "e_out"], "route_S_to_W": ["s_in", "w_out"],
    "route_E_to_S": ["e_in", "s_out"], "route_W_to_N": ["w_in", "n_out"],
    "route_N_to_W": ["n_in", "w_out"], "route_E_to_N": ["e_in", "n_out"],
    "route_W_to_S": ["w_in", "s_out"], "route_S_to_E": ["s_in", "e_out"]
}

for r_id, edges in routes.items():
    try:
        traci.route.add(r_id, edges)
    except:
        pass

conflict_points = {
    "cp_1": ConflictPoint("cp_1", -5.25, 5.25), "cp_2": ConflictPoint("cp_2", -1.75, 5.25),
    "cp_3": ConflictPoint("cp_3", 1.75, 5.25), "cp_4": ConflictPoint("cp_4", 5.25, 5.25),
    "cp_5": ConflictPoint("cp_5", -5.25, 1.75), "cp_6": ConflictPoint("cp_6", -1.75, 1.75),
    "cp_7": ConflictPoint("cp_7", 0.0, 1.75), "cp_8": ConflictPoint("cp_8", 1.75, 1.75),
    "cp_9": ConflictPoint("cp_9", 5.25, 1.75), "cp_10": ConflictPoint("cp_10", -1.75, 0.0),
    "cp_11": ConflictPoint("cp_11", 1.75, 0.0), "cp_12": ConflictPoint("cp_12", -5.25, -1.75),
    "cp_13": ConflictPoint("cp_13", -1.75, -1.75), "cp_14": ConflictPoint("cp_14", 0.0, -1.75),
    "cp_15": ConflictPoint("cp_15", 1.75, -1.75), "cp_16": ConflictPoint("cp_16", 5.25, -1.75),
    "cp_17": ConflictPoint("cp_17", -5.25, -5.25), "cp_18": ConflictPoint("cp_18", -1.75, -5.25),
    "cp_19": ConflictPoint("cp_19", 1.75, -5.25), "cp_20": ConflictPoint("cp_20", 5.25, -5.25),
    "cp_21": ConflictPoint("cp_21", -11, 5.25), "cp_22": ConflictPoint("cp_22", 5.25, 11),
    "cp_23": ConflictPoint("cp_23", -5.25, -11), "cp_24": ConflictPoint("cp_24", 11, -5.25)
}

route_conflict_map = {
    "route_N_to_S_0": ["cp_1", "cp_5", "cp_12", "cp_17"],
    "route_N_to_S_1": ["cp_2", "cp_6", "cp_10", "cp_13", "cp_18"],
    "route_S_to_N_0": ["cp_20", "cp_16", "cp_9", "cp_4"],
    "route_S_to_N_1": ["cp_19", "cp_15", "cp_11", "cp_8", "cp_3"],
    "route_E_to_W_0": ["cp_4", "cp_3", "cp_2", "cp_1"],
    "route_E_to_W_1": ["cp_9", "cp_8", "cp_7", "cp_6", "cp_5"],
    "route_W_to_E_0": ["cp_17", "cp_18", "cp_19", "cp_20"],
    "route_W_to_E_1": ["cp_12", "cp_13", "cp_14", "cp_15", "cp_16"],
    "route_N_to_E_1": ["cp_2", "cp_7", "cp_11", "cp_16"],
    "route_S_to_W_1": ["cp_19", "cp_14", "cp_10", "cp_5"],
    "route_E_to_S_1": ["cp_9", "cp_11", "cp_14", "cp_18"],
    "route_W_to_N_1": ["cp_12", "cp_10", "cp_7", "cp_3"],
    "route_N_to_W_0": ["cp_21"], "route_E_to_N_0": ["cp_22"],
    "route_W_to_S_0": ["cp_23"], "route_S_to_E_0": ["cp_24"]
}

try:
    with open('scenario_40.json', 'r', encoding='utf-8') as f:
        scenario_data = json.load(f)
    print(f"✅ 시나리오 파일 로드 성공: 총 {len(scenario_data)}대")
except FileNotFoundError:
    sys.exit("❌ 'scenario_40.json' 파일을 찾을 수 없습니다.")

# =========================================================
# 🔄 메인 루프
# =========================================================
vehicle_original_speeds = {}
initialized_cars = set()
step = 0

while step < 72000:
    traci.simulationStep()
    curr_time = traci.simulation.getTime()

    if step == 0:
        for veh_info in scenario_data:
            v_id = veh_info['veh_id']
            orig_edge = veh_info['origin_edge']
            dest_edge = veh_info['to_edge']
            v_speed = veh_info['speed']
            v_lane = str(veh_info['lane'])

            orig_letter = orig_edge.split('_')[0].upper()
            dest_letter = dest_edge.split('_')[0].upper()
            route_name = f"route_{orig_letter}_to_{dest_letter}"

            traci.vehicle.add(v_id, route_name, departLane=v_lane)
            vehicle_original_speeds[v_id] = v_speed

    active_cars = traci.vehicle.getIDList()

    if step > 0 and traci.simulation.getMinExpectedNumber() == 0:
        print(f"\n🏁 모든 차량 통과 완료! 🏁")
        print(f"⏱️ 총 소요 시간: {curr_time:.1f}초")
        break

    for car in active_cars:
        if car not in initialized_cars:
            try:
                traci.vehicle.setLaneChangeMode(car, 0)
                traci.vehicle.setSpeedMode(car, 0)
                initialized_cars.add(car)
            except:
                continue

        try:
            orig_v = vehicle_original_speeds.get(car, 10.0)
            rid = traci.vehicle.getRouteID(car)
            lidx = traci.vehicle.getLaneIndex(car)
            key = f"{rid}_{lidx}"
            x, y = traci.vehicle.getPosition(car)

            target_cps = route_conflict_map.get(key, [])
            final_v = orig_v

            for cp_id in target_cps:
                if cp_id in conflict_points:
                    cp = conflict_points[cp_id]
                    cp.UpdateCar(car, x, y, curr_time, orig_v)
                    if car not in cp.PassedCars:
                        req_v = cp.GetRequiredSpeed(car, curr_time, orig_v)
                        if req_v < final_v: final_v = req_v

            leader = traci.vehicle.getLeader(car, 15.0)
            if leader:
                l_id, l_dist = leader
                l_speed = traci.vehicle.getSpeed(l_id)
                if l_dist < 5.0:
                    final_v = min(final_v, l_speed * 0.85)
                    if l_dist < 2.0: final_v = 0.1
                elif final_v > l_speed:
                    final_v = min(final_v, l_speed + 1.0)

            traci.vehicle.setSpeed(car, final_v)
        except:
            pass

    for car in list(vehicle_original_speeds.keys()):
        if car not in active_cars and traci.simulation.getTime() > 1.0:
            pass

    # =========================================================
    # 🔍 CP 19 상태 모니터링 로그 (이 부분만 추가되었습니다!)
    # =========================================================
    cp_19 = conflict_points.get("cp_3")
    if cp_19 and cp_19.ApproachCar:
        print(f"\n⏱️ [Time: {curr_time:.1f}s] CP 3 상태")
        for c_id, c_data in cp_19.ApproachCar.items():
            start_t = c_data['start']
            exit_t = c_data['exit']
            try:
                curr_spd = traci.vehicle.getSpeed(c_id)
                prev_spd = c_data.get('log_prev_speed', curr_spd)
                spd_diff = curr_spd - prev_spd

                # 감속/가속 텍스트 생성
                accel_str = ""
                if abs(spd_diff) > 0.05:
                    direction = "가속" if spd_diff > 0 else "감속"
                    accel_str = f" ({direction} {abs(spd_diff):.1f}m/s)"

                print(f"{c_id}: start {start_t:.1f}, exit {exit_t:.1f}, 속도 {curr_spd:.1f}m/s{accel_str}")

                # 다음 스텝 비교를 위해 현재 속도 저장
                c_data['log_prev_speed'] = curr_spd
            except:
                print(f"{c_id}: start {start_t:.1f}, exit {exit_t:.1f}, 속도 알수없음")
        print("-" * 40)
    # =========================================================

    step += 1

traci.close()