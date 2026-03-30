import os
import sys
import random
import traci
import threading
import queue
import math


class ConflictPoint:
    def __init__(self, point_id, x, y):
        self.id = point_id
        self.x, self.y = x, y
        self.latest_exit_time = 0.0
        self.occ_length = 5.0
        self.safety_gap = 1.0

    def clear_ledger(self, current_time):
        self.latest_exit_time = 0.0  # 아무도 없을 때 리셋

    def get_safe_entry_time(self, proposed_entry):
        required_entry = self.latest_exit_time + self.safety_gap
        return max(proposed_entry, required_entry)

    def add_reservation(self, exit_t):
        if exit_t > self.latest_exit_time:
            self.latest_exit_time = exit_t


# =========================================================
# ⚙️ SUMO 초기화
# =========================================================
input_queue = queue.Queue()

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
    "route_N_to_S_0": ["cp_1", "cp_5", "cp_12", "cp_17"], "route_N_to_S_1": ["cp_2", "cp_6", "cp_10", "cp_13", "cp_18"],
    "route_S_to_N_0": ["cp_20", "cp_16", "cp_9", "cp_4"], "route_S_to_N_1": ["cp_19", "cp_15", "cp_11", "cp_8", "cp_3"],
    "route_E_to_W_0": ["cp_4", "cp_3", "cp_2", "cp_1"], "route_E_to_W_1": ["cp_9", "cp_8", "cp_7", "cp_6", "cp_5"],
    "route_W_to_E_0": ["cp_17", "cp_18", "cp_19", "cp_20"],
    "route_W_to_E_1": ["cp_12", "cp_13", "cp_14", "cp_15", "cp_16"],
    "route_N_to_E_1": ["cp_2", "cp_7", "cp_11", "cp_16"], "route_S_to_W_1": ["cp_19", "cp_14", "cp_10", "cp_5"],
    "route_E_to_S_1": ["cp_9", "cp_11", "cp_14", "cp_18"], "route_W_to_N_1": ["cp_12", "cp_10", "cp_7", "cp_3"],
    "route_N_to_W_0": ["cp_21"], "route_E_to_N_0": ["cp_22"],
    "route_W_to_S_0": ["cp_23"], "route_S_to_E_0": ["cp_24"]
}


def get_user_input():
    while True:
        try:
            val = input()
            if val.lower() == 'q': input_queue.put('q'); break
            input_queue.put(float(val))
        except:
            pass


threading.Thread(target=get_user_input, daemon=True).start()


# =========================================================
# 🧮 물리 기반 예측 함수
# =========================================================
def calc_kinematic_time(d, v_curr, v_max, a=2.6):
    if d <= 0: return 0.0
    t_acc = max(0.0, (v_max - v_curr) / a)
    d_acc = (v_curr * t_acc) + (0.5 * a * (t_acc ** 2))

    if d > d_acc:
        d_cruise = d - d_acc
        t_cruise = d_cruise / v_max
        return t_acc + t_cruise
    else:
        return (-v_curr + math.sqrt(max(0, v_curr ** 2 + 2 * a * d))) / a


# =========================================================
# 🔄 메인 루프
# =========================================================
vehicle_original_speeds = {}
car_passed_cps = {}
car_prev_dist = {}
car_last_log_time = {}  # 교차로 감속 로그 도배 방지
car_sumo_log_time = {}  # SUMO 강제 감속 로그 도배 방지
car_commanded_speed = {}  # 파이썬이 이전 스텝에 명령한 속도 기억
car_id_counter = 0
step = 0


def get_car_id_num(car_name):
    try:
        return int(car_name.split('_')[1])
    except:
        return 999999


while step < 72000:
    traci.simulationStep()
    curr_time = traci.simulation.getTime()

    if not input_queue.empty():
        cmd = input_queue.get()
        if cmd == 'q': break
        car_id_counter += 1
        name = f"car_{car_id_counter}"
        chosen_r = random.choice(list(routes.keys()))
        traci.vehicle.add(name, chosen_r, departLane="best")

        # 🚨 [중요] 사용자가 원한 대로 setSpeedMode(1) 유지 (SUMO 물리엔진 개입 허용)
        traci.vehicle.setLaneChangeMode(name, 0)
        traci.vehicle.setSpeedMode(name, 1)

        vehicle_original_speeds[name] = cmd
        car_commanded_speed[name] = cmd  # 초기 명령 속도는 최고속도
        car_passed_cps[name] = set()
        car_prev_dist[name] = {}
        car_last_log_time[name] = -999.0
        car_sumo_log_time[name] = -999.0
        print(f"✅ [투입] {name} | {chosen_r} | 최고속도: {cmd}m/s")

    active_cars = traci.vehicle.getIDList()

    for cp in conflict_points.values():
        cp.clear_ledger(curr_time)

    sorted_cars = sorted(active_cars, key=get_car_id_num)

    for car in sorted_cars:
        try:
            orig_v = vehicle_original_speeds.get(car, 10.0)
            curr_v = max(traci.vehicle.getSpeed(car), 0.0)

            # =================================================================
            # 🚨 [SUMO 개입 감지 로직]
            # 파이썬이 이전 스텝에 명령한 속도보다 실제 속도가 비정상적으로 뚝 떨어졌는지 확인
            # =================================================================
            last_cmd_v = car_commanded_speed.get(car, orig_v)
            if curr_v < last_cmd_v - 0.5:  # 0.5m/s 이상의 오차면 SUMO가 강제로 깎은 것!
                leader = traci.vehicle.getLeader(car, 20.0)
                if leader and (curr_time - car_sumo_log_time.get(car, -999.0) >= 1.0):
                    l_id, l_dist = leader
                    print(f"🛑 [SUMO 자동 감속] {curr_time:.1f}s | {car}")
                    print(f"   ⚠️ 이유: 앞차 {l_id}(거리 {l_dist:.1f}m) 안전거리 확보")
                    print(f"   📉 파이썬 명령: {last_cmd_v:.1f} m/s ➡️ SUMO 실제 강제적용: {curr_v:.1f} m/s")
                    car_sumo_log_time[car] = curr_time
            # =================================================================

            rid = traci.vehicle.getRouteID(car)
            lidx = traci.vehicle.getLaneIndex(car)
            key = f"{rid}_{lidx}"
            x, y = traci.vehicle.getPosition(car)

            target_cps = route_conflict_map.get(key, [])

            if not target_cps and (curr_time - car_last_log_time.get(car, -999.0) >= 2.0):
                print(f"⚠️ [디버그] {car}가 '{key}' 경로를 타서 매핑된 CP가 없습니다!")
                car_last_log_time[car] = curr_time

            final_v = orig_v

            # [A] 교차로 충돌 회피 속도 계산 (파이썬 로직)
            for cp_id in target_cps:
                if cp_id in car_passed_cps.get(car, set()):
                    continue

                cp = conflict_points[cp_id]
                dist = math.dist((x, y), (cp.x, cp.y))

                prev_dist = car_prev_dist[car].get(cp_id, 9999.0)
                if dist > prev_dist and dist > 5.0:
                    car_passed_cps[car].add(cp_id)
                    continue
                car_prev_dist[car][cp_id] = dist

                time_to_reach = calc_kinematic_time(dist, curr_v, final_v)
                t_entry = curr_time + time_to_reach

                safe_entry = cp.get_safe_entry_time(t_entry)

                if safe_entry > t_entry:
                    required_v = dist / (safe_entry - curr_time)
                    temp_v = min(final_v, max(0.0, required_v))

                    if temp_v < final_v and (curr_time - car_last_log_time.get(car, -999.0) >= 1.0):
                        print(f"🔻 [교차로 파이썬 감속] {curr_time:.1f}s | {car} -> {cp_id} 대기 중")
                        print(f"   📉 파이썬 목표속도 계산: {orig_v:.1f} m/s ➡️ {temp_v:.1f} m/s")
                        print(f"   ⏱️ 원래 도착: {t_entry:.1f}s | 강제 지연: {safe_entry:.1f}s")
                        car_last_log_time[car] = curr_time

                    final_v = temp_v

            # [B] 장부 기록 (교차로)
            for cp_id in target_cps:
                if cp_id in car_passed_cps.get(car, set()):
                    continue

                cp = conflict_points[cp_id]
                dist = math.dist((x, y), (cp.x, cp.y))

                time_to_reach = calc_kinematic_time(dist, curr_v, final_v)
                t_entry_confirmed = curr_time + time_to_reach

                pass_speed = max(curr_v, final_v, 0.1)
                t_exit_confirmed = t_entry_confirmed + (cp.occ_length / pass_speed)

                cp.add_reservation(t_exit_confirmed)

            # [C] 최종 속도 적용 및 기록
            # SUMO가 알아서 앞차 추종을 하도록 setSpeedMode(1)로 설정했으므로,
            # 여기서 앞차 추종 계산 로직은 뺐습니다. 오직 교차로 통과 속도만 명령합니다.
            traci.vehicle.setSpeed(car, final_v)
            car_commanded_speed[car] = final_v  # 내가 명령한 값 저장 (다음 스텝 비교용)

        except Exception:
            pass

    # 메모리 정리
    for car in list(vehicle_original_speeds.keys()):
        if car not in active_cars:
            del vehicle_original_speeds[car]
            if car in car_passed_cps: del car_passed_cps[car]
            if car in car_prev_dist: del car_prev_dist[car]
            if car in car_last_log_time: del car_last_log_time[car]
            if car in car_sumo_log_time: del car_sumo_log_time[car]
            if car in car_commanded_speed: del car_commanded_speed[car]

    step += 1

traci.close()