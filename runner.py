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
        self.ApproachCar = {}
        self.PassedCars = set()
        self.last_exit_time = 0
        self.safety_gap = 3.0
        self.occ_length = 5.0

    def UpdateCar(self, car_id, car_x, car_y, current_sim_time, orig_speed):
        if car_id in self.PassedCars: return
        dist = math.dist((self.x, self.y), (car_x, car_y))

        # 1. CP가 완전히 비어있을 때만 장부 기준점을 현재로 동기화
        if not self.ApproachCar and current_sim_time > self.last_exit_time:
            self.last_exit_time = current_sim_time

        # 물리적으로 도달 가능한 최단 시간 계산
        earliest_arrival = current_sim_time + (dist / orig_speed)

        # ---------------------------------------------------------
        # 2. [신규 예약 단계] 차량이 처음 장부에 등록될 때
        # ---------------------------------------------------------
        if car_id not in self.ApproachCar:
            # 처음 예약할 때는 현재 장부의 마지막 시간(last_exit_time)을 따름
            start_time = max(earliest_arrival, self.last_exit_time + self.safety_gap)
            exit_time = start_time + 3.0

            self.last_exit_time = exit_time
            self.ApproachCar[car_id] = {
                'start': start_time,
                'exit': exit_time,
                'dist': dist,
                'prev_dist': dist
            }
            print(f"📌 [신규예약] {car_id} -> {self.id} | {start_time:.1f}s")

        # ---------------------------------------------------------
        # 3. [업데이트 로직] 이미 예약된 차량이 가속을 시도할 때 (사용자 제안 반영)
        # ---------------------------------------------------------
        else:
            # 내 예약 순번 확인 (파이썬 3.7+ 딕셔너리는 순서를 보장함)
            car_list = list(self.ApproachCar.keys())
            try:
                my_idx = car_list.index(car_id)
            except ValueError:
                return

            # 🔥 [자기 참조 방지] 기준 시간(reference_time) 결정
            if my_idx == 0:
                # 내가 1등이면 앞차가 이미 빠진 것이므로 '현재 시간'이 기준이다.
                reference_time = current_sim_time
            else:
                # 내 앞에 차가 있다면, '바로 앞차의 탈출 시간'이 기준이다.
                front_car = car_list[my_idx - 1]
                reference_time = self.ApproachCar[front_car]['exit']

            # 다시 계산한 내가 갈 수 있는 가장 빠른 예약 시간
            possible_start = max(earliest_arrival, reference_time + self.safety_gap)

            # 현재 예약된 시간보다 더 빨리 갈 수 있다면(앞차가 빠졌다면) 업데이트한다.
            if possible_start < self.ApproachCar[car_id]['start']:
                self.ApproachCar[car_id]['start'] = possible_start
                self.ApproachCar[car_id]['exit'] = possible_start + 3.0

                # 내가 줄의 마지막 차라면, 전역 장부 시간(last_exit_time)도 함께 갱신한다.
                if my_idx == len(car_list) - 1:
                    self.last_exit_time = self.ApproachCar[car_id]['exit']

        # 4. [물리 기반 해제 단계]
        if car_id in self.ApproachCar:
            prev_dist = self.ApproachCar[car_id]['prev_dist']

            # --- [Step 1] 가속 트리거 (5m 지점) ---
            # 앞범퍼가 중심을 지나 멀어지기 시작하면(V자 변곡점), 뒷차를 위해 내 예약 시간을 확 줄여줌
            if dist > prev_dist and dist < 5.0:
                # 내 탈출 시간을 현재 시간 근처로 땡겨버림 (0.5초 정도만 남기고)
                # 이렇게 하면 뒷차는 내가 아직 장부에 있어도 'reference_time'이 당겨져서 가속함
                self.ApproachCar[car_id]['exit'] = current_sim_time + 0.5

                # 내가 마지막 차였다면 전체 장부 시간도 동기화
                car_list = list(self.ApproachCar.keys())
                if car_id == car_list[-1]:
                    self.last_exit_time = current_sim_time + 0.5

            # --- [Step 2] 실제 장부 삭제 (8m 지점) ---
            # 뒷범퍼까지 확실히 빠졌을 때 장부에서 완전히 제거 (충돌 방지)
            if dist > prev_dist and dist > 7.0:
                self.PassedCars.add(car_id)
                del self.ApproachCar[car_id]

                # 내가 빠졌으니 장부 전체 리셋 (뒷차가 1등이 됨)
                if not self.ApproachCar:
                    self.last_exit_time = current_sim_time
            else:
                # 거리 데이터 업데이트
                self.ApproachCar[car_id]['prev_dist'] = dist
                self.ApproachCar[car_id]['dist'] = dist

    def GetRequiredSpeed(self, car_id, current_time, orig_speed):
        if car_id not in self.ApproachCar: return orig_speed
        data = self.ApproachCar[car_id]

        time_left = data['start'] - current_time

        # 🔥 [수정] 가속 구간 제어
        if time_left <= 0:
            # 진입 시간(start)은 지났지만 아직 탈출 시간(exit) 전이라면
            # 급가속해서 충돌하지 않도록 원래 속도의 80~90% 정도로만 부드럽게 가속 유지
            if current_time < data['exit']:
                return orig_speed * 0.9
            return orig_speed

        req_v = data['dist'] / time_left
        return min(orig_speed, max(0.1, req_v))


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

# 1. 네가 준 경로 데이터 (그대로 사용)
routes = {
    "route_N_to_S": ["n_in", "s_out"],
    "route_S_to_N": ["s_in", "n_out"],
    "route_E_to_W": ["e_in", "w_out"],
    "route_W_to_E": ["w_in", "e_out"],
    "route_N_to_E": ["n_in", "e_out"],
    "route_S_to_W": ["s_in", "w_out"],
    "route_E_to_S": ["e_in", "s_out"],
    "route_W_to_N": ["w_in", "n_out"],
    "route_N_to_W": ["n_in", "w_out"],
    "route_E_to_N": ["e_in", "n_out"],
    "route_W_to_S": ["w_in", "s_out"],
    "route_S_to_E": ["s_in", "e_out"]
}

for r_id, edges in routes.items():
    try:
        traci.route.add(r_id, edges)
    except:
        pass

# 2. 네가 준 CP 설정 (그대로 사용)
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

# 3. 네가 준 매핑 데이터 (그대로 사용)
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
# 🔄 메인 루프
# =========================================================
vehicle_original_speeds = {}
car_id_counter = 0
step = 0

while step < 72000:
    traci.simulationStep()
    curr_time = traci.simulation.getTime()

    if not input_queue.empty():
        cmd = input_queue.get()
        if cmd == 'q': break
        car_id_counter += 1
        name = f"car_{car_id_counter}"

        # 네가 만든 routes 딕셔너리의 키값 그대로 사용
        chosen_r = random.choice(list(routes.keys()))
        traci.vehicle.add(name, chosen_r, departLane="best")

        # 🔥 요청사항 1: 차선 변경 끄기
        traci.vehicle.setLaneChangeMode(name, 0)

        traci.vehicle.setSpeedMode(name, 0)
        traci.vehicle.setSpeed(name, cmd)
        vehicle_original_speeds[name] = cmd
        print(f"✅ [투입] {name} | {chosen_r}")

    active_cars = traci.vehicle.getIDList()

    for car in active_cars:
        try:
            orig_v = vehicle_original_speeds.get(car, 10.0)
            rid = traci.vehicle.getRouteID(car)
            lidx = traci.vehicle.getLaneIndex(car)
            key = f"{rid}_{lidx}"
            x, y = traci.vehicle.getPosition(car)

            target_cps = route_conflict_map.get(key, [])
            final_v = orig_v

            # 1. 교차로 제어
            for cp_id in target_cps:
                if cp_id in conflict_points:
                    cp = conflict_points[cp_id]
                    cp.UpdateCar(car, x, y, curr_time, orig_v)
                    if car not in cp.PassedCars:
                        req_v = cp.GetRequiredSpeed(car, curr_time, orig_v)
                        if req_v < final_v: final_v = req_v

            # 2. 앞차 추종 (뚫고 가는 현상 방지)
            leader = traci.vehicle.getLeader(car, 20.0)
            if leader:
                l_id, l_dist = leader
                l_speed = traci.vehicle.getSpeed(l_id)
                if l_dist < 6.0:
                    final_v = min(final_v, l_speed * 0.8)
                    if l_dist < 3.0: final_v = 0.1
                elif final_v > l_speed:
                    final_v = min(final_v, l_speed + 1.0)

            traci.vehicle.setSpeed(car, final_v)
        except:
            pass

    for car in list(vehicle_original_speeds.keys()):
        if car not in active_cars: del vehicle_original_speeds[car]
    step += 1

traci.close()