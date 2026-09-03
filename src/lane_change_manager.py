import numpy as np
import math
from copy import deepcopy

class LaneChangeManager:
    def __init__(self):
        self.route_mode = "GLOBAL"
        self.overtake_counter = 0
        self.last_overtake_side = "left"
        self.return_settle_frames = 85
        self.last_closest_idx_orig = 0
        self.cooldown_frames = 0

        # memoria de la maniobra
        self.overtake_start_s = None
        #self.overtake_start_idx = 0
        self.target_obstacle_kind = None
        self.target_obstacle_start_distance = None

        # confirmación de retorno
        self.return_clear_counter = 0
        self.return_clear_need_frames = 10  #cambiar si vuelve pronto

        # respaldo si no puedo confirmar bien el objetivo
        self.min_progress_for_fallback_return_m = 18.0
        self.min_overtake_frames_before_return = 50
        self.max_overtake_frames_force_return = 300

        self.persistence_counter = 0 
        # Memoria específica para evitar volver por glitches o huecos pequeños
        # entre coches de una hilera.
        self.return_lane_blocked_memory = 0
        self.return_lane_blocked_memory_frames = 18 ##si vuelve demasiado pronto cambiar a 25. Si tarda demasiado en volver, bajarlo a 12
        self.return_front_clear_m = 30.0

        # Evita volver antes de superar completamente bicicletas/peatones lentos.
        self.vehicle_return_extra_m = 8.0
        self.vulnerable_return_extra_m = 14.0
        self.vehicle_min_overtake_frames = 50
        self.vulnerable_min_overtake_frames = 90

    def get_closest_point_route(self, vehicle_location, spline_coords_np, last_idx=0, back_window=10, fwd_window=60, max_local_dist_m=8.0):
        '''Busca el waypoint más cercano'''
        if len(spline_coords_np) == 0:  # seguridad ante ruta vacía
            return 0
        
        veh_pos = np.array([vehicle_location.x, vehicle_location.y], dtype=float)
        i0 = max(0, last_idx - back_window)
        i1 = min(len(spline_coords_np), last_idx + fwd_window)

        if i1 <= i0:    # si la ventana sale mal amplia búsqueda a toda la ruta
            i0 = 0
            i1 = len(spline_coords_np)

        subset = spline_coords_np[i0:i1]
        distances_sq = np.sum((subset - veh_pos) ** 2, axis=1)
        local_best_rel = int(np.argmin(distances_sq))   # busca el índice de menor valor (pto más cercano)
        local_best_idx = i0 + local_best_rel
        local_best_dist = math.sqrt(float(distances_sq[local_best_rel]))

        if local_best_dist > max_local_dist_m:
            distances_sq_full = np.sum((spline_coords_np - veh_pos) ** 2, axis=1)
            return int(np.argmin(distances_sq_full))
        
        return local_best_idx
    
    def _get_xy(self, wp): ### Revisar en qué formato vienen los waypoints para no usar todos ellos. Prueba, borrar lo comentado si funciona
        if isinstance(wp, dict): return float(wp.get('x', 0.0)), float(wp.get('y', 0.0))
        if hasattr(wp, 'transform'): return float(wp.transform.location.x), float(wp.transform.location.y)
        if hasattr(wp, 'location'): return float(wp.location.x), float(wp.location.y)
        return float(getattr(wp, 'x', 0.0)), float(getattr(wp, 'y', 0.0))
        #return float(wp.x), float(wp.y) ###borrar si descomento lo demás
    
    def _compute_cumulative_distance(self, coords):
        '''Calcula cuánta distancia lleva recorrida desde el inicio de la ruta'''
        cumulative_distance = np.zeros(len(coords), dtype=float)
        for i in range(1, len(coords)):
            cumulative_distance[i] = cumulative_distance[i-1] + np.linalg.norm(coords[i] - coords[i-1])
        return cumulative_distance
    
    def _target_has_been_passed(self, progress_m, obs, last_overtake_side):
        """
        Decide si ya se ha superado el obstáculo inicial.

        Idea:
        - Para un coche aislado, no espera a irse lejísimos: basta con avanzar
        la distancia a la que estaba el coche al iniciar + un margen.
        - Para peatones/bicicletas se exige más margen para no volver antes de tiempo.
        - Para una hilera, si todavía se detecta un vehículo delante en el carril
        al que queremos volver, se bloquea el retorno.
        """
        target_kind = str(self.target_obstacle_kind).upper()
        is_vulnerable = target_kind in ["PED_IN_LANE", "PEDESTRIAN", "BICYCLE"]

        # No permitir vuelta instantánea justo después de empezar a salir.
        min_frames = self.vulnerable_min_overtake_frames if is_vulnerable else self.vehicle_min_overtake_frames
        if self.overtake_counter < min_frames:
            return False

        # Progreso mínimo respecto al punto donde empezó el adelantamiento.
        # Para vulnerables damos más margen porque se mueven y la distancia detectada
        # suele ser al primer píxel, no a todo el actor.
        extra_m = self.vulnerable_return_extra_m if is_vulnerable else self.vehicle_return_extra_m

        if self.target_obstacle_start_distance is not None:
            required_progress = float(self.target_obstacle_start_distance) + extra_m
        else:
            required_progress = 18.0 if is_vulnerable else 12.0

        if progress_m < required_progress:
            return False

        # Distancia frontal SOLO en el carril al que queremos volver.
        # No usamos d_vehicle_ahead_m porque puede pertenecer al carril actual
        # de adelantamiento y no debe impedir el retorno.
        if last_overtake_side == "left":
            # Salí a la izquierda, quiero volver a la derecha/original.
            d_front_return = obs.get('d_right_front_m')
        else:
            # Salí a la derecha, quiero volver a la izquierda/original.
            d_front_return = obs.get('d_left_front_m')

        # Si vemos un vehículo cerca en el carril de retorno, bloqueamos.
        if d_front_return is not None and float(d_front_return) < self.return_front_clear_m:
            self.return_lane_blocked_memory = self.return_lane_blocked_memory_frames
            return False

        # Si lo hemos visto hace poco, seguimos bloqueando.
        # Esto evita volver por glitches de percepción o por huecos pequeños entre coches.
        if self.return_lane_blocked_memory > 0:
            self.return_lane_blocked_memory -= 1
            return False

        return True

    def _build_smooth_route(self, global_route, idx_ego, offset_inicial, offset_final, length_m, start_offset=1.0):
        """Genera una nueva ruta para salir al carril de adelantamiento o retornar al original
        Genera la ruta de transición de una vez"""
        if global_route is None or len(global_route) == 0: return None

        new_route = deepcopy(global_route)
        coords = np.array([self._get_xy(wp) for wp in new_route])
        cumulative_distance = self._compute_cumulative_distance(coords)

        # Empezamos el giro suave 3 metros por delante del morro del coche
        s_start = cumulative_distance[idx_ego] + start_offset #3.0 puesto a 1.0 el coche empieza a girar 1.0 m por delante del punto actual del coche
        s_end = s_start + length_m
        prev_yaw_rad = 0.0

        for idx, wp in enumerate(new_route):    #modifica waypoint por waypoint
            if cumulative_distance[idx] <= s_start:
                factor = 0.0   # factor: indica cuánto de la transición lateral se ha aplicado ya en ese punto
            elif cumulative_distance[idx] >= s_end:
                factor = 1.0
            else:
                # transición cosenoidal para un giro perfecto sin latigazos
                progress = (cumulative_distance[idx] - s_start) / length_m
                factor = (1.0 - math.cos(progress * math.pi)) / 2.0 # podría ser factor=progress pero sería una transición menos suave

            offset_lateral = offset_inicial + factor * (offset_final - offset_inicial)

            if abs(offset_lateral) < 0.01: continue # para desplazamiento lateral muy pequeño, no toca ese waypoint y pasa al siguiente idx

            x_current, y_current = coords[idx]

            if idx < len(coords) - 1:   # calcula la orientación de la ruta con respecto al siguiente punto en radianes
                dx = coords[idx+1][0] - x_current   #del siguiente punto [idx+1] coje la componente 0 que es la x
                dy = coords[idx+1][1] - y_current   #coge la componente 1 que es la y
                if abs(dx) > 1e-4 or abs(dy) > 1e-4:
                    yaw = math.atan2(dy, dx)
                    prev_yaw_rad = yaw
                else: yaw = prev_yaw_rad
            else: yaw = prev_yaw_rad

            # Vector normal apuntando a la izquierda 
            nx = -math.sin(yaw)
            ny = math.cos(yaw)

            x_new = x_current + offset_lateral * nx
            y_new = y_current + offset_lateral * ny
            
            if isinstance(wp, dict): wp['x'] = x_new; wp['y'] = y_new   ### revisar cuál es la opción buena y dejar solo esa
            elif hasattr(wp, 'transform'): wp.transform.location.x = x_new; wp.transform.location.y = y_new
            elif hasattr(wp, 'location'): wp.location.x = x_new; wp.location.y = y_new
            else:
                if hasattr(wp,'x'): wp.x = x_new
                if hasattr(wp,'y'): wp.y = y_new

        return new_route

    def update(self, lane_option, global_route, ego_location, obs, v_now, is_urgent=False):
        if global_route is None: return None, None
        if obs is None: obs = {}

        # 1. Normalización de comandos
        lane_change_command = str(lane_option).strip().lower()
        if lane_change_command in ["none", "0", "false", "keep"]: lane_change_command = "keep"
        elif lane_change_command in ["left", "-1"]: lane_change_command = "left"
        elif lane_change_command in ["right", "1"]: lane_change_command = "right"
        else: lane_change_command = "keep"

        output_route = None 
        output_mode = self.route_mode 

        coords_orig = np.array([self._get_xy(wp) for wp in global_route])
        idx_ego = self.get_closest_point_route(ego_location, coords_orig, last_idx=self.last_closest_idx_orig)
        self.last_closest_idx_orig = idx_ego
        s_ego = self._compute_cumulative_distance(coords_orig)[idx_ego]

        # Cálculo robusto del desplazamiento lateral (usando vectores normales)
        dx = coords_orig[min(idx_ego+1, len(coords_orig)-1)][0] - coords_orig[idx_ego][0]
        dy = coords_orig[min(idx_ego+1, len(coords_orig)-1)][1] - coords_orig[idx_ego][1]
        yaw = math.atan2(dy, dx)
        # Offset lateral: proyección del vector ego-ruta sobre la normal de la ruta
        normal_x, normal_y = -math.sin(yaw), math.cos(yaw)
        route_offset_distance = (ego_location.x - coords_orig[idx_ego][0]) * normal_x + \
                                (ego_location.y - coords_orig[idx_ego][1]) * normal_y

        if self.route_mode == "GLOBAL":
            if self.cooldown_frames > 0: self.cooldown_frames -= 1
            
            target_kind = obs.get("target_obstacle_kind")
            if lane_change_command in ["left", "right"] and target_kind in ["VEHICLE", "PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE"]:
                self.last_overtake_side = lane_change_command
                self.overtake_start_s = s_ego
                self.target_obstacle_kind = target_kind
                self.target_obstacle_start_distance = obs.get('target_obstacle_distance_m')

                longitud_dinamica = 14.0 if is_urgent else 25.0 #Rampa
                offset_final = 3.5 if self.last_overtake_side == "left" else -3.5
                
                output_route = self._build_smooth_route(global_route, idx_ego, 0.0, offset_final, longitud_dinamica)
                self.route_mode = "OVERTAKE"
                self.overtake_counter = 0

        elif self.route_mode == "OVERTAKE":
            self.overtake_counter += 1
            progress_m = max(0.0, s_ego - self.overtake_start_s)
            
            # El carril contrario es donde el offset tiene el mismo signo que nuestra maniobra
            # Si last_overtake_side es 'left', el offset debe ser > 2.0m para estar fuera
            estoy_en_carril_contrario = (self.last_overtake_side == "left" and route_offset_distance > 2.0) or \
                                        (self.last_overtake_side == "right" and route_offset_distance < -2.0)

            if self._target_has_been_passed(progress_m, obs, self.last_overtake_side) and estoy_en_carril_contrario:
                self.return_clear_counter += 1
            else:
                self.return_clear_counter = 0

            if self.return_clear_counter >= self.return_clear_need_frames:
                offset_actual = route_offset_distance
                output_route = self._build_smooth_route(global_route, idx_ego, offset_actual, 0.0, 20.0)
                self.route_mode = "RETURN"
                self.overtake_counter = 0

        elif self.route_mode == "RETURN":
            if abs(route_offset_distance) < 0.5:
                self.route_mode = "GLOBAL"
                self.cooldown_frames = 60
        
        return output_route, self.route_mode
