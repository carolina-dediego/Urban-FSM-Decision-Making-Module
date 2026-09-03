import numpy as np

class BasicPlannerConfig:   # Agrupación de parámetros
    def __init__(self):
        self.grid_h = 200
        self.grid_w = 200
        self.corridor_width_px = 22   #ancho donde buscar obstáculos
        self.lookahead_px = 90    #90 ve 45m por delante
        self.pixels_per_meter = 2.0   # 2 px/m

        # dinámica
        self.dt_s = 0.05 # tiempo entre decisiones 0.05s (20Hz)
        self.accel_limit_mps2 = 4.0#acceleración máxima permitida
        self.decel_limit_mps2 =4.0

        # frenada de emergencia para actor tipo pedestrian muy cercano en carril propio
        self.emergency_ped_distance_m = 3.0
        self.emergency_decel_mps2 = 14.0
        self.emergency_direct_stop_m = 2.2

        # suavizado
        self.ema_alpha = 0.2
        
        # follow
        self.time_gap_follow = 1.5 # tiempo de separación follow en segundos (dist=min+Tgap*v)
        self.min_distance_follow = 1.2 #minimo de m en follow
        self.follow_start_m = 50.0
        self.follow_accel_limit_mps2 = 2.2 #follow más fuerte en aceleración

        self.coasting_start_m = 60.0
        #stop
        self.stop_distance_m = 4.0  #distancia de stop más grande por si necesita revasar por estar bloqueado
        self.traffic_light_stop_m = 12.0 
        self.tl_forget_after_no_red_s = 15.0

        self.obstacle_front_bool_fallback_m = 4.0
        self.obstacle_front_min_latch_m = 0.5
        self.obstacle_front_clear_after_lc_s = 3.0
        self.obstacle_front_force_decel_mps2 = 8.0

        # intersecciones
        self.intersection_check_m = 22.0          # solo vigila cruces si estoy cerca de la intersección
        self.intersection_turn_yaw_deg = 25.0     # considera giro si la ruta cambia más de 25 grados
        self.cross_traffic_max_m = 20.0           # vehículo cruzando a menos de esta distancia
        self.cross_traffic_memory_frames = 3      # memoria antiparpadeo
        self.cross_traffic_stop_m = 6.0           # margen de parada para conflicto lateral
        

        
        # zona de conflicto en intersecciones
        self.cross_conflict_before_junction_m = 5.0   # zona antes del punto de cruce
        self.cross_conflict_after_junction_m = 12.0   # zona dentro/después del cruce
        self.cross_conflict_half_width_m = 14.0       # ancho base alrededor de la trayectoria
        self.cross_conflict_side_extra_m = 7.0        # extra hacia el lado del giro

        # curva
        self.a_lat_max_mps2 = 3.5
        self.curve_min_v_mps = 4.5
        self.curve_filter_alpha = 0.6

        # lane change
        self.lc_front_min_m = 10.0
        self.lc_rear_min_m = 12.0
        self.lc_time_gap_s = 1.2
        self.lc_trigger_m = 20.0  # distancia a la q está el obstáculo para plantear cambio
        self.lc_min_start_m = 2.0

        self.block_lc_in_curve = True
        self.lc_curve_block_ratio = 0.80
        self.lc_curve_allow_if_lead_below_mps = 1.0

        self.lc_cooldown_s = 2.0
        self.lc_slow_v_mps = 2.0        # tráfico lento si el lead va por debajo de esto (m/s)
        self.lc_slow_ratio = 0.55       # o si lead < 55% de v_max_used
        
        self.lc_follow_ticks = 40

        self.lc_max_overtake_lead_v_mps = 6.0      # si el lead va más rápido que esto, no adelantar
        self.lc_min_speed_advantage_mps = 4      # margen mínimo entre mi v_max_used y la del lead para que compense adelantar
        self.max_stopped_time_s = 15.0            # segundos
        
        # velocidad durante cambio de carril normal
        self.lc_normal_speed_duration_s = 5.0
        self.lc_normal_min_speed_ratio = 0.70
        self.lc_normal_keep_speed_ratio = 0.95

        self.lc_slow_ticks = 40 #trafico lento durante 1 segundo

        # adelantamiento inmediato solo si el obstáculo delantero está prácticamente parado
        self.lc_static_lead_v_mps = 0.8
        self.lc_static_trigger_m = 26.0

        # peatones en cruces/curvas: búsqueda más ancha en el carril propio
        self.ped_wide_check_m = 24.0

        # salida algo más decidida justo después de liberar tráfico cruzado
        self.cross_clear_boost_s = 3.0
        self.cross_clear_start_v_mps = 4.0
        self.cross_clear_accel_mps2 = 7.0

        # nuevos parámetros para STOP e intersecciones
        self.stop_sign_stop_m = 3.0 
        self.stop_sign_wait_time_s = 1.0       # Tiempo de parada obligatorio
        self.stop_sign_clear_cooldown_s = 20.0  # Tiempo para ignorar el STOP tras paro
        
        

class BasicEnvironmentObserver: # Devuelve distancias
    # IDs de bev_segmentation 
    CAR_SEGMENT = 0 
    PEDESTRIAN_SEGMENT = 1 

    # IDs de bev_lane 
    OUT_OF_LANE = -1
    CURRENT_FRONT_LANE = 0  
    LEFT_FRONT_LANE = 1 
    RIGHT_FRONT_LANE = 2
    INTERSECTING_LANE = 3
    OPPOSITE_LANE = 4

    def __init__(self, cfg=None):
        self.cfg = cfg if cfg is not None else BasicPlannerConfig()

    def _px_to_m(self, px):
        return float(px) / max(float(self.cfg.pixels_per_meter), 1e-6)
    
    def _distance_in_front_m(self, bev_segmentation_cls, bev_lane_info, object_types, lane_types):
        """ Busca el primer píxel delante del coche dentro de una roi.
        Si bev_lane_info existe, filtra por lane_types."""
        cfg = self.cfg
        row_front = (cfg.grid_h // 2) - 10
        column_center = cfg.grid_w // 2

        half_corridor_width = cfg.corridor_width_px // 2
        column_start = max(0, column_center - half_corridor_width)
        column_end = min(cfg.grid_w, column_center + half_corridor_width + 1)

        row_start = row_front - 1
        row_end = max(-1, row_front - cfg.lookahead_px - 1)

        for row in range(row_start, row_end, -1):
            roi = bev_segmentation_cls[row, column_start:column_end]

            if bev_lane_info is None:
                if np.any(np.isin(roi, object_types)):
                    return self._px_to_m(row_front - row)
            else:
                valid_object_mask = np.isin(roi, object_types) & np.isin(
                    bev_lane_info[row, column_start:column_end], lane_types
                )
                if np.any(valid_object_mask):
                    return self._px_to_m(row_front - row)

        return None

    def _distance_in_front_wide_lane_m(self, bev_segmentation_cls, bev_lane_info, object_types, lane_types):
        """
        Busca delante, pero sin limitarse al corredor central.
        Se usa solo para peatones en curvas/intersecciones, donde la trayectoria
        gira y el peatón puede estar en CURRENT_FRONT_LANE pero fuera del corredor recto.
        """
        if bev_lane_info is None:
            return None

        cfg = self.cfg
        row_front = (cfg.grid_h // 2) - 10
        row_start = row_front - 1
        row_end = max(-1, row_front - cfg.lookahead_px - 1)

        for row in range(row_start, row_end, -1):
            seg_roi = bev_segmentation_cls[row, :]
            lane_roi = bev_lane_info[row, :]

            mask = np.isin(seg_roi, object_types) & np.isin(lane_roi, lane_types)

            if np.any(mask):
                d_m = self._px_to_m(row_front - row)
                if d_m <= self.cfg.ped_wide_check_m:
                    return d_m
                return None

        return None

    def _distance_behind_m(self, bev_segmentation_cls, bev_lane_info, object_types, lane_types):
        """ Busca el primer píxel detrás del coche dentro de una roi.
        Usado para cambiar de carril.
        """
        cfg = self.cfg
        row_front = (cfg.grid_h // 2) - 10
        column_center = cfg.grid_w // 2

        half_corridor_width = cfg.corridor_width_px // 2
        column_start = max(0, column_center - half_corridor_width)
        column_end = min(cfg.grid_w, column_center + half_corridor_width + 1)

        row_start = row_front + 1
        row_end = min(cfg.grid_h, row_front + cfg.lookahead_px + 1)

        for row in range(row_start, row_end):  # detrás: cerca -> lejos
            roi = bev_segmentation_cls[row, column_start:column_end]

            if bev_lane_info is None:
                if np.any(np.isin(roi, object_types)):
                    return self._px_to_m(row - row_front)
            else:
                valid_object_mask = np.isin(roi, object_types) & np.isin(
                    bev_lane_info[row, column_start:column_end], lane_types
                )
                if np.any(valid_object_mask):
                    return self._px_to_m(row - row_front)

        return None
    
    def _distance_intersecting_vehicle_m(
        self,
        bev_segmentation_cls,
        bev_lane_info,
        distance_to_junction=None,
        turn_direction="straight"
    ):
        """
        Detecta tráfico conflictivo en una intersección.
        """

        if turn_direction not in ("left", "right"):
            return None

        if distance_to_junction is None:
            return None

        cfg = self.cfg
        ppm = float(cfg.pixels_per_meter)
        row_front = (cfg.grid_h // 2) - 10
        col_center = cfg.grid_w // 2

        # Activar cerca y dentro de la intersección.
        if distance_to_junction > cfg.intersection_check_m or distance_to_junction < -10.0:
            return None

        # Fila aproximada del punto de intersección en la BEV.
        row_junction = int(round(row_front - float(distance_to_junction) * ppm))

        before_px = int(round(cfg.cross_conflict_before_junction_m * ppm))
        after_px = int(round(cfg.cross_conflict_after_junction_m * ppm))

        row_min = max(0, row_junction - after_px)
        row_max = min(cfg.grid_h - 1, row_junction + before_px)

        if row_max < row_min:
            return None

        # Zona lateral amplia alrededor del punto de cruce.
        half_w = int(round(cfg.cross_conflict_half_width_m * ppm))
        side_extra = int(round(cfg.cross_conflict_side_extra_m * ppm))
        half_w_total = half_w + side_extra

        col_start = max(0, col_center - half_w_total)
        col_end = min(cfg.grid_w, col_center + half_w_total + 1)

        cols_abs = np.arange(col_start, col_end)
        ego_corridor_half = max(2, cfg.corridor_width_px // 2)
        lateral_mask = np.abs(cols_abs - col_center) > ego_corridor_half

        # Incluimos coches y actores vulnerables para cubrir CrossingBicycleFlow,
        # pedestrian crossing dentro del cruce, motos/bicis etiquetadas como pedestrian, etc.
        conflict_object_types = [self.CAR_SEGMENT]
        conflict_lane_types = [
            self.INTERSECTING_LANE,
            self.LEFT_FRONT_LANE,
            self.RIGHT_FRONT_LANE,
            self.OPPOSITE_LANE,
            self.OUT_OF_LANE,
        ]

        best_d = None

        for row in range(row_max, row_min - 1, -1):
            seg_roi = bev_segmentation_cls[row, col_start:col_end]
            obj_mask = np.isin(seg_roi, conflict_object_types)

            if not np.any(obj_mask):
                continue

            if bev_lane_info is not None:
                lane_roi = bev_lane_info[row, col_start:col_end]
                lane_conflict_mask = np.isin(lane_roi, conflict_lane_types)

                # Aceptamos el actor si está en un carril conflictivo o si está
                # claramente lateral respecto al corredor frontal propio.
                conflict_mask = obj_mask & (lane_conflict_mask | lateral_mask)
            else:
                # Sin información de carril, mejor conservador en la zona de cruce.
                conflict_mask = obj_mask

            if not np.any(conflict_mask):
                continue

            d_m = max(0.0, self._px_to_m(row_front - row))

            if best_d is None or d_m < best_d:
                best_d = d_m

        return best_d

    def _traffic_light_distance_m(self, traffic_light_info, distance_to_junction):
        
        if traffic_light_info is None:
            return None

        status_raw = traffic_light_info.get("traffic_light_status", "")
        status = str(status_raw).lower().strip()

        if "green" in status:
            return -1.0
        
        if distance_to_junction is None:
            return None
        
        if distance_to_junction <= 0:
            return None
        
        if ("red" in status) or ("yellow" in status):
            return float(distance_to_junction)

        return None

    def observe(self, bev_segmentation_cls, bev_lane_info, traffic_light_info, distance_to_junction, turn_direction="straight",
                closest_stop_sign=None, obstacle_in_front=None):
        lane_change_obstacles = [self.CAR_SEGMENT, self.PEDESTRIAN_SEGMENT]

        d_vehicle_ahead = self._distance_in_front_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=[self.CAR_SEGMENT],
            lane_types=[self.CURRENT_FRONT_LANE]
        )

        d_ped_in_lane = self._distance_in_front_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=[self.PEDESTRIAN_SEGMENT],
            lane_types=[self.CURRENT_FRONT_LANE]
        )

        near_junction = (
            distance_to_junction is not None and
            -6.0 <= float(distance_to_junction) <= self.cfg.intersection_check_m
        )

        if d_ped_in_lane is None and (turn_direction in ("left", "right") or near_junction):
            d_ped_in_lane = self._distance_in_front_wide_lane_m(
                bev_segmentation_cls, bev_lane_info,
                object_types=[self.PEDESTRIAN_SEGMENT],
                lane_types=[self.CURRENT_FRONT_LANE]
            )

        left_lane_types = [self.LEFT_FRONT_LANE, self.OPPOSITE_LANE]
        
        # Distancias para cambio de carril (delante)
        d_left_front = self._distance_in_front_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=lane_change_obstacles,
            lane_types=left_lane_types  ##si, los nombres están al revés, tengo q cambiarlo
        )

        d_right_front = self._distance_in_front_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=lane_change_obstacles,
            lane_types=[self.RIGHT_FRONT_LANE]  ##si, los nombres están al revés, tengo q cambiarlo
        )

        d_left_rear = self._distance_behind_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=lane_change_obstacles,
            lane_types=left_lane_types ##si, los nombres están al revés, tengo q cambiarlo
        )

        d_right_rear = self._distance_behind_m(
            bev_segmentation_cls, bev_lane_info,
            object_types=lane_change_obstacles,
            lane_types=[self.RIGHT_FRONT_LANE]  ##si, los nombres están al revés, tengo q cambiarlo
        )

        d_traffic_light = self._traffic_light_distance_m(traffic_light_info, distance_to_junction)

        d_intersecting_vehicle = self._distance_intersecting_vehicle_m(
            bev_segmentation_cls,
            bev_lane_info,
            distance_to_junction=distance_to_junction,
            turn_direction=turn_direction
        )

        left_normal_exists = False
        left_opposite_exists = False
        right_exists = False

        if bev_lane_info is not None:
            # Separa la existencia de los carriles en dos variables distintas
            left_normal_exists = np.any(bev_lane_info == self.LEFT_FRONT_LANE)
            left_opposite_exists = np.any(bev_lane_info == self.OPPOSITE_LANE)
            right_exists = np.any(bev_lane_info == self.RIGHT_FRONT_LANE)
            
        d_stop_sign = None
        if closest_stop_sign is not None:
            # Extraemos la distancia dependiendo de si es diccionario, objeto o float
            if isinstance(closest_stop_sign, dict):
                d_stop_sign = closest_stop_sign.get('distance')
            elif hasattr(closest_stop_sign, 'distance'):
                d_stop_sign = closest_stop_sign.distance
            elif isinstance(closest_stop_sign, (float, int)):
                d_stop_sign = float(closest_stop_sign)

        d_obstacle_front = None
        obstacle_front_flag = False

        if obstacle_in_front is not None:

            if isinstance(obstacle_in_front, dict):
                for key in ("distance", "distance_m", "dist", "dist_m", "d"):
                    if key in obstacle_in_front and obstacle_in_front[key] is not None:
                        try:
                            d_obstacle_front = float(obstacle_in_front[key])
                            break
                        except (TypeError, ValueError):
                            d_obstacle_front = None

                obstacle_front_flag = bool(
                    obstacle_in_front.get("flag", False) or
                    obstacle_in_front.get("obstacle_in_front", False) or
                    obstacle_in_front.get("is_obstacle", False)
                )

            elif hasattr(obstacle_in_front, 'distance'):
                try:
                    d_obstacle_front = float(obstacle_in_front.distance)
                except (TypeError, ValueError):
                    d_obstacle_front = None

            elif isinstance(obstacle_in_front, bool):
                # Caso valla: solo llega True/False.
                obstacle_front_flag = bool(obstacle_in_front)

            elif isinstance(obstacle_in_front, (float, int)):
                d_obstacle_front = float(obstacle_in_front)

        if d_obstacle_front is None and obstacle_front_flag:
            d_obstacle_front = float(self.cfg.obstacle_front_bool_fallback_m)

        obstacle_front_detected = (
            obstacle_front_flag or
            (d_obstacle_front is not None and d_obstacle_front > 0.0)
)

        obs = {
            "d_vehicle_ahead_m": d_vehicle_ahead,
            "d_pedestrian_in_lane_m": d_ped_in_lane,
            "d_traffic_light_m": d_traffic_light,
            "d_intersecting_vehicle_m": d_intersecting_vehicle,
            "d_left_front_m": d_left_front,
            "d_right_front_m": d_right_front,
            "d_left_rear_m": d_left_rear,
            "d_right_rear_m": d_right_rear,
            "left_normal_exists": left_normal_exists,       
            "left_opposite_exists": left_opposite_exists,   
            "right_lane_exists": right_exists,
            "d_stop_sign_m": d_stop_sign,
            "d_obstacle_front_m": d_obstacle_front,
            "obstacle_front_detected": obstacle_front_detected,
        } 
        return obs

class  BasicPlanner:
    STATE_CRUISE = "CRUISE"         # Sin obstáculos, velocidad máxima
    STATE_COASTING = "COASTING"     # Obstáculo lejos
    STATE_FOLLOW = "FOLLOW"         # Seguimiento a distancia
    STATE_BRAKING = "BRAKING"       # Obstáculo cerca, frenada progresiva
    STATE_STOPPED = "STOPPED"       # Parada

    def __init__(self, v_max=10.0, cfg=None, observer=None):
        self.cfg = cfg if cfg is not None else BasicPlannerConfig()
        self.v_max = float(v_max)
        self.observer = observer if observer is not None else BasicEnvironmentObserver(self.cfg)
        
        self.current_state = self.STATE_CRUISE
        self.decision_speed = 0.0
        ##self.state = {"v_target_mps": 0.0, "last_observations": None}

        self.v_curve_mem = 999.0 #empieza asumiendo que es una recta

        #memoria semáforo
        self.last_tl_dist = None  # Última distancia válida
        self.tl_loss_counter = 0  # Cuántos frames llevamos sin verlo
        self.TL_MEMORY_LIMIT = 30 # Recordar durante 30 frames (aprox 1.5 segundo)
        self.tl_soft_approach = False
        self.tl_no_red_timer_s = 0.0

        # memoria coches y pedestrian (antiparpadeo)
        self.last_obs_d = None
        self.last_obs_kind = None
        self.obs_loss_counter = 0
        self.OBS_MEMORY_LIMIT = 10  # Recordarlo durante 10 frames (0.5 segundos) si desaparece

        #follow
        self.prev_lead_d = None     # distancia anterior al coche delante 
        self.prev_lead_kind = None
        self.v_lead_est = 0.85   ###0.0      # estimación de velocidad del coche delante (m/s) 
        self.v_lead_alpha = 0.6        # suavizado (0.6–0.85) 

        self.yaw_rate_f = 0.0
        
        # lane change decision
        self.lane_cmd = 0  # -1 left, 0 keep, +1 right
        self.lc_cooldown_t = 0.0
        self.lead_slow_counter = 0

        self.maneuver_time_counter = 0.0

        self.follow_ticks_counter = 0
        self.stopped_frustration_timer = 0.0 # Cronómetro para invadir sentido contrario

        self.lane_change_urgent = False

        # Memoria específica para vallas/obstáculos frontales detectados con obstacle_in_front.
        # Sirve para que no se olvide la valla si la percepción parpadea.
        self.front_obstacle_latched = False
        self.front_obstacle_latch_d = None
        self.front_obstacle_lc_timer_s = 0.0

        self.emergency_brake_active = False

        # salida suave cuando el cambio de carril se inicia por frustración
        # Solo afecta a los primeros segundos de la maniobra urgente, no al comportamiento normal.
        self.frustrated_escape_timer_s = 0.0
        self.frustrated_escape_duration_s = 3.5
        self.frustrated_escape_v_mps = 3.0
        self.frustrated_escape_accel_mps2 = 1.2

        # ventana de mantenimiento de velocidad en cambio de carril normal
        self.normal_lc_speed_timer_s = 0.0

        # memoria de tráfico cruzado en intersección
        self.cross_traffic_memory = 0
        self.last_cross_traffic_d = None

        self.cross_was_detected = False
        self.cross_clear_boost_timer_s = 0.0
        self.stopped_for_cross_turn = False

        # Memoria específica para el STOP interno
        self.stop_timer_s = 0.0
        self.stop_cleared = False
        self.stop_cooldown_s = 0.0

        self.obs = {}


    def _lane_cmd_to_option(self, cmd: int):
        if cmd == -1:
            return "left"
        if cmd == 1:
            return "right"
        return "keep" 
    
    def _clear_front_obstacle_latch(self):
        self.front_obstacle_latched = False
        self.front_obstacle_latch_d = None
        self.front_obstacle_lc_timer_s = 0.0

    def _extract_distance_from_stop_item(self, item, ego_location=None):
        """Devuelve distancia a un STOP desde distintos formatos posibles."""
        if item is None:
            return None

        if isinstance(item, bool):
            # Un booleano indica presencia, pero no sirve para calcular frenada.
            return None

        if isinstance(item, (int, float)):
            d = float(item)
            return d if d > 0.0 else None

        def xy_from_any(obj):
            """Extrae (x, y) de Location, Transform, Actor CARLA o dict."""
            if obj is None:
                return None

            try:
                if isinstance(obj, dict):
                    if "x" in obj and "y" in obj:
                        return float(obj["x"]), float(obj["y"])
                    obj = obj.get("location") or obj.get("transform")
                    if obj is None:
                        return None

                if hasattr(obj, "get_location"):
                    obj = obj.get_location()
                if hasattr(obj, "transform"):
                    obj = obj.transform
                if hasattr(obj, "location"):
                    obj = obj.location

                return float(getattr(obj, "x")), float(getattr(obj, "y"))
            except Exception:
                return None

        distance_keys = (
            "distance", "distance_m", "dist", "dist_m", "d",
            "distance_to_stop", "distance_to_stop_sign", "distance_to_junction"
        )

        if isinstance(item, dict):
            for key in distance_keys:
                if key in item and item[key] is not None:
                    try:
                        d = float(item[key])
                        return d if d > 0.0 else None
                    except (TypeError, ValueError):
                        pass

            # Algunos wrappers guardan la señal dentro de otra clave.
            for key in ("stop_sign", "sign", "actor", "traffic_sign"):
                if key in item:
                    d = self._extract_distance_from_stop_item(item[key], ego_location)
                    if d is not None:
                        return d
        else:
            for key in distance_keys:
                if hasattr(item, key):
                    try:
                        d = float(getattr(item, key))
                        return d if d > 0.0 else None
                    except (TypeError, ValueError):
                        pass

        # Si no viene distancia pero sí localización, la calculamos respecto al ego.
        ego_xy = xy_from_any(ego_location)
        stop_xy = xy_from_any(item)
        if ego_xy is not None and stop_xy is not None:
            ex, ey = ego_xy
            sx, sy = stop_xy
            return float(np.hypot(sx - ex, sy - ey))

        return None

    def _resolve_stop_sign_distance(self, closest_stop_sign=None, traffic_signs_stop=None, ego_location=None):
        """
        Unifica closest_stop_sign y traffic_signs_stop en una sola distancia.
        Toma la señal de STOP positiva más cercana.
        """
        distances = []

        def feed(source):
            if source is None:
                return

            if isinstance(source, dict):
                # Si el dict contiene una lista de señales, la recorremos.
                for key in ("traffic_signs_stop", "stop_signs", "stops", "actors", "items"):
                    value = source.get(key)
                    if isinstance(value, (list, tuple, set)):
                        for sub_item in value:
                            feed(sub_item)

                d = self._extract_distance_from_stop_item(source, ego_location)
                if d is not None:
                    distances.append(d)
                return

            if isinstance(source, (list, tuple, set)):
                for item in source:
                    feed(item)
                return

            d = self._extract_distance_from_stop_item(source, ego_location)
            if d is not None:
                distances.append(d)

        feed(closest_stop_sign)
        feed(traffic_signs_stop)

        distances = [float(d) for d in distances if d is not None and 0.0 < float(d) < 80.0]
        if not distances:
            return None

        return min(distances)

    def _analyze_curve_speed(self, localPath):
        if localPath is None:
            return 999.0 
        if len(localPath) < 5:
            return 999.0 

        lookahead = min(len(localPath), 60)
        step = 5 # Paso para ver la curva bien
        
        radii = [] 

        # función para extraer x e y de los waypoints
        def get_coords(point):
                return point.transform.location.x, point.transform.location.y

        for i in range(0, lookahead - 2 * step, 2): 
            # Extraemos las coordenadas REALES usando el helper
            try:
                p1_x, p1_y = get_coords(localPath[i])
                p2_x, p2_y = get_coords(localPath[i + step])
                p3_x, p3_y = get_coords(localPath[i + 2 * step])
            except:
                continue # Si falla al leer un punto, saltamos
            
            # Distancias
            a = np.hypot(p2_x-p1_x, p2_y-p1_y) ###Creo que dividiendo entre 2 calcula mejor si hay curva
            b = np.hypot(p3_x-p2_x, p3_y-p2_y)
            c = np.hypot(p3_x-p1_x, p3_y-p1_y)
            
            if a < 0.01 or b < 0.01: continue

            # Fórmula Herón para calcular el área del triángulo
            s = (a+b+c)/2   
            area = np.sqrt(max(s*(s-a)*(s-b)*(s-c), 0.0))
            
            if area < 0.001: continue 
            
            R = (a*b*c) / (4*area)
            
            if R < 150.0:
                radii.append(R)

        # resultado
        if not radii:
            return 999.0 

        radii.sort()
        count = min(len(radii), 3)
        avg_radius = sum(radii[:count]) / count

        v_lim = np.sqrt(self.cfg.a_lat_max_mps2 * avg_radius)
        
        return max(v_lim, self.cfg.curve_min_v_mps)
    
    def _get_turn_direction_at_junction(self, localPath, distance_to_junction):
        """
        Devuelve:
        - left si estamos cerca de una intersección y la ruta gira a la izquierda.
        - right si gira a la derecha.
        - straight si no hay giro claro o no estamos cerca.
        """
        if distance_to_junction is None:
            return "straight"

        if distance_to_junction > self.cfg.intersection_check_m or distance_to_junction < -8.0:
            return "straight"

        if localPath is None or len(localPath) < 12:
            return "straight"

        def get_xy(point):
            return point.transform.location.x, point.transform.location.y

        try:
            p0x, p0y = get_xy(localPath[0])
            p1x, p1y = get_xy(localPath[5])
            p2x, p2y = get_xy(localPath[min(25, len(localPath)-1)])
        except:
            return "straight"

        yaw0 = np.arctan2(p1y - p0y, p1x - p0x)
        yaw1 = np.arctan2(p2y - p1y, p2x - p1x)

        dyaw = np.arctan2(np.sin(yaw1 - yaw0), np.cos(yaw1 - yaw0))
        dyaw_deg = abs(np.degrees(dyaw))

        if dyaw_deg < self.cfg.intersection_turn_yaw_deg:
            return "straight"

        return "left" if dyaw > 0.0 else "right"

    def _choose_state(self, d_ahead, v_now, is_static_obstacle, obstacle_kind=None, v_curve_limit=999.0):
        cfg = self.cfg

        if d_ahead is None:
            return self.STATE_CRUISE
        
        if d_ahead is None:
            return self.STATE_CRUISE

        # Si un actor tipo pedestrian está ya muy cerca en el carril propio,
        # no se intenta seguirlo: se fuerza frenada de emergencia.
        if obstacle_kind == "PED_IN_LANE" and d_ahead <= cfg.emergency_ped_distance_m:
            if v_now < 0.2:
                return self.STATE_STOPPED
            return self.STATE_BRAKING

        if obstacle_kind == "CROSS_TRAFFIC":
            dist_stop = self.cfg.cross_traffic_stop_m
        elif obstacle_kind == "TL":
            dist_stop = max(self.cfg.stop_distance_m, self.cfg.traffic_light_stop_m)
        elif obstacle_kind == "STOP_SIGN":
            dist_stop = self.cfg.stop_sign_stop_m
        elif obstacle_kind == "FRONT_OBSTACLE":
            dist_stop = self.cfg.stop_distance_m
        elif is_static_obstacle:
            dist_stop = self.cfg.traffic_light_stop_m
        else:
            dist_stop = self.cfg.stop_distance_m

        if d_ahead <= (dist_stop + 0.5) and v_now < 0.2:
            return self.STATE_STOPPED

        d_frenado_necesaria = (
            (v_now**2) / (2.0 * self.cfg.decel_limit_mps2)
            + dist_stop
            + (v_now * 1.2)
        )

        if d_ahead < d_frenado_necesaria:
            return self.STATE_BRAKING

        #curve_required = v_curve_limit < (v_now * 0.6) or v_curve_limit < (self.v_max * 0.8)

        if d_ahead < cfg.follow_start_m:
            if is_static_obstacle:
                return self.STATE_BRAKING
            return self.STATE_FOLLOW

        if d_ahead < cfg.coasting_start_m:
            return self.STATE_COASTING

        return self.STATE_CRUISE

    def _get_target_speed(self, state, v_now, v_max_used, d_ahead, is_static_obstacle, obstacle_kind=None, v_curve_limit=999.0):
        
        if obstacle_kind == "CROSS_TRAFFIC":
            dist_stop = self.cfg.cross_traffic_stop_m
        elif obstacle_kind == "TL":
            dist_stop = max(self.cfg.stop_distance_m, self.cfg.traffic_light_stop_m)
        elif obstacle_kind == "STOP_SIGN":
            dist_stop = self.cfg.stop_sign_stop_m
        elif obstacle_kind == "FRONT_OBSTACLE":
            dist_stop = self.cfg.stop_distance_m
        elif is_static_obstacle:
            dist_stop = self.cfg.traffic_light_stop_m
        else:
            dist_stop = self.cfg.stop_distance_m

        if state == self.STATE_STOPPED:
            return 0.0

        elif state == self.STATE_BRAKING:
            dist_total_libre = max(d_ahead - dist_stop, 0.0)
            if dist_total_libre < 1.0 and v_now < 1.5:
                return 0.0
            dist_reaccion = v_now * 0.0  #*0.5 # 0.5 segundos de margen ###borrar si al final la dejo en 0
            dist_efectiva = max(dist_total_libre - dist_reaccion, 0.0)
            # v max q te permite parar
            v_brake_physics = np.sqrt(2.0 * self.cfg.decel_limit_mps2 * dist_efectiva)  #v^2 = vo^2 + 2*a*d
            v_brake_safe = v_brake_physics * 0.7 #*factor de seguridad
            v_ret = min(v_max_used, v_brake_safe)

            return v_ret

        elif state == self.STATE_FOLLOW: ########
            v_lead = max(self.v_lead_est, 0.0) 
            desired_gap = self.cfg.min_distance_follow + self.cfg.time_gap_follow * max(v_now,0.0)

            e = d_ahead - desired_gap #error distancia (>0 vas lejos, <0 vas cerca)

            # Control: iguala v_lead y corrige distancia poco a poco
            #si funciona raro, cambiar entre sí los parámetros de far y close
            k_p_far = 0.85  # (m/s) por metro. Más bajo = decelera poco
            k_p_close = 0.4    # la idea es, si estás cerca corriges más fuerte, si estás lejos recuperas distancia más suave
            k_p = k_p_close if e < 0 else k_p_far
            
            v_cmd = v_lead + k_p * e 
            
            v_rel = v_now - v_lead
            if v_rel > 1.5 and d_ahead < 20.0:
                # Reducimos v_cmd extra si nos acercamos rápido
                v_cmd -= (v_rel * 0.7)

            v_clip = np.clip(v_cmd, 0.0, v_max_used * 0.95)
            return v_clip

        if state == self.STATE_COASTING:
            return v_now * 0.9

        return v_max_used # CRUISE
    
    def _lane_change_decision(self, obs, d_ahead, obstacle_kind, v_now, v_max_used, block_new_lc=False):
        cfg = self.cfg
        dt = float(cfg.dt_s)

        self.lane_change_urgent = False

        # Bloquea SOLO el inicio de nuevos cambios de carril en curva.
        # Si ya hay un cambio activo, no lo cancela.
        if block_new_lc and self.lane_cmd == 0:
            return 0
        
        #bloqueo por semáforo
        d_tl = obs.get("d_traffic_light_m")
        d_stop = obs.get("d_stop_sign_m")
        tl_is_red = (d_tl is not None and d_tl > 0) or (d_stop is not None and d_stop > 0 and not self.stop_cleared)

        # El timer se calcula en step(), aquí solo miramos si ya ha llegado al límite.
        frustrated = self.stopped_frustration_timer >= cfg.max_stopped_time_s

        # cooldown para no cambiar de decisión muy seguido,
        # pero no debe bloquear una salida urgente por frustración.
        if self.lc_cooldown_t > 0.0 and not frustrated:
            return 0


        #solo permitir adelantamiento en una ventana razonable (ni lejos ni cerca)
        # Coche delantero prácticamente parado.
        # Se calcula ANTES de descartar por distancia, porque si no a 1 m ya no permite iniciar.
        almost_static_lead = (
            obstacle_kind in ("VEHICLE", "FRONT_OBSTACLE") and
            d_ahead is not None and
            d_ahead <= cfg.lc_static_trigger_m and
            max(self.v_lead_est, 0.0) <= cfg.lc_static_lead_v_mps
        )

        # solo permitir adelantamiento en una ventana razonable
        lc_trigger_m = cfg.lc_trigger_m
        lc_min_start_m = max(cfg.lc_min_start_m, 0.5*v_now+2.0)

        # demasiado lejos, no adelantar
        # Excepción: coche prácticamente parado dentro de su trigger específico.
        if d_ahead is None:
            return 0

        if d_ahead > lc_trigger_m and not almost_static_lead:
            return 0

        # demasiado cerca, no iniciar adelantamiento normal
        # Excepción: coche prácticamente parado. En ese caso sí dejamos decidir,
        # porque si no se queda bloqueado a 1 m sin cambiar.
        if not frustrated and d_ahead < lc_min_start_m and not almost_static_lead:
            return 0

        if obstacle_kind in ["VEHICLE","PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE"] and not frustrated:
            v_lead = max(self.v_lead_est, 0.0)

            # Si el coche de delante no lleva suficiente tiempo siendo lento,
            # no hacemos cambio de carril. Esto evita cambios innecesarios por cut-in.
            # Excepción: coche prácticamente parado y cerca -> se permite decidir antes.
            if self.lead_slow_counter < cfg.lc_slow_ticks and not almost_static_lead:
                return 0

            # Si el lead ya va relativamente rápido, no merece la pena adelantar.
            if v_lead >= cfg.lc_max_overtake_lead_v_mps and not almost_static_lead:
                return 0

            # Si no tengo ventaja clara de velocidad, no adelanto.
            if (v_max_used - v_lead) < cfg.lc_min_speed_advantage_mps and not almost_static_lead:
                return 0
        
        min_gap_frontal = 999.0  # Bloqueado cambio si detecta alguien. No cambia si hay alguien a menos
        min_gap_trasero = 18.0  # No cambia si hay alguien a menos 

        # Gap dinámico (tiempo de seguridad)
        front_need = max(min_gap_frontal, cfg.lc_front_min_m + cfg.lc_time_gap_s * max(v_now, 0.0))
        rear_need  = max(min_gap_trasero, cfg.lc_rear_min_m  + cfg.lc_time_gap_s * max(v_now, 0.0))

        #permite en estados follow,braking,stopped
        allowed = (self.current_state in (self.STATE_FOLLOW, self.STATE_BRAKING, self.STATE_STOPPED))
        if not allowed:
            return 0

        # Extraemos distancias de los sensores
        lf = obs.get("d_left_front_m")
        rf = obs.get("d_right_front_m") # <--- Añadimos el derecho también
        lr = obs.get("d_left_rear_m")
        rr = obs.get("d_right_rear_m")
        
        l_normal = obs.get("left_normal_exists", False)
        l_opposite = obs.get("left_opposite_exists", False)

        # Si ya hemos dado la orden de girar (lane_cmd != 0), 
        # ignoramos lf y rf para que el coche que rebasamos no nos frene.
        if self.lane_cmd != 0 and obstacle_kind != "TL":
            lf = None
            rf = None

        # Atascado
        if frustrated:
            distancia_seguridad_urgencia = 8.0 #distancia q mira de carril contrario para iniciar adelantamiento
            
            # Comprobamos que no haya nadie pegado en los dos lados frontales
            libre_de_frente = (lf is None or lf > distancia_seguridad_urgencia) and \
                              (rf is None or rf > distancia_seguridad_urgencia)
            
            if (l_normal or l_opposite) and libre_de_frente:
                self.lane_change_urgent = True
                print(f"[URGENCIA] Iniciando maniobra. Ignorando interferencias de curva.")
                return -1 # Ordenamos salida a la izquierda
            return 0


        if obstacle_kind not in ["VEHICLE", "PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE"] or d_ahead is None:
            return 0
            
        v_lead = max(self.v_lead_est, 0.0)
        if v_lead < 1.0 and not almost_static_lead:
            return 0

        # Gaps normales
        front_need = max(cfg.lc_front_min_m, 10.0 + cfg.lc_time_gap_s * max(v_now, 0.0))
        rear_need  = max(cfg.lc_rear_min_m, 12.0 + cfg.lc_time_gap_s * max(v_now, 0.0))

        left_ok = False
        if l_normal and not l_opposite:
            left_ok = True
            if lf is not None and lf < front_need: left_ok = False
            if lr is not None and lr < (rear_need * 0.7): left_ok = False
            
        
        if left_ok: return -1
        return 0
        
        '''#si ambos seguros elegimos el que tiene más espacio delante
        #if left_ok:  return -1
        #if right_ok: return +1
        return 0'''



    def step(self, bev_segmentation_cls, bev_lane_info, traffic_light_info, distance_to_junction, actual_velocity, 
             localPath=None, v_max=None, yaw_rate=None, location=None,
             traffic_signs_stop=None, closest_stop_sign=None, obstacle_in_front=None):

        dt = float(self.cfg.dt_s)
        v_max_used = float(v_max if v_max is not None else self.v_max)

        # control de temporizador de stop
        if self.stop_cooldown_s > 0.0:
            self.stop_cooldown_s = max(0.0, self.stop_cooldown_s - dt)
            if self.stop_cooldown_s == 0.0:
                self.stop_cleared = False # El cooldown terminó, el coche ya pasó el cruce, reactivamos los STOPs

        # cooldown timer lane change
        self.lc_cooldown_t = max(self.lc_cooldown_t - float(self.cfg.dt_s), 0.0)
        v_now = float(actual_velocity)
        v_curve_instant = self._analyze_curve_speed(localPath)

        if v_curve_instant < self.v_curve_mem:
            self.v_curve_mem = v_curve_instant
        else:
            recovery_rate = 2.0 * self.cfg.dt_s 
            self.v_curve_mem = min(v_curve_instant, self.v_curve_mem + recovery_rate)
        v_curve_limit = self.v_curve_mem

        if yaw_rate is not None:
            yr = float(yaw_rate)
            a = float(self.cfg.curve_filter_alpha)
            self.yaw_rate_f = a*self.yaw_rate_f + (1.0-a) * yr
            yr_abs = abs(self.yaw_rate_f)
            if yr_abs > 0.15:
                v_curve_limit = np.clip(self.cfg.a_lat_max_mps2/yr_abs, self.cfg.curve_min_v_mps,v_max_used)

        turn_direction = self._get_turn_direction_at_junction(localPath, distance_to_junction)
        turning_at_junction = turn_direction in ("left", "right")

        # Observar entorno
        # Conectamos STOP desde cualquiera de las dos entradas posibles:
        # - closest_stop_sign: si el agente ya trae la señal más cercana
        # - traffic_signs_stop: si el agente trae una lista/dict de señales STOP
        stop_sign_distance_m = self._resolve_stop_sign_distance(
            closest_stop_sign=closest_stop_sign,
            traffic_signs_stop=traffic_signs_stop,
            ego_location=location
        )

        obs = self.observer.observe(bev_segmentation_cls, bev_lane_info, traffic_light_info, distance_to_junction, 
                                    turn_direction=turn_direction, closest_stop_sign=stop_sign_distance_m, 
                                    obstacle_in_front=obstacle_in_front)
        

        # memoria específica de obstáculo frontal
        front_obstacle_seen_now = bool(obs.get("obstacle_front_detected", False))
        front_obstacle_raw_d = obs.get("d_obstacle_front_m")

        if front_obstacle_seen_now:
            self.front_obstacle_latched = True

            if front_obstacle_raw_d is not None:
                try:
                    self.front_obstacle_latch_d = float(front_obstacle_raw_d)
                except (TypeError, ValueError):
                    self.front_obstacle_latch_d = float(self.cfg.obstacle_front_bool_fallback_m)
            else:
                self.front_obstacle_latch_d = float(self.cfg.obstacle_front_bool_fallback_m)

            self.front_obstacle_lc_timer_s = 0.0

        elif self.front_obstacle_latched and self.front_obstacle_latch_d is not None:
            # Si la percepción deja de ver la valla, no la olvidamos.
            # Estimamos que nos acercamos a ella restando lo avanzado por el ego.
            self.front_obstacle_latch_d = max(
                float(self.cfg.obstacle_front_min_latch_m),
                float(self.front_obstacle_latch_d) - max(v_now, 0.0) * dt
            )
        
        self.obs = {
            'red_light_distance': obs.get('d_traffic_light_m') or 0.0,
            'd_vehicle_ahead_m': obs.get('d_vehicle_ahead_m') or 999.0,
            'd_left_front_m': obs.get('d_left_front_m') or 999.0,
            'd_right_front_m': obs.get('d_right_front_m') or 999.0,
            'd_intersecting_vehicle_m': obs.get('d_intersecting_vehicle_m') or 999.0,
            'd_pedestrian_m': obs.get('d_pedestrian_in_lane_m') or 999.0,
            'd_left_rear_m': obs.get('d_left_rear_m') or 999.0,
            'd_right_rear_m': obs.get('d_right_rear_m') or 999.0,
            'left_normal_exists': obs.get('left_normal_exists', True),
            'left_opposite_exists': obs.get('left_opposite_exists', True),
            'right_lane_exists': obs.get('right_lane_exists', True),
            # Guardamos los nuevos en la memoria interna de la clase
            'd_stop_sign_m': obs.get('d_stop_sign_m') or 999.0,
            'd_obstacle_front_m': obs.get('d_obstacle_front_m') or 999.0,
            'front_obstacle_latched': bool(self.front_obstacle_latched),
            'front_obstacle_latch_d_m': self.front_obstacle_latch_d if self.front_obstacle_latch_d is not None else 999.0,
        }
        
        d_tl_raw = obs.get("d_traffic_light_m")
        d_tl_final = None

        
        # Tiempo acumulado sin ver rojo/amarillo real.
        # Sirve para no quedarse bloqueado si el rojo se recuerda, pero el verde nunca llega a clasificarse.
        if d_tl_raw is not None and d_tl_raw > 0.0:
            self.tl_no_red_timer_s = 0.0
        elif d_tl_raw == -1.0:
            self.tl_no_red_timer_s = 0.0
        elif self.last_tl_dist is not None:
            self.tl_no_red_timer_s += dt
        else:
            self.tl_no_red_timer_s = 0.0
        
        if d_tl_raw == -1.0: #verde
            # Borra recuerdo específico del semáforo rojo
            self.last_tl_dist = None
            self.tl_loss_counter = 0
            d_tl_final = None

            # Borra también la memoria general si estaba recordando un TL.
            # Si no, puede seguir usando un semáforo fantasma durante varios frames.
            if self.last_obs_kind == "TL":
                self.last_obs_d = None
                self.last_obs_kind = None
                self.obs_loss_counter = 0

        elif d_tl_raw is not None:
            # Vemos semáforo rojo/amarillo 
            self.last_tl_dist = d_tl_raw
            self.tl_loss_counter = 0
            d_tl_final = d_tl_raw


        else:
            if self.last_tl_dist is not None:
                self.last_tl_dist = max(self.last_tl_dist - v_now * self.cfg.dt_s, 0.0)
                
                # if v_now<0.2 and self.last_tl_dist <= (self.cfg.traffic_light_stop_m + 2.0):
                #     self.tl_loss_counter = 0
                # else:
                #     self.tl_loss_counter += 1

                waiting_red = (v_now < 0.5 and self.last_tl_dist <= (self.cfg.traffic_light_stop_m + 2.0))
                if waiting_red:
                    self.tl_loss_counter = 0
                    d_tl_final = self.last_tl_dist
                else:
                    self.tl_loss_counter += 1

                    if self.tl_loss_counter < self.TL_MEMORY_LIMIT:
                        d_tl_final = self.last_tl_dist
                    else:
                        self.last_tl_dist = None
                        d_tl_final = None
            else:
                d_tl_final = None

        # Si llevamos demasiado tiempo sin volver a ver rojo/amarillo,
        # asumimos que el semáforo ya no debe bloquear la ruta.
        if self.tl_no_red_timer_s >= self.cfg.tl_forget_after_no_red_s:

            self.last_tl_dist = None
            self.tl_loss_counter = 0
            self.tl_no_red_timer_s = 0.0
            d_tl_final = None

            # importante: borrar también la memoria general del obstáculo TL
            if self.last_obs_kind == "TL":
                self.last_obs_d = None
                self.last_obs_kind = None
                self.obs_loss_counter = 0

        self.obs['red_light_distance'] = (float(d_tl_final) if d_tl_final is not None and d_tl_final > 0.0 else 0.0)

        d_stop_raw = obs.get("d_stop_sign_m")
        d_stop_final = None
        if d_stop_raw is not None and 0.0 < d_stop_raw < 20.0:
            # Si ya hicimos la parada reglamentaria o estamos en cooldown largo, ignoramos la señal por completo
            if self.stop_cleared or self.stop_cooldown_s > 0.0:
                d_stop_final = None
            else:
                d_stop_final = d_stop_raw
        
        # Distancia bruta a tráfico cruzado detectado en la BEV
        d_cross_raw = obs.get("d_intersecting_vehicle_m")

        # Señales normativas activas.
        # para que el tráfico cruzado no compita contra un semáforo o STOP.
        signal_stop_active = d_tl_final is not None and d_tl_final > 0.0
        stop_sign_active = d_stop_final is not None and d_stop_final > 0.0 and not self.stop_cleared

        candidates = []
        
        if obs.get("d_vehicle_ahead_m") is not None:
            candidates.append((obs.get("d_vehicle_ahead_m"), "VEHICLE")) # False = Coche
        
        if obs.get("d_pedestrian_in_lane_m") is not None:
            candidates.append((obs.get("d_pedestrian_in_lane_m"), "PED_IN_LANE")) 

        if d_tl_final is not None:
            candidates.append((d_tl_final, "TL")) # True = Estático

        if d_stop_final is not None:
            candidates.append((d_stop_final, "STOP_SIGN"))

        d_obs_front = obs.get("d_obstacle_front_m")

        # Si la percepción actual no ve la valla, pero la tenemos memorizada,
        # seguimos usándola como obstáculo frontal.
        if self.front_obstacle_latched and self.front_obstacle_latch_d is not None:
            d_obs_front = float(self.front_obstacle_latch_d)

        if d_obs_front is not None and d_obs_front > 0.0:
            # Lo consideramos como VEHICLE para reutilizar la lógica de frenada + adelantamiento.
            candidates.append((d_obs_front, "FRONT_OBSTACLE"))

        # Riesgo de tráfico cruzado en intersección.
        # El tráfico cruzado solo se considera si no estamos ya bloqueados por semáforo o STOP.
        cross_relevant = (
            turning_at_junction and
            not signal_stop_active and
            not stop_sign_active
        )

        cross_detected = (
            cross_relevant and
            d_cross_raw is not None and
            d_cross_raw < self.cfg.cross_traffic_max_m
        )

        if cross_detected:
            self.cross_traffic_memory = self.cfg.cross_traffic_memory_frames
            self.last_cross_traffic_d = float(d_cross_raw)
        elif cross_relevant and self.cross_traffic_memory > 0:
            self.cross_traffic_memory -= 1
            cross_detected = True
            d_cross_raw = self.last_cross_traffic_d
        else:
            self.cross_traffic_memory = 0
            self.last_cross_traffic_d = None

        # Marcamos que el ego se ha quedado parado esperando tráfico cruzado
        # en una intersección con giro.
        if cross_detected and turning_at_junction and v_now < 0.5:
            self.stopped_for_cross_turn = True

        # Solo damos boost si realmente veníamos de estar parados por ese tráfico cruzado.
        if (
            self.cross_was_detected
            and not cross_detected
            and turning_at_junction
            and self.stopped_for_cross_turn
        ):
            self.cross_clear_boost_timer_s = float(self.cfg.cross_clear_boost_s)
            self.stopped_for_cross_turn = False

        self.cross_was_detected = bool(cross_detected)

        # Si ya no estamos en una intersección con giro, limpiamos la bandera.
        if not turning_at_junction:
            self.stopped_for_cross_turn = False

        if cross_detected:
            # Si aún estamos antes de la intersección, usamos distance_to_junction
            # para frenar antes de entrar. Si ya estamos dentro, usamos la distancia
            # detectada al vehículo cruzado.
            if distance_to_junction is not None and distance_to_junction > 0:
                d_cross_candidate = min(float(distance_to_junction), float(d_cross_raw))
            else:
                d_cross_candidate = float(d_cross_raw) if d_cross_raw is not None else 0.0

            candidates.append((d_cross_candidate, "CROSS_TRAFFIC"))

        if candidates:
            d_ahead_raw, obstacle_kind_raw = min(candidates, key=lambda x: x[0])
        else:
            d_ahead_raw, obstacle_kind_raw = None, None

        # evitar parpadeo de obstáculos
        usar_memoria = False
        
        # Si recordamos un obstáculo, y de repente el nuevo obstáculo desaparece
        # o está más lejos que el que recordábamos, es que el sensor ha parpadeado.
        if self.last_obs_d is not None:
            if obstacle_kind_raw == "CROSS_TRAFFIC":
                usar_memoria = False
            elif d_ahead_raw is None or d_ahead_raw > (self.last_obs_d + 2.0):
                usar_memoria = True

        if usar_memoria:
            self.last_obs_d = max(self.last_obs_d - (v_now * dt), 0.0)
            self.obs_loss_counter += 1

            if self.obs_loss_counter < self.OBS_MEMORY_LIMIT:
                # Usamos el obstáculo recordado (fantasma)
                d_ahead = self.last_obs_d
                obstacle_kind = self.last_obs_kind
            else:
                # Ya ha pasado demasiado tiempo, lo olvidamos
                self.last_obs_d = d_ahead_raw
                self.last_obs_kind = obstacle_kind_raw
                d_ahead = d_ahead_raw
                obstacle_kind = obstacle_kind_raw
                self.obs_loss_counter = 0
        else:
            # Confiamos en el sensor actual porque lo que vemos es lógico
            self.last_obs_d = d_ahead_raw
            self.last_obs_kind = obstacle_kind_raw
            self.obs_loss_counter = 0
            d_ahead = d_ahead_raw
            obstacle_kind = obstacle_kind_raw

        self.obs['target_obstacle_kind'] = obstacle_kind
        self.obs['target_obstacle_distance_m'] = d_ahead

        # Timer de bloqueo por obstáculo frontal
        blocked_by_signal = (
            (d_tl_final is not None and d_tl_final > 0.0) or
            (d_stop_final is not None and d_stop_final > 0.0 and not self.stop_cleared)
        )

        front_obstacle_timer_active = False
        d_obs_for_timer = d_obs_front

        if d_obs_for_timer is not None and not isinstance(d_obs_for_timer, bool):
            try:
                d_obs_for_timer = float(d_obs_for_timer)
                front_obstacle_timer_active = (
                    0.0 < d_obs_for_timer < max(30.0, float(self.cfg.lc_static_trigger_m))
                )
            except (TypeError, ValueError):
                front_obstacle_timer_active = False

        front_blocked_for_frustration = (
            (
                obstacle_kind in ("VEHICLE", "PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE") and
                d_ahead is not None and
                d_ahead < max(26.0, float(self.cfg.lc_static_trigger_m))
            )
            or front_obstacle_timer_active
        )

        if v_now < 0.5 and front_blocked_for_frustration and not blocked_by_signal:
            self.stopped_frustration_timer += dt
        else:
            self.stopped_frustration_timer = 0.0

        self.obs['stopped_frustration_timer_s'] = float(self.stopped_frustration_timer)
        self.obs['front_blocked_for_frustration'] = bool(front_blocked_for_frustration)
        self.obs['blocked_by_signal'] = bool(blocked_by_signal)

        # cronómetro interno del stop
        if self.current_state == self.STATE_STOPPED and obstacle_kind == "STOP_SIGN":
            self.stop_timer_s += dt
            if self.stop_timer_s >= self.cfg.stop_sign_wait_time_s:
                self.stop_cleared = True
                self.stop_cooldown_s = self.cfg.stop_sign_clear_cooldown_s
                self.stop_timer_s = 0.0
                print(f"[STOP] Parada obligatoria de {self.cfg.stop_sign_wait_time_s}s cumplida. Reanudando marcha...")
        elif obstacle_kind != "STOP_SIGN" and not self.stop_cleared:
            # Si el obstáculo cambia o nos movemos antes de tiempo, reiniciamos el contador
            self.stop_timer_s = 0.0

        # Frenada de emergencia para pedestrian/moto muy cercano en el carril propio.
        # Se activa solo a distancia muy corta para no romper casos donde pedestrian son motos/bicis.
        self.emergency_brake_active = (
            obstacle_kind == "PED_IN_LANE" and
            d_ahead is not None and
            d_ahead <= self.cfg.emergency_ped_distance_m
)
        
        # Estimar velocidad del lead.
        # Para FRONT_OBSTACLE no estimamos velocidad: es una valla/obstáculo estático.
        if obstacle_kind == "FRONT_OBSTACLE":
            self.prev_lead_d = None
            self.prev_lead_kind = None
            self.v_lead_est = 0.0

        else:
            lead_kind = obstacle_kind

            if lead_kind == "PED_IN_LANE":
                lead_kind = "VEHICLE"

            if lead_kind == "VEHICLE" and d_ahead is not None:
                if self.prev_lead_kind != lead_kind:
                    self.prev_lead_d = None

                if self.prev_lead_d is not None:
                    d_dot = (d_ahead - self.prev_lead_d) / max(dt, 1e-6)
                    d_dot = np.clip(d_dot, -15.0, 6.0)
                    v_lead_raw = max(v_now + d_dot, 0.0)
                    self.v_lead_est = self.v_lead_alpha * self.v_lead_est + (1.0 - self.v_lead_alpha) * v_lead_raw
                else:
                    self.v_lead_est = v_now

                self.prev_lead_d = d_ahead
                self.prev_lead_kind = lead_kind

            else:
                self.prev_lead_d = None
                self.prev_lead_kind = None
                self.v_lead_est = 0.0
        
        slow_by_abs   = (self.v_lead_est <= self.cfg.lc_slow_v_mps)
        slow_by_ratio = (self.v_lead_est <= self.cfg.lc_slow_ratio * v_max_used)

        # Contador de persistencia trafico lento
        if obstacle_kind in ["VEHICLE", "PED_IN_LANE", "FRONT_OBSTACLE"] and d_ahead is not None and (slow_by_abs or slow_by_ratio):
            self.lead_slow_counter += 1
        else:
            self.lead_slow_counter = max (0, self.lead_slow_counter -1)


        # Decisión cambio de carril
        curve_active_for_lc = (
            self.cfg.block_lc_in_curve and
            self.lane_cmd == 0 and
            v_curve_limit < self.cfg.lc_curve_block_ratio * v_max_used
        )

        lead_almost_stopped_for_curve_lc = (
            obstacle_kind in ("VEHICLE", "FRONT_OBSTACLE") and
            d_ahead is not None and
            (
                obstacle_kind == "FRONT_OBSTACLE" or
                max(self.v_lead_est, 0.0) < self.cfg.lc_curve_allow_if_lead_below_mps
            )
        )

        lane_cmd_candidate = self._lane_change_decision(obs, d_ahead, obstacle_kind, v_now, v_max_used)

        if curve_active_for_lc and not lead_almost_stopped_for_curve_lc and not self.lane_change_urgent:
            lane_cmd = 0
        else:
            lane_cmd = lane_cmd_candidate


        if lane_cmd != 0:
            self.lane_cmd = lane_cmd
            self.lc_cooldown_t = float(self.cfg.lc_cooldown_s)

            # Cambio urgente/frustrado: salida suave desde parado o bloqueo.
            if self.lane_change_urgent:
                self.frustrated_escape_timer_s = float(self.frustrated_escape_duration_s)
                self.normal_lc_speed_timer_s = 0.0

            # Cambio normal: durante unos segundos evitamos que la velocidad caiga demasiado.
            else:
                self.normal_lc_speed_timer_s = float(self.cfg.lc_normal_speed_duration_s)

        else:
            if self.lc_cooldown_t <= 0.0:
                self.lane_cmd = 0

        # borrado de memoria de valla tras cambio de carril
        if self.front_obstacle_latched and self.lane_cmd != 0:
            self.front_obstacle_lc_timer_s += dt

            if self.front_obstacle_lc_timer_s >= self.cfg.obstacle_front_clear_after_lc_s:
                self._clear_front_obstacle_latch()

                # Limpiamos también la memoria general para no arrastrar un obstáculo fantasma.
                if self.last_obs_kind in ("VEHICLE", "FRONT_OBSTACLE"):
                    self.last_obs_d = None
                    self.last_obs_kind = None
                    self.obs_loss_counter = 0

        elif not self.front_obstacle_latched:
            self.front_obstacle_lc_timer_s = 0.0
        
        lane_option = self._lane_cmd_to_option(self.lane_cmd)

        maneuver_escape_active = (
            self.lane_cmd != 0 and
            self.current_state == self.STATE_STOPPED and 
            obstacle_kind in ("VEHICLE", "PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE")
        )


        if obstacle_kind == "TL" and d_ahead is not None and v_now <0.2:
            self.tl_soft_approach = True
        if obstacle_kind != "TL" or d_ahead is None:
            self.tl_soft_approach = False

        is_static_obstacle = obstacle_kind in ["TL", "CROSS_TRAFFIC", "STOP_SIGN", "FRONT_OBSTACLE"]    #"PED_OUT_LANE" frena por motos

        # FSM: Cambiar de estado
        nuevo_estado = self._choose_state(d_ahead, v_now, is_static_obstacle, obstacle_kind=obstacle_kind, v_curve_limit=v_curve_limit)
        if nuevo_estado != self.current_state:
            tipo = obstacle_kind if obstacle_kind is not None else "LIBRE"
            print(f"[FSM] CAMBIO: {self.current_state} -> {nuevo_estado} | Tipo obstáculo: {tipo}")
            self.current_state = nuevo_estado

        if self.current_state in (self.STATE_FOLLOW, self.STATE_BRAKING, self.STATE_STOPPED): #añado stopped por si está bloqueado con vehiculo delante
            self.follow_ticks_counter += 1
        else:
            self.follow_ticks_counter = max (0, self.follow_ticks_counter - 1)

        #Calcular v_target
        v_target = self._get_target_speed(self.current_state, v_now, v_max_used, d_ahead, is_static_obstacle=is_static_obstacle, obstacle_kind=obstacle_kind, v_curve_limit=v_curve_limit)
        
        ##
        # Cambio de carril normal:
        # evita que el coche se quede demasiado atrás durante el adelantamiento.
        # Solo se aplica en FOLLOW, nunca en BRAKING ni STOPPED.
        normal_lc_speed_active = (
            self.normal_lc_speed_timer_s > 0.0 and
            self.frustrated_escape_timer_s <= 0.0 and
            self.current_state == self.STATE_FOLLOW and
            obstacle_kind in ("VEHICLE", "PED_IN_LANE", "PEDESTRIAN", "FRONT_OBSTACLE")
        )

        if normal_lc_speed_active:
            safe_gap_for_lc_speed = (
                d_ahead is not None and
                d_ahead > max(
                    self.cfg.stop_distance_m + 4.0,
                    (v_now ** 2) / (2.0 * self.cfg.decel_limit_mps2) + self.cfg.stop_distance_m + 2.0
                )
            )

            if safe_gap_for_lc_speed:
                v_lc_floor = min(
                    v_max_used * 0.90,
                    max(
                        v_now * self.cfg.lc_normal_keep_speed_ratio,
                        v_max_used * self.cfg.lc_normal_min_speed_ratio
                    )
                )

                v_target = max(v_target, v_lc_floor)
        ##
        if maneuver_escape_active:
            v_target = max(v_target, 2.2)

        # Durante la salida por frustración, limitamos la velocidad objetivo.
        # No cambia el stop ni la lógica normal: solo suaviza la rampa inicial del adelantamiento urgente.
        frustrated_escape_active = (self.frustrated_escape_timer_s > 0.0 and self.lane_cmd != 0)
        if frustrated_escape_active:
            v_target = min(v_target, self.frustrated_escape_v_mps)

        #v_curve_limit = 999.0   #prueba, borrar al final

        cross_escape_active = self.cross_clear_boost_timer_s > 0.0

        # Si acabamos de liberar un cruce después de estar parados por tráfico cruzado,
        # mandamos velocidad máxima para salir de la intersección.
        if cross_escape_active:
            v_target = v_max_used

        # En ese caso concreto no aplicamos el límite de curva,
        # porque queremos terminar de cruzar la intersección con decisión.
        if v_curve_limit is not None and not cross_escape_active:
            v_target = min(v_target, v_curve_limit)


        # aplicar dinámica EMA
        v_out, _ = self._apply_dynamics_and_ema(v_now, v_target, v_max_used, self.current_state)

        # El temporizador se consume después de calcular la velocidad de este frame.
        if self.frustrated_escape_timer_s > 0.0:
            self.frustrated_escape_timer_s = max(self.frustrated_escape_timer_s - dt, 0.0)

        if self.normal_lc_speed_timer_s > 0.0:
            self.normal_lc_speed_timer_s = max(self.normal_lc_speed_timer_s - dt, 0.0)

        if self.cross_clear_boost_timer_s > 0.0:
            self.cross_clear_boost_timer_s = max(self.cross_clear_boost_timer_s - dt, 0.0)

        dist_str = f"{d_ahead:.1f}m" if d_ahead is not None else "Libre"
        lc_str = "L" if self.lane_cmd == -1 else ("R" if self.lane_cmd == 1 else "-")
        
        curve_active = (v_curve_limit < 0.95 * v_max_used)
        crv_str = "CURVE" if curve_active else "----"
        if lane_option in ["left", "right"]:
            aviso_lc = f"[CAMBIO ACTIVO: {lane_option.upper()}]"
        else:
            aviso_lc = "--- [CARRIL NORMAL] ---"
  
        # print(f"St: {self.current_state} | Dist: {dist_str:6} | {aviso_lc:25} | Carril  | {crv_str} | V_crv: {v_curve_limit:.1f} | V_ema: {v_out:.2f} | V_target: {v_target:.2f} | V_now: {v_now:.2f}")
        
        self.decision_speed = v_out    #descomentar si quiero usar filtro ema
        #self.decision_speed = v_target #comprobación sin filtro ema

        return float(self.decision_speed), str(lane_option), bool(self.lane_change_urgent)#, location
        #return self.decision_speed
    
    def _apply_dynamics_and_ema(self, v_now, v_target, v_max_used, state):
        dt = float(self.cfg.dt_s)
        
        # usam la velocidad que mandamos en el frame anterior
        try:
            v_prev = float(self.decision_speed) 
        except AttributeError:
            v_prev = 0.0
            
        # para el arranque
        if v_now < 0.2 and v_target > 0.5:
            if self.cross_clear_boost_timer_s > 0.0:
                v_start = min(v_target, v_max_used)
            else:
                v_start = min(v_target, 2.2)

            return float(v_start), float(v_target)

        #filtro ema asimetrico
        if self.current_state == self.STATE_BRAKING or v_target == 0.0:
            alpha = 0.0   # Cero inercia en emergencia: clava los frenos
        elif self.current_state == self.STATE_FOLLOW:
            if v_target < (v_prev - 1.0):
                alpha = 0.1 # Semi-frenazo si el de delante frena
            else:
                alpha = 0.6 # Seguimiento suave
        else:
            alpha = 0.8   # Alta inercia en crucero

        v_smoothed_desire = alpha * v_prev + (1.0 - alpha) * v_target

        # limites fisicos
        accel = float(self.cfg.accel_limit_mps2)
        decel = float(self.cfg.decel_limit_mps2)

        if self.last_obs_kind == "TL":
            decel = max(decel, 6.0)

        if self.emergency_brake_active:
            decel = max(decel, float(self.cfg.emergency_decel_mps2))

        if self.front_obstacle_latched:
            decel = max(decel, float(self.cfg.obstacle_front_force_decel_mps2))

        # Si el adelantamiento nace por frustración, la aceleración inicial también se limita.
        # Esto evita la sensación de tirón aunque la ruta lateral sea correcta.
        if self.frustrated_escape_timer_s > 0.0 and self.lane_cmd != 0:
            accel = min(accel, float(self.frustrated_escape_accel_mps2))

        if self.current_state == self.STATE_FOLLOW and self.last_obs_kind == "VEHICLE":
            accel = float(self.cfg.follow_accel_limit_mps2)
            decel = 2.0  # Límite más suave para confort

        if self.cross_clear_boost_timer_s > 0.0:
            accel = max(accel, float(self.cfg.cross_clear_accel_mps2))
        

        # Limitamos el cambio de velocidad (dv)
        dv = v_smoothed_desire - v_prev
        dv_limited = np.clip(dv, -decel * dt, accel * dt)
        
        v_final = np.clip(v_prev + dv_limited, 0.0, v_max_used)

        # parada para no quedarse avanzando lentisimo
        if v_target == 0.0 and v_now < 0.5:
            v_final = 0.0

        # Si el actor tipo pedestrian está prácticamente encima, mandamos stop directo.
        # Esto evita atropello aunque la rampa normal de deceleración sea demasiado suave.
        if self.emergency_brake_active and self.last_obs_d is not None:
            if self.last_obs_d <= self.cfg.emergency_direct_stop_m:
                v_final = 0.0
                    
        return float(v_final), float(v_target)
