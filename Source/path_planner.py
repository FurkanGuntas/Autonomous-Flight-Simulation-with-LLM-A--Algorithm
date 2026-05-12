import numpy as np
from geopy.distance import geodesic, distance
import heapq

# SINIF 1: ROTA PLANLAYICI (A*)
class PathPlanner:

    # Dinamik parametreleri alabilir
    def __init__(self, danger_zones, safety_distance_km, turn_penalty_config, step_distance_km): 
        self.danger_zones = danger_zones 
        self.safety_margin_m = safety_distance_km * 1000
        self.turn_penalty_config = turn_penalty_config
        self.step_distance_km = step_distance_km


    def haversine_calc(self, lat1, lon1, lat2, lon2):
            R = 6371000  
            dLat = np.radians(lat2 - lat1)
            dLon = np.radians(lon2 - lon1)
            lat1_rad = np.radians(lat1)
            lat2_rad = np.radians(lat2)
            a = (np.sin(dLat / 2) ** 2) + (np.sin(dLon / 2) ** 2) * np.cos(lat1_rad) * np.cos(lat2_rad)
            c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
            distance_m = R * c
            return distance_m

    def calculate_bearing(self, p1, p2):
        lat1_rad, lon1_rad = np.deg2rad(p1[0]), np.deg2rad(p1[1])
        lat2_rad, lon2_rad = np.deg2rad(p2[0]), np.deg2rad(p2[1])
        dLon = lon2_rad - lon1_rad
        y = np.sin(dLon) * np.cos(lat2_rad)
        x = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(lat2_rad) * np.cos(dLon)
        bearing_rad = np.arctan2(y, x)
        return (np.rad2deg(bearing_rad) + 360) % 360

    def is_path_in_danger(self, current_pos, target_wp):
        num_checks = 10
        for i in range(num_checks + 1):
            fraction = i / num_checks
            check_lat = current_pos[0] + fraction * (target_wp[0] - current_pos[0])
            check_lon = current_pos[1] + fraction * (target_wp[1] - current_pos[1])
            for zone in self.danger_zones:
                distance_to_danger = self.haversine_calc(check_lat, check_lon, zone['center'][0], zone['center'][1])
                if distance_to_danger < (zone['radius'] + self.safety_margin_m):
                    return True
        return False

    # get_neighbors fonksiyonu dinamik adım mesafesini kullanır
    def get_neighbors(self, current_wp, final_goal):
        neighbors = []
        current_bearing = self.calculate_bearing(current_wp, final_goal)
        # Artık parametre olarak gelen step_distance_km kullanılıyor
        step_distance_km = self.step_distance_km
        angles = [0, -20, 20, -10, 10, -5, 5, -45, 45, -55, 55, -65, 65]
        for angle in angles:
            new_bearing = (current_bearing + angle + 360) % 360
            new_point = distance(kilometers=step_distance_km).destination(point=current_wp, 
                                                                          bearing=new_bearing)
            neighbors.append((new_point.latitude, new_point.longitude))
        return neighbors

    # Dinamik ceza faktörlerini kullanır
    def calculate_turn_penalty(self, p1, p2, p3):
        if p1 is None: 
            return 0
        
        bearing1 = self.calculate_bearing(p1, p2)
        bearing2 = self.calculate_bearing(p2, p3)
        turn_angle = abs(bearing2 - bearing1)
        
        if turn_angle > 180: 
            turn_angle = 360 - turn_angle
        
        # Dinamik ceza faktörleri kullanılıyor
        if turn_angle >= 70: 
            return (turn_angle ** 2) * self.turn_penalty_config['sharp_turn_factor']
        elif turn_angle > 60: 
            return (turn_angle ** 1.5) * self.turn_penalty_config['moderate_turn_factor']
        return 0

    def find_safe_path(self, start_wp, end_wp):
        close_set = set() 
        came_from = {}
        g_score = {start_wp: 0}
        f_score = {start_wp: geodesic(start_wp, end_wp).meters}
        aday_noktalar = []
        heapq.heappush(aday_noktalar, (f_score[start_wp], start_wp))

        while aday_noktalar:
            current_wp = heapq.heappop(aday_noktalar)[1]

            if geodesic(current_wp, end_wp).meters < 5500:
                path = []
                while current_wp in came_from:
                    path.append(current_wp)
                    current_wp = came_from[current_wp]
                path.append(start_wp)
                return path[::-1]

            close_set.add(current_wp)
            parent_wp = came_from.get(current_wp, None)

            for neighbor_wp in self.get_neighbors(current_wp, end_wp):
                step_distance = geodesic(current_wp, neighbor_wp).meters

                danger_penalty = 0
                for zone in self.danger_zones:
                    distance_to_danger = geodesic(neighbor_wp, zone['center']).meters
                    if distance_to_danger < (zone['radius'] + self.safety_margin_m):
                        danger_penalty = 100000
                        break                 
                turn_penalty = self.calculate_turn_penalty(parent_wp, current_wp, neighbor_wp)
                temporary_g_score = g_score[current_wp] + step_distance + danger_penalty + turn_penalty

                if neighbor_wp in close_set: 
                    if temporary_g_score >= g_score.get(neighbor_wp, 0):
                       continue
                
                if temporary_g_score < g_score.get(neighbor_wp, float('inf')):
                    came_from[neighbor_wp] = current_wp
                    g_score[neighbor_wp] = temporary_g_score
                    f_score[neighbor_wp] = temporary_g_score + geodesic(neighbor_wp, end_wp).meters
                    heapq.heappush(aday_noktalar, (f_score[neighbor_wp], neighbor_wp))
        return None