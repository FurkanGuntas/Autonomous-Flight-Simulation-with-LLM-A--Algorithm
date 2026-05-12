import matplotlib.pyplot as plt
import numpy as np
from geopy.distance import distance

# SINIF 2: UÇUŞ VERİLERİNİ KAYDEDİCİ
class FlightData:

    def __init__(self):
        self.data = {
            "time": [], "lat": [], "lon": [], "alt": [],
            "speed": [], "pitch": [], "roll": [], "yaw": []
        }

    def log_state(self, fdm):

        self.data["time"].append(fdm.get_sim_time())
        self.data["lat"].append(fdm.get_property_value('position/lat-gc-deg'))
        self.data["lon"].append(fdm.get_property_value('position/long-gc-deg'))
        self.data["alt"].append(fdm.get_property_value('position/h-sl-ft'))
        self.data["speed"].append(fdm.get_property_value('velocities/vc-kts'))
        self.data["pitch"].append(fdm.get_property_value('attitude/theta-deg'))
        self.data["roll"].append(fdm.get_property_value('attitude/phi-deg'))
        self.data["yaw"].append(fdm.get_property_value('attitude/psi-deg'))


# SINIF 3: UÇUŞ VERİLERİNİ GÖRSELLEŞTİRİCİ 
class FlightDataVisualizer:

    def __init__(self, logger):
        self.data = logger.data

    def plot_2d_data(self, waypoints, generated_wps, danger_zones): 

        plt.figure("Flight Data", figsize=(12, 8))
        plt.suptitle("Cessna C310 A* Pathfinding Simulation", fontsize=16)

        plt.subplot(2, 2, 1)
        plt.plot(self.data['lon'], self.data['lat'])
        plt.xlabel("Longitude (deg)")
        plt.ylabel("Latitude (deg)")
        plt.title("2D Flight Path")
        
        # Ana waypoint'ler Kırmızı 'x' olarak çizilir
        wps_lat, wps_lon = zip(*waypoints)
        plt.scatter(wps_lon, wps_lat, c='red', marker='x', s=40, label='Ana Waypointler')

        # Oluşturulan virtual waypoint'leri çizmek için yeni bölüm
        if generated_wps: # Eğer en az bir virtual WP oluşturulduysa
            gen_wps_lat, gen_wps_lon = zip(*generated_wps)
            # Virtual waypoint'ler Yeşil 'o' olarak çizilir
            plt.scatter(gen_wps_lon, gen_wps_lat, c='green', marker='o', s=15, label='Virtual Waypointler')

        for i, zone in enumerate(danger_zones):
            center_lat, center_lon = zone['center']
            radius_km = zone['radius'] / 1000.0
            circle_lats, circle_lons = [], []
            for angle in range(361):
                point = distance(kilometers=radius_km).destination(point=(center_lat, center_lon), bearing=angle)
                circle_lats.append(point.latitude)
                circle_lons.append(point.longitude)
            
            # Sadece ilk daire için etiket göster, diğerleri için gösterme
            label = 'Tehlikeli Bölge' if i == 0 else ""
            plt.plot(circle_lons, circle_lats, color='black', linestyle='--', label=label)
        
        plt.legend(loc = 'lower right', fontsize = 7)
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')

        plt.subplot(2, 2, 2)
        plt.plot(self.data['time'], self.data['alt'])
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (ft)")
        plt.title("Altitude Profile")
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(self.data['time'], self.data['speed'], color='blue')
        plt.axhline(y=85, color='r', linestyle='--', label='TAKE-OFF Speed')
        plt.axhline(y=100, color='g', linestyle='--', label='CLIMB Speed')
        plt.xlabel("Time (s)")
        plt.ylabel("Airspeed (kts)")
        plt.title("Speed Profile")
        plt.grid(True)
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.data['time'], self.data['pitch'], label="Pitch (°)")
        plt.plot(self.data['time'], self.data['roll'], label="Roll (°)")
        plt.plot(self.data['time'], self.data['yaw'], label="Yaw (°)")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title("Aircraft Attitude")
        plt.grid(True)
        plt.legend()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    def lla_to_enu(self, lat_log, lon_log, alt_log):
        a = 6378137.0
        f = 1 / 298.257223563
        e_sq = f * (2 - f)
        lat_rad, lon_rad = np.deg2rad(np.array(lat_log)), np.deg2rad(np.array(lon_log))
        alt_m = np.array(alt_log) * 0.3048
        N = a / np.sqrt(1 - e_sq * np.sin(lat_rad)**2)
        ecef_x = (N + alt_m) * np.cos(lat_rad) * np.cos(lon_rad)
        ecef_y = (N + alt_m) * np.cos(lat_rad) * np.sin(lon_rad)
        ecef_z = (N * (1 - e_sq) + alt_m) * np.sin(lat_rad)
        ref_lat_rad, ref_lon_rad = lat_rad[0], lon_rad[0]
        R = np.array([
            [-np.sin(ref_lon_rad), np.cos(ref_lon_rad), 0],
            [-np.sin(ref_lat_rad)*np.cos(ref_lon_rad), -np.sin(ref_lat_rad)*np.sin(ref_lon_rad), np.cos(ref_lat_rad)],
            [np.cos(ref_lat_rad)*np.cos(ref_lon_rad), np.cos(ref_lat_rad)*np.sin(ref_lon_rad), np.sin(ref_lat_rad)]
        ])
        delta_x, delta_y, delta_z = ecef_x - ecef_x[0], ecef_y - ecef_y[0], ecef_z - ecef_z[0]
        enu = np.dot(R, np.vstack((delta_x, delta_y, delta_z)))
        return enu[0], enu[1], enu[2]

    def plot_3d_path(self):
        east, north, up = self.lla_to_enu(self.data['lat'], self.data['lon'], self.data['alt'])
        fig = plt.figure("3D Flight Path")
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(east, north, up, color='blue', label='Flight Path')
        ax.scatter(east[0], north[0], up[0], color='green', s=100, label='Start')
        ax.scatter(east[-1], north[-1], up[-1], color='red', s=100, label='End')
        ax.set_title('3D Flight Path (ENU Coordinates)', fontsize=16)
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_zlabel('Up (m)')
        ax.view_init(elev=30., azim=50)
        ax.legend()
        ax.grid(True)

    def show_plots(self):
        plt.show()