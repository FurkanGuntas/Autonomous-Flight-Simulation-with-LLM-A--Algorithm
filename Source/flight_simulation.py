import jsbsim
from geopy.distance import geodesic

from path_planner import PathPlanner
from data import FlightData, FlightDataVisualizer

# SINIF 4: ANA SİMÜLATÖR 
class FlightSimulator:

    def __init__(self, config, planner_params):
        self.config = config
        self.fdm = self.jsbsim_setup()
        self.logger = FlightData()
        self.danger_acknowledged = False
        
        # PathPlanner, LLM'den gelen dinamik parametrelerle başlatılıyor
        self.planner = PathPlanner(
            config['dangerous_zones'],
            planner_params['safety_distance_km'],
            planner_params['turn_penalty_config'],
            planner_params['step_distance_km']
        )
        
        self.current_phase = "GROUND"
        self.flight_mode = "NORMAL_SEYIR"
        self.waypoints = config['waypoints'].copy()
        self.wp_index = 0
        self.maneuver_wps = []
        self.maneuver_wp_index = 0
        self.original_wp_index_backup = 0
        self.s_turn_initiated = False
        self.all_virtual_wps = []


    def jsbsim_setup(self):

        fdm = jsbsim.FGFDMExec(self.config['sim_path'])
        fdm.set_debug_level(0)
        fdm.set_dt(self.config['dt'])
        fdm.load_model(self.config['model'])
        ic = self.config['initial_conditions']
        fdm.set_property_value('ic/lat-gc-deg', ic['lat'])
        fdm.set_property_value('ic/long-gc-deg', ic['lon'])
        fdm.set_property_value('ic/h-sl-ft', ic['alt'])
        fdm.set_property_value('ic/h-agl-ft', ic['h_agl'])
        fdm.run_ic()
        fdm.set_property_value('fcs/mixture-cmd-norm[0]', 1.0)
        fdm.set_property_value('fcs/mixture-cmd-norm[1]', 1.0)
        fdm.set_property_value('propulsion/magneto_cmd', 3.0)
        fdm.set_property_value('propulsion/starter_cmd', 1.0)
        fdm.set_property_value('fcs/center-brake-cmd-norm', 0)
        return fdm

    def update_autopilot_heading(self, target_wp):

        lat = self.fdm.get_property_value('position/lat-gc-deg')
        lon = self.fdm.get_property_value('position/long-gc-deg')
        target_heading = self.planner.calculate_bearing((lat, lon), target_wp)
        self.fdm.set_property_value('ap/heading_setpoint', target_heading)
        self.fdm.set_property_value('ap/heading_hold', 1)

    def flight_phases(self):

        alt = self.fdm.get_property_value('position/h-sl-ft')
        speed = self.fdm.get_property_value('velocities/vc-kts')
        t = self.fdm.get_sim_time()
        if self.current_phase == "GROUND":
            self.fdm.set_property_value('fcs/throttle-cmd-norm[0]', 1.0)
            self.fdm.set_property_value('fcs/throttle-cmd-norm[1]', 1.0)
            self.fdm.set_property_value('ap/heading_hold', 1) 
            if speed >= 100:
                self.current_phase = "ROTATE"
                print(f"--> ROTATE (t={t:.2f}s)")
        elif self.current_phase == "ROTATE":
            self.fdm.set_property_value('fcs/elevator-cmd-norm', -0.25)
            agl_ft = self.fdm.get_property_value('position/h-agl-ft')
            if agl_ft > 15: 
                self.current_phase = "CLIMB"
                print(f"--> CLIMB (t={t:.2f}s)")
                self.fdm.set_property_value('fcs/elevator-cmd-norm', 0)
                self.fdm.set_property_value('gear/gear-cmd-norm', 0)
        elif self.current_phase == "CLIMB":
            self.fdm.set_property_value('fcs/throttle-cmd-norm[0]', 1.0)
            self.fdm.set_property_value('fcs/throttle-cmd-norm[1]', 1.0)
            self.fdm.set_property_value('ap/altitude_setpoint', 2000)
            if alt >= 2000:
                self.current_phase = "CRUISE"
                print(f"--> CRUISE (t={t:.2f}s)")
        elif self.current_phase == "CRUISE":
            self.a_star_integrated_cruise()
        elif self.current_phase == "DESCENT":
            self.fdm.set_property_value('fcs/throttle-cmd-norm[0]', 0.5)
            self.fdm.set_property_value('fcs/throttle-cmd-norm[1]', 0.5)
            self.fdm.set_property_value('ap/altitude_setpoint', 450)
            if alt <= 450:
                self.current_phase = "LANDING"
                print(f"--> LANDING (t={t:.2f}s)")
        elif self.current_phase == "LANDING":
            self.fdm.set_property_value('fcs/throttle-cmd-norm[0]', 0.0)
            self.fdm.set_property_value('fcs/throttle-cmd-norm[1]', 0.0)
            self.fdm.set_property_value('fcs/elevator-cmd-norm', -0.1)
            agl_ft = self.fdm.get_property_value('position/h-agl-ft')
            if agl_ft <= self.config['initial_conditions']['h_agl']:
                self.current_phase = "GROUND_ROLL"
                print(f"--> GROUND ROLL (t={t:.2f}s)")
                self.fdm.set_property_value('gear/gear-cmd-norm', 1)
        elif self.current_phase == "GROUND_ROLL":      
            self.fdm.set_property_value('ap/attitude_hold', 0)
            self.fdm.set_property_value('fcs/center-brake-cmd-norm', 1.0)
            self.fdm.set_property_value('fcs/left-brake-cmd-norm', 1.0)
            self.fdm.set_property_value('fcs/right-brake-cmd-norm', 1.0)
            if speed <= .01:
                self.fdm.set_property_value('propulsion/engine/set-running', 0)
                raise StopIteration("Simülasyon tamamlandı.")

    def a_star_integrated_cruise(self):
        target_wp = None
        lat = self.fdm.get_property_value('position/lat-gc-deg')
        lon = self.fdm.get_property_value('position/long-gc-deg')
        current_pos = (lat, lon)
        
        if self.flight_mode == "NORMAL_SEYIR":
            if self.wp_index < len(self.waypoints):
                target_wp = self.waypoints[self.wp_index]
                path_is_dangerous = self.planner.is_path_in_danger(current_pos, target_wp)
                
                if path_is_dangerous:
                    if not self.danger_acknowledged:
                        print("UYARI: Rota tehlikeli bölgeye yakın! Kaçınma manevrası planlanacak...")
                        self.danger_acknowledged = True

                        virtual_wps = self.planner.find_safe_path(current_pos, target_wp)
                        if virtual_wps and len(virtual_wps) > 1:
                            print(f"Başarılı: {len(virtual_wps)} adımlık kaçış rotası oluşturuldu.")
                            self.maneuver_wps = virtual_wps[1:]
                            self.all_virtual_wps.extend(self.maneuver_wps) 
                            self.maneuver_wp_index = 0
                            self.original_wp_index_backup = self.wp_index
                            self.flight_mode = "KACIS_MANEVRASI"
                            print("--> KACIS_MANEVRASI moduna geçildi.")
                        else:
                            print("UYARI: A* güvenli bir rota bulamadı! Mevcut rotada devam ediliyor.")
                elif self.danger_acknowledged:
                    # Tehlike durumu ortadan kalktıysa, bayrağı sıfırla
                    print("BİLGİ: Tehlikeli rota temizlendi, normal seyre dönülüyor.")
                    self.danger_acknowledged = False
        
        elif self.flight_mode == "KACIS_MANEVRASI":
            if self.maneuver_wp_index < len(self.maneuver_wps):
                target_wp = self.maneuver_wps[self.maneuver_wp_index]
                if geodesic(current_pos, target_wp).meters < self.config['wp_threshold_m']:
                    print(f"Virtual waypoint {self.maneuver_wp_index + 1}/{len(self.maneuver_wps)} tamamlandı.")
                    self.maneuver_wp_index += 1
            else:
                print("Kaçınma manevrası tamamlandı. Orijinal göreve dönülüyor.")
                self.flight_mode = "NORMAL_SEYIR"
                self.wp_index = self.original_wp_index_backup
                if self.wp_index < len(self.waypoints):
                    target_wp = self.waypoints[self.wp_index]
        
        if target_wp:
            self.update_autopilot_heading(target_wp)
        if self.flight_mode == "NORMAL_SEYIR" and self.wp_index < len(self.waypoints):
            if geodesic(current_pos, self.waypoints[self.wp_index]).meters < self.config['wp_threshold_m']:
                if self.wp_index == 0 and not self.s_turn_initiated:
                    print(f"İlk waypoint'e ulaşıldı. Manuel S-Dönüşü başlatılıyor...")
                    self.s_turn_initiated = True
                    self.waypoints[1:1] = self.config['s_turn_wps']
                
                print(f"Ana waypoint {self.wp_index + 1} tamamlandı.")
                self.wp_index += 1
        
        self.fdm.set_property_value('fcs/throttle-cmd-norm[0]', .87)
        self.fdm.set_property_value('fcs/throttle-cmd-norm[1]', .87)
        self.fdm.set_property_value('ap/altitude_hold', 1)
        if self.flight_mode == "NORMAL_SEYIR" and self.wp_index == len(self.waypoints) - 1:
            if geodesic(current_pos, self.waypoints[self.wp_index]).meters < self.config['descent_start_m']:
                self.current_phase = "DESCENT"
                print(f"Son hedefe yakın. --> DESCENT (t={self.fdm.get_sim_time():.2f}s)")

    def run(self):

        print("Simülasyon başlatılıyor...")
        last_print_time = -1
        try:
            while self.fdm.get_sim_time() < self.config['t_end']:
                self.fdm.run()
                self.flight_phases()
                self.logger.log_state(self.fdm)
                t = self.fdm.get_sim_time()
                if int(t) > last_print_time:
                    alt = self.fdm.get_property_value('position/h-sl-ft')
                    speed = self.fdm.get_property_value('velocities/vc-kts')
                    #print(f"t={t:.0f}s, Alt={alt:.2f} ft, speed={speed:.2f} kts, Phase={self.current_phase}, Mode={self.flight_mode}")
                    last_print_time = int(t)
        except StopIteration as e:
            print(e)
        finally:
            self.post_process()

    def post_process(self):
        print("Simülasyon tamamlandı. Grafikler oluşturuluyor...")
        visualizer = FlightDataVisualizer(self.logger)
        visualizer.plot_2d_data(self.waypoints, self.all_virtual_wps, self.config['dangerous_zones'])
        visualizer.plot_3d_path()
        visualizer.show_plots()