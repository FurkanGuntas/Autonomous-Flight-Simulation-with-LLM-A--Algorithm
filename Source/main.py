from flight_simulation import FlightSimulator
from llm import StrategicAI

# GÖREV TANIMI

'''
mission_briefing = "Urgent Mission: The mission is time-critical and requires immediate transit " \
"to the area via the fastest possible route. The UAV must perform target identification with " \
"the sharpest and most abrupt maneuvers possible. The success of the mission depends on its " \
"ability to move in narrow spaces following an unpredictable flight path and to perform sudden " \
"maneuvers"
'''

mission_briefing = "Calm Mission: The top priority is to avoid known hazardous airspace " \
"along the route as much as possible. There is no time constraint to complete the mission, " \
"so the safest route should be chosen. Because a large area will already be protected from " \
"hazards, the escape route does not need to be meticulously planned. Gentle turns are sufficient " \
"for stability."


# SABİT PARAMETRELER
flight_parameters = {
    'sim_path': r'C:\Users\PC_6273__\Desktop\JSBSimData\jsbsim-master',
    'model': 'c310',
    'dt': 0.01,
    't_end': 2000.0,
    'initial_conditions': {'lat': 41.2753, 
                           'lon': 28.7519, 
                           'alt': 300, 
                           'h_agl': 3.32},

    'waypoints': [(41.2, 29.15), 
                  (41.0054, 28.9), 
                  (41.2753, 28.7519)],
                  
    's_turn_wps': [(41.14, 29.16), 
                   (41.20, 29.22), 
                   (41.13, 29.25)],
                   
    'dangerous_zones': [{'center': (41.32, 28.9), 'radius': 3500}, 
                        {'center': (41.19, 29.03), 'radius': 4500}],

    'wp_threshold_m': 150,
    'descent_start_m': 8700
}

if __name__ == "__main__":
    print("==================================================")
    print("       MUHARİP İHA GÖREV SİMÜLASYONU BAŞLIYOR      ")
    print("==================================================")

    # STRATEJİK PLANLAMA 
    print("\n--- AŞAMA 1: STRATEJİK PLANLAMA (LLM) ---")
    ai_commander = StrategicAI(model_name='gemma3:4b')
    print(f"Model Yüklendi: '{ai_commander.model_name}'")
    print("Görev tanımı analiz için LLM'e gönderiliyor...")
    
    llm_decision = ai_commander.select_flight_profile(mission_briefing)
    #llm_reasoning = llm_decision.get('reasoning', 'Gerekçe belirtilmedi.')
    # LLM'den gelen "düz" parametreleri al
    flat_params = llm_decision['parameters']

    # Düz parametreleri, simülatörün beklediği iç içe geçmiş (nested) yapıya dönüştür.
    dynamic_planner_params = {
        "safety_distance_km": flat_params['safety_distance_km'],
        "turn_penalty_config": {
            "sharp_turn_factor": flat_params['sharp_turn_factor'],
            "moderate_turn_factor": flat_params['moderate_turn_factor']
        },
        "step_distance_km": flat_params['step_distance_km']
    }

   # Raporun daha okunaklı olması için görev tanımını da ekliyoruz.
    llm_report_text = (
        "\n=================================================="
        "\n           LLM STRATEJİK KARAR RAPORU          "
        "\n=================================================="
        f"\n\n[GÖREV TANIMI]\n  > {mission_briefing.replace(chr(92), '').replace(' ', '', 1)}\n"
        #f"\n[LLM GEREKÇESİ]\n  > {llm_reasoning}\n"
        "\n[ÜRETİLEN UÇUŞ PARAMETRELERİ]"
        f"\n - Güvenlik Mesafesi (safety_distance): {dynamic_planner_params['safety_distance_km']} km"
        f"\n - Dönüş Cezaları (turn_penalty): Keskin={dynamic_planner_params['turn_penalty_config']['sharp_turn_factor']}, Orta={dynamic_planner_params['turn_penalty_config']['moderate_turn_factor']}"
        f"\n - Adım Aralığı (step_distance): {dynamic_planner_params['step_distance_km']} km"
        "\n\n=================================================="
    )
    print("Stratejik karar alındı ve parametreler belirlendi. Uçuş operasyonuna geçiliyor.")

    # SİMÜLATÖR KURULUMU
    print("\n--- AŞAMA 2: SİMÜLATÖR KURULUMU ---")
    simulator = FlightSimulator(flight_parameters, dynamic_planner_params)
    print("JSBSim Uçuş Modeli başarıyla yüklendi.")
    print("Rota Planlayıcı (PathPlanner) LLM tarafından üretilen parametrelerle başlatıldı.")

    # UÇUŞ OPERASYONU
    print("\n--- AŞAMA 3: UÇUŞ OPERASYONU ---")
    print("Simülasyon başlatılıyor...\n")
    
    simulator.run()
 
    print(llm_report_text)