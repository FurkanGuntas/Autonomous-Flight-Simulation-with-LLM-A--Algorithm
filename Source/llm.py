import ollama
import json

class StrategicAI:
    def __init__(self, model_name='gemma3:4b'):
        self.model_name = model_name


    def select_flight_profile(self, mission_brief):
        prompt = f"""
        You are a mission planning AI for a combat UAV. Your task is to analyze the following mission briefing and determine the most" \
        "suitable flight parameters for the A* path planner. You must provide your decisions in JSON format.

        Parameters and Their Meanings:
        - "safety_distance_km": How far to stay away from dangerous zones. High values (2.0-6.0) are for cover missions and missions" \
        "where we need to stay as far away from dangerous zones as possible, while low values (0.1-1.0) are for close engagement." \
        
        - "sharp_turn_factor": The penalty factor for sharp turns. High values (30-60) enforce smooth routes (low maneuverability). 
        "Low values (0-10) allow for aggressive and agile maneuvers (high maneuverability).

        - "moderate_turn_factor": The penalty factor for medium-level turns (1-4).

        - "step_distance_km": The step interval for the A* algorithm. High values (between 4.5-7 km) are used for fast and efficient " \
        "planning **if already far from dangers or if the route is in open terrain**. Low values (between 2.5-4.5 km) are used for " \
        "precise maneuvers **when close to dangers and in tight spaces**.
        
        Mission Briefing:
        "{mission_brief}"


        {{
            "parameters": 
            {{
            "safety_distance_km": float,
            "sharp_turn_factor": int,
            "moderate_turn_factor": int,
            "step_distance_km": float
            }}
        }}
        """

        try:
            response = ollama.chat(model=self.model_name, messages=[{'role': 'user', 'content': prompt}])
            llm_response_text = response['message']['content'].strip()
            
            if llm_response_text.startswith("```json"):
                llm_response_text = llm_response_text.strip("```json").strip()

            decision_data = json.loads(llm_response_text)
            
            # Artık tüm karar verisini (gerekçe dahil) döndürelim
            return decision_data

        except Exception as e:
            print(f"LLM'den yanıt alınamadı. Hata: {e}. Varsayılan güvenli parametreler kullanılıyor.")
            return e