import tkinter as tk

# ==========================================
# 2. LA VUE (Interface Graphique pure - Passive)
# ==========================================
class RobotDirectionGUI(tk.Tk):
    def __init__(self, on_direction_click_callback):
        super().__init__()
        self.title("Contrôle Robot - MVC Strict")
        self.geometry("300x200")
        
        # La vue reçoit une fonction de rappel, elle ignore qui la traite
        self.trigger_movement = on_direction_click_callback
        
        # Création et configuration des composants graphiques
        btn_up = tk.Button(self, text="▲ Avant", command=lambda: self.trigger_movement("forward"))
        btn_down = tk.Button(self, text="▼ Arrière", command=lambda: self.trigger_movement("backward"))
        btn_left = tk.Button(self, text="◀ Gauche", command=lambda: self.trigger_movement("left"))
        btn_right = tk.Button(self, text="▶ Droite", command=lambda: self.trigger_movement("right"))
        btn_stop = tk.Button(self, text="■ STOP", command=lambda: self.trigger_movement("stop"), bg="red", fg="white")
        
        # Placement strict via la grille
        btn_up.grid(row=0, column=1, pady=5)
        btn_left.grid(row=1, column=0, padx=5)
        btn_stop.grid(row=1, column=1, padx=5)
        btn_right.grid(row=1, column=2, padx=5)
        btn_down.grid(row=2, column=1, pady=5)
