import pygame

class JoystickInput:
    def __init__(self, on_axis_change_callback):
        """
        Classe d'entrée pure (Périphérique).
        Elle capture les mouvements physiques et notifie le contrôleur.
        """
        # Initialisation silencieuse des modules nécessaires de Pygame
        pygame.init()
        pygame.joystick.init()
        
        self.joystick = None
        self.trigger_axis_change = on_axis_change_callback
        
        # Détection du matériel connecté
        if pygame.joystick.get_count() > 0:
            # On prend la première manette disponible (index 0)
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"[Joystick] Connecté avec succès : {self.joystick.get_name()}")
        else:
            print("[Joystick] ATTENTION : Aucun périphérique détecté.")

        # Historique des anciennes valeurs pour éviter de saturer le réseau ROS2
        # si les sticks ne bougent pas
        self.last_linear = 0.0
        self.last_angular = 0.0

    def poll_events(self):
        """
        Méthode appelée régulièrement par le timer du contrôleur (ex: à 20Hz).
        Elle lit les axes et transmet les floats continus.
        """
        if not self.joystick:
            return

        # Pumper/Vider la pile d'événements SDL (requis pour mettre à jour les valeurs d'axes)
        pygame.event.pump()
        
        # --- MAPPAGE STANDARD (Style Xbox/PS4) ---
        # Axe 1 : Stick Gauche Vertical (Avant/Arrière)
        # Axe 0 : Stick Gauche Horizontal (Rotation Gauche/Droite)
        # Note : En standard, pousser le stick vers l'avant renvoie du négatif (-1.0), 
        # on ajoute un signe moins (-) pour avoir du positif (1.0) en marche avant pour le robot.
        raw_linear = -self.joystick.get_axis(1)
        raw_angular = -self.joystick.get_axis(0)

        # Arrondi de sécurité à 3 décimales pour éviter le bruit analogique
        axis_linear = round(raw_linear, 3)
        axis_angular = round(raw_angular, 3)

        # Optimisation : On ne notifie le contrôleur QUE si la valeur a changé
        if axis_linear != self.last_linear or axis_angular != self.last_angular:
            self.last_linear = axis_linear
            self.last_angular = axis_angular
            
            # Envoi direct des valeurs float au contrôleur
            self.trigger_axis_change(axis_linear, axis_angular)

