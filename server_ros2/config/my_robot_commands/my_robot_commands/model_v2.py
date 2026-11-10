# ==========================================
# 1. LE MODÈLE (Logique métier pure - Zéro ROS2, Zéro GUI)
# ==========================================
class RobotCommandModel:
    def __init__(self, max_linear_speed=0.2, max_angular_speed=0.5):
        # Configuration métier du robot
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed
        
        # État interne de la commande actuelle
        self.linear_x = 0.0
        self.angular_z = 0.0

    def compute_target_speeds(self, direction):
        """
        Calcule les vitesses cibles selon la direction demandée.
        C'est ici que réside la 'physique' ou les règles métier du robot.
        """
        if direction == "forward":
            self.linear_x = self._max_linear
            self.angular_z = 0.0
        elif direction == "backward":
            self.linear_x = -self._max_linear
            self.angular_z = 0.0
        elif direction == "left":
            self.linear_x = 0.0
            self.angular_z = self._max_angular
        elif direction == "right":
            self.linear_x = 0.0
            self.angular_z = -self._max_angular
        elif direction == "stop":
            self.linear_x = 0.0
            self.angular_z = 0.0
        else:
            raise ValueError(f"Direction inconnue : {direction}")
            
        return self.linear_x, self.angular_z
