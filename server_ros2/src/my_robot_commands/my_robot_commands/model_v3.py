class RobotCommandModel:
    def __init__(self, max_linear_speed=0.2, max_angular_speed=0.5, deadzone=0.1):
        # Limites physiques du robot
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed
        self._deadzone = deadzone
        
        # État interne
        self.linear_x = 0.0
        self.angular_z = 0.0

    def compute_from_axes(self, axis_linear, axis_angular):
        """
        Calcule les vitesses cibles proportionnelles à partir des axes du joystick (-1.0 à 1.0).
        Gère la zone morte et applique les limites de vitesse maximales.
        """
        # Application de la zone morte (si la valeur est trop faible, on force à 0)
        linear_input = axis_linear if abs(axis_linear) > self._deadzone else 0.0
        angular_input = axis_angular if abs(axis_angular) > self._deadzone else 0.0
        
        # Règle métier : calcul proportionnel de la vitesse finale
        self.linear_x = linear_input * self._max_linear
        self.angular_z = angular_input * self._max_angular
        
        return self.linear_x, self.angular_z

    def compute_from_direction(self, direction):
        """
        Conserve la compatibilité avec l'interface graphique (boutons Tout ou Rien).
        Traduit une direction discrète en valeurs d'axes virtuelles (-1.0, 0.0, 1.0).
        """
        if direction == "forward":   return self.compute_from_axes(1.0, 0.0)
        if direction == "backward":  return self.compute_from_axes(-1.0, 0.0)
        if direction == "left":      return self.compute_from_axes(0.0, 1.0)
        if direction == "right":     return self.compute_from_axes(0.0, -1.0)
        if direction == "stop":      return self.compute_from_axes(0.0, 0.0)
        raise ValueError(f"Direction inconnue : {direction}")

