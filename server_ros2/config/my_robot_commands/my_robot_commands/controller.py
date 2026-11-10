class TeleopController:
    """
    Contrôleur (MVC) : reçoit les événements de la Vue,
    applique la logique nécessaire, et appelle le Modèle.
    """

    def __init__(self, model):
        self._model = model

    def handle_send_text(self, texte: str) -> None:
        self._model.send_command(texte)
