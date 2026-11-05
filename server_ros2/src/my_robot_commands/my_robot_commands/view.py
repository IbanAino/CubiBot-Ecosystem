import tkinter as tk
from tkinter import ttk


class TeleopView(tk.Tk):
    """
    Vue (MVC) : construit l'interface et délègue chaque action
    au contrôleur via des callbacks. Ne parle jamais directement à ROS2.
    """

    def __init__(self, on_send_text):
        super().__init__()
        self.title('Téléopération du robot')
        self.geometry('720x200')

        self._on_send_text = on_send_text

        self._build_shortcuts_frame()
        self._build_manual_frame()

    def _build_shortcuts_frame(self):
        shortcuts_frame = ttk.Frame(self, padding=10)
        shortcuts_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(shortcuts_frame, text='Raccourcis :').grid(
            row=0, column=0, columnspan=3, sticky='w', pady=(0, 5))

        boutons = [
            ('Avancer (CMD,0.2,0.0)', 'CMD,0.2,0.0'),
            ('Reculer (CMD,-0.2,0.0)', 'CMD,-0.2,0.0'),
            ('Tourner G (CMD,0.0,1.0)', 'CMD,0.0,1.0'),
            ('Tourner D (CMD,0.0,-1.0)', 'CMD,0.0,-1.0'),
            ('Stop', 'STOP'),
            ('RESET', 'RESET'),
        ]

        colonnes_max = 3
        for index, (texte_bouton, commande) in enumerate(boutons):
            ligne = 1 + index // colonnes_max
            colonne = index % colonnes_max
            ttk.Button(
                shortcuts_frame,
                text=texte_bouton,
                width=22,
                command=lambda cmd=commande: self._on_send_text(cmd)
            ).grid(row=ligne, column=colonne, padx=3, pady=3)

    def _build_manual_frame(self):
        manual_frame = ttk.Frame(self, padding=10)
        manual_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(manual_frame, text='Commande manuelle :').pack(side=tk.LEFT, padx=(0, 5))

        self.entry_var = tk.StringVar()
        entry = ttk.Entry(manual_frame, textvariable=self.entry_var, width=30)
        entry.pack(side=tk.LEFT, padx=2)
        entry.bind('<Return>', lambda event: self._handle_manual_send())

        ttk.Button(manual_frame, text='Envoyer',
                   command=self._handle_manual_send).pack(side=tk.LEFT, padx=2)

    def _handle_manual_send(self):
        texte = self.entry_var.get().strip()
        if texte:
            self._on_send_text(texte)
            self.entry_var.set('')
