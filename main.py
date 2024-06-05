import tkinter as tk
import math

class RobotSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Simulation de Robot")

        # Création du canevas
        self.canvas = tk.Canvas(root, width=1520, height=938, bg="white")
        self.canvas.pack(side=tk.LEFT)

        # Variables pour les paramètres
        self.angle = tk.DoubleVar()
        self.angle.set(90)  # Angle initial de 90 degrés

        self.x_position = tk.DoubleVar()
        self.x_position.set(760)  # Position initiale au centre de l'écran

        self.mass = tk.DoubleVar()
        self.mass.set(10.0)  # Masse par défaut
        self.height = tk.DoubleVar()
        self.height.set(10.0)  # Hauteur par défaut
        self.length = tk.DoubleVar()
        self.length.set(5.0)  # Longueur par défaut

        # Variable temps (en secondes)
        self.time = 0
        self.dt = 0.1  # Intervalle de temps en secondes

        # Gravité
        self.g = 9.81  # Accélération due à la gravité (m/s^2)

        # Inertie en x
        self.inertia_x = 0.1  # Valeur par défaut de l'inertie en x

        # Flag pour suivre l'état de la simulation
        self.running = False

        # Création des widgets
        self.create_widgets()

    def create_widgets(self):
        # Frame pour les paramètres
        params_frame = tk.Frame(self.root)
        params_frame.pack(side=tk.RIGHT, padx=20)

        # Widgets pour les paramètres
        angle_label = tk.Label(params_frame, text="Angle de rotation (°):")
        angle_label.grid(row=0, column=0, pady=5)
        
        self.angle_entry = tk.Entry(params_frame, textvariable=self.angle)
        self.angle_entry.grid(row=0, column=1, pady=5)

        mass_label = tk.Label(params_frame, text="Masse (kg):")
        mass_label.grid(row=1, column=0, pady=5)
        
        self.mass_entry = tk.Entry(params_frame, textvariable=self.mass)
        self.mass_entry.grid(row=1, column=1, pady=5)

        height_label = tk.Label(params_frame, text="Hauteur (m):")
        height_label.grid(row=2, column=0, pady=5)
        
        self.height_entry = tk.Entry(params_frame, textvariable=self.height)
        self.height_entry.grid(row=2, column=1, pady=5)

        length_label = tk.Label(params_frame, text="Longueur (m):")
        length_label.grid(row=3, column=0, pady=5)
        
        self.length_entry = tk.Entry(params_frame, textvariable=self.length)
        self.length_entry.grid(row=3, column=1, pady=5)

        inertia_label = tk.Label(params_frame, text="Inertie en x:")
        inertia_label.grid(row=4, column=0, pady=5)
        
        self.inertia_entry = tk.Entry(params_frame, textvariable=tk.DoubleVar(value=self.inertia_x))
        self.inertia_entry.grid(row=4, column=1, pady=5)

        self.update_button = tk.Button(params_frame, text="Mettre à jour", command=self.start_simulation)
        self.update_button.grid(row=5, columnspan=2, pady=10)

        self.stop_button = tk.Button(params_frame, text="Arrêter", command=self.stop_simulation)
        self.stop_button.grid(row=6, columnspan=2, pady=10)

    def start_simulation(self):
        if not self.running:
            self.running = True
            self.update_robot()

    def stop_simulation(self):
        self.running = False

    def update_robot(self):
        if not self.running:
            return

        self.canvas.delete("all")
        x = self.x_position.get()
        y = 500  # Position fixe en y pour la base du robot
        angle = math.radians(self.angle.get())
        length = 100  # Longueur du robot pour la représentation graphique

        x_end = x + length * math.cos(angle)
        y_end = y - length * math.sin(angle)

        # Dessiner le robot
        self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill="black")
        self.canvas.create_line(x, y, x_end, y_end, fill="blue", width=5)

        # Calcul du moment d'inertie (I)
        m = self.mass.get()
        h = self.height.get()
        l = self.length.get()
        I = (1/12) * m * (h**2 + l**2)

        # Calcul de la force de gravité agissant sur le centre de masse
        F_gravity = m * self.g
        L = length / 2  # Distance du pivot au centre de masse
        torque_gravity = F_gravity * L * math.sin(angle)  # Couple dû à la gravité

        # Calcul de l'accélération angulaire (α) due à la gravité
        alpha = torque_gravity / I

        # Déterminer la direction de la chute
        if self.angle.get() < 90:
            alpha = -abs(alpha)  # Tendre vers 0°
        elif self.angle.get() > 90:
            alpha = abs(alpha)  # Tendre vers 180°
        else:
            if self.inertia_x > 0:
                alpha = abs(alpha)  # Tendre vers 180°
            else:
                alpha = -abs(alpha)  # Tendre vers 0°

        # Mise à jour de l'angle du robot
        omega = alpha * self.dt  # Vitesse angulaire
        new_angle = self.angle.get() + math.degrees(omega * self.dt)

        self.angle.set(new_angle)

        # Affichage des valeurs dans la console
        print(f"--- Temps: {self.time} ---")
        print(f"--- Angle: {self.angle.get()} ---")
        print(f"--- Force de gravité: {F_gravity} ---")
        print(f"--- Couple dû à la gravité: {torque_gravity} ---")
        print(f"--- Accélération angulaire: {alpha} ---")
        print(f"--- Nouvel angle: {new_angle} ---")
        print(f"--- Moment d'inertie :, {I} ---\n")

        # Mise à jour du temps
        self.time += self.dt

        # Rappeler la méthode update_robot après un certain délai (pour simuler le passage du temps)
        self.root.after(int(self.dt * 1000), self.update_robot)

if __name__ == "__main__":
    root = tk.Tk()
    simulator = RobotSimulator(root)
    root.mainloop()
