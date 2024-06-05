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

        # Création des widgets
        self.create_widgets()
        self.update_robot()

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

        update_button = tk.Button(params_frame, text="Mettre à jour", command=self.update_robot)
        update_button.grid(row=4, columnspan=2, pady=10)

    def update_robot(self, event=None):
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

        # Mise à jour de l'angle du robot
        omega = alpha * self.dt  # Vitesse angulaire
        new_angle = math.degrees(angle) + math.degrees(omega * self.dt)

        self.angle.set(new_angle)

        # Affichage des valeurs dans la console
        print("Temps:", self.time)
        print("Moment d'inertie (I) :", I)
        print("Force de gravité (F_gravity) :", F_gravity)
        print("Couple dû à la gravité (torque_gravity) :", torque_gravity)
        print("Accélération angulaire (α) :", alpha)
        print("Nouvel angle (°) :", new_angle)

        # Mise à jour du temps
        self.time += self.dt

        # Rappeler la méthode update_robot après un certain délai (pour simuler le passage du temps)
        self.root.after(int(self.dt * 1000), self.update_robot)

if __name__ == "__main__":
    root = tk.Tk()
    simulator = RobotSimulator(root)
    root.mainloop()
