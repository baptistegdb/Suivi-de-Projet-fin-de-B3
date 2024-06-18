import tkinter as tk
import math
import numpy as np
import matplotlib.pyplot as plt

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
        self.mass.set(0.2)  # Masse par défaut
        self.height = tk.DoubleVar()
        self.height.set(0.01)  # Hauteur par défaut
        self.length = tk.DoubleVar()
        self.length.set(0.005)  # Longueur par défaut

        # Variable temps (en secondes)
        self.time = 0
        self.dt = 0.1  # Intervalle de temps en secondes

        # Gravité
        self.g = 9.81  # Accélération due à la gravité (m/s^2)

        # Inertie en x
        self.inertia_x = 0.1  # Valeur par défaut de l'inertie en x

        # PID constants
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05

        # PID variables
        self.previous_error = 0
        self.integral = 0

        # Constante du moteur
        self.k_v = 1.0

        # Flag pour suivre l'état de la simulation
        self.running = False

        # Listes pour stocker les valeurs pour les graphiques
        self.time_data = []
        self.angle_data = []
        self.F_data = []
        self.P_data = []
        self.I_data = []
        self.D_data = []

        # Temps durant lequel l'angle est stable
        self.stable_time = 0

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
            self.calibrate_pid()  # Calibrate PID before starting simulation
            self.update_robot()

    def stop_simulation(self):
        self.running = False
        self.plot_pid()

    def calibrate_pid(self):
        best_params = None
        best_error = float('inf')

        for Kp in np.linspace(0.1, 5.0, 10):
            for Ki in np.linspace(0.0, 1.0, 10):
                for Kd in np.linspace(0.0, 1.0, 10):
                    error = self.simulate_pid(Kp, Ki, Kd)
                    if error < best_error:
                        best_error = error
                        best_params = (Kp, Ki, Kd)

        self.Kp, self.Ki, self.Kd = best_params
        print(f"Calibrated PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}")

    def simulate_pid(self, Kp, Ki, Kd):
        error_sum = 0
        temp_angle = 90
        temp_integral = 0
        temp_previous_error = 0

        for _ in range(100):
            error = 90 - temp_angle
            temp_integral += error * self.dt
            derivative = (error - temp_previous_error) / self.dt
            F = Kp * error + Ki * temp_integral + Kd * derivative
            temp_previous_error = error

            L = self.length.get() / 2  # Distance du pivot au centre de masse
            I = (1/12) * self.mass.get() * (self.height.get()**2 + self.length.get()**2)
            alpha = (F * L) / I

            omega = alpha * self.dt  # Vitesse angulaire
            temp_angle += omega * self.dt
            temp_angle %= 360  # Normalisation de l'angle dans la plage 0-360 degrés

            error_sum += abs(error)

        return error_sum

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

        # PID controller
        error = 90 - self.angle.get()
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        F = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error

        # Calcul de l'accélération angulaire (α) due à la force PID
        L = self.length.get() / 2  # Distance du pivot au centre de masse
        alpha = (F * L) / I

        # Mise à jour de l'angle du robot
        omega = alpha * self.dt  # Vitesse angulaire
        new_angle = self.angle.get() + omega * self.dt

        # Normalisation de l'angle dans la plage 0-360 degrés
        new_angle %= 360

        # Calcul de la tension des moteurs (V)
        voltage = (1 / self.k_v) * omega

        # Intégrer la commande PID pour ajuster l'angle du robot
        self.angle.set(new_angle)

        # Enregistrement des données pour les graphiques
        self.time_data.append(self.time)
        self.angle_data.append(self.angle.get())
        self.F_data.append(F)
        self.P_data.append(self.Kp * error)
        self.I_data.append(self.Ki * self.integral)
        self.D_data.append(self.Kd * derivative)

        # Affichage des valeurs dans la console
        print(f"--- Temps: {self.time} ---")
        print(f"--- Angle: {self.angle.get()} ---")
        print(f"--- Force PID: {F} ---")
        print(f"--- Proportionnelle (P): {self.Kp * error} ---")
        print(f"--- Intégrale (I): {self.Ki * self.integral} ---")
        print(f"--- Dérivée (D): {self.Kd * derivative} ---")
        print(f"--- Accélération angulaire: {alpha} ---")
        print(f"--- Nouvel angle: {new_angle} ---")
        print(f"--- Tension des moteurs: {voltage} ---")
        print(f"--- Moment d'inertie : {I} ---")
        print(f"--- Inertie en x: {self.inertia_x} ---")
        print(f"--- Intégrale: {self.integral} ---")
        print(f"--- Dérivée: {derivative} ---")
        print(f"--- Erreur: {error} ---")
        print(f"--- coefficient Kp: {self.Kp}, Ki: {self.Ki}, Kd: {self.Kd} ---\n")

        # Vérification de la stabilisation
        if 85 <= new_angle <= 95:
            self.stable_time += self.dt
        else:
            self.stable_time = 0

        if self.stable_time >= 5:
            self.stop_simulation()

        # Mise à jour du temps
        self.time += self.dt

        # Rappeler la méthode update_robot après un certain délai (pour simuler le passage du temps)
        self.root.after(int(self.dt * 1000), self.update_robot)

    def plot_pid(self):
        plt.figure(figsize=(12, 8))

        plt.subplot(4, 1, 1)
        plt.plot(self.time_data, self.angle_data, label='Angle')
        plt.ylabel('Angle (°)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.time_data, self.P_data, label='P')
        plt.ylabel('P')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(self.time_data, self.I_data, label='I')
        plt.ylabel('I')
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(self.time_data, self.D_data, label='D')
        plt.ylabel('D')
        plt.xlabel('Temps (s)')
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    root = tk.Tk()
    simulator = RobotSimulator(root)
    root.mainloop()
