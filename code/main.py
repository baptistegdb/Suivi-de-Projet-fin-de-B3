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

        # Initialisation des variables
        self.init_variables()

        # Création des widgets
        self.create_widgets()

    def init_variables(self):
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

        self.inertia_x = tk.DoubleVar()
        self.inertia_x.set(0.1)  # Valeur par défaut de l'inertie en x

        self.time = 0
        self.dt = 0.1  # Intervalle de temps en secondes

        self.g = 9.81  # Accélération due à la gravité (m/s^2)

        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05

        self.previous_error = 0
        self.integral = 0

        self.k_v = 1.0

        self.running = False

        self.time_data = []
        self.angle_data = []
        self.F_data = []
        self.P_data = []
        self.I_data = []
        self.D_data = []

        self.stable_time = 0

        self.integral_limit = 10  # Limite pour l'accumulation intégrale
        self.derivative_window = 5  # Fenêtre pour lisser la dérivée
        self.error_history = []  # Historique des erreurs pour lisser la dérivée

    def create_widgets(self):
        # Frame pour les paramètres
        params_frame = tk.Frame(self.root)
        params_frame.pack(side=tk.RIGHT, padx=20)

        # Widgets pour les paramètres
        self.create_param_widgets(params_frame)

        self.update_button = tk.Button(params_frame, text="Démarrer", command=self.start_simulation)
        self.update_button.grid(row=5, columnspan=2, pady=10)

        self.stop_button = tk.Button(params_frame, text="Arrêter", command=self.stop_simulation)
        self.stop_button.grid(row=6, columnspan=2, pady=10)

        self.reset_button = tk.Button(params_frame, text="Réinitialiser", command=self.reset_simulation)
        self.reset_button.grid(row=7, columnspan=2, pady=10)

    def create_param_widgets(self, frame):
        angle_label = tk.Label(frame, text="Angle de rotation (°):")
        angle_label.grid(row=0, column=0, pady=5)
        self.angle_entry = tk.Entry(frame, textvariable=self.angle)
        self.angle_entry.grid(row=0, column=1, pady=5)

        mass_label = tk.Label(frame, text="Masse (kg):")
        mass_label.grid(row=1, column=0, pady=5)
        self.mass_entry = tk.Entry(frame, textvariable=self.mass)
        self.mass_entry.grid(row=1, column=1, pady=5)

        height_label = tk.Label(frame, text="Hauteur (m):")
        height_label.grid(row=2, column=0, pady=5)
        self.height_entry = tk.Entry(frame, textvariable=self.height)
        self.height_entry.grid(row=2, column=1, pady=5)

        length_label = tk.Label(frame, text="Longueur (m):")
        length_label.grid(row=3, column=0, pady=5)
        self.length_entry = tk.Entry(frame, textvariable=self.length)
        self.length_entry.grid(row=3, column=1, pady=5)

        inertia_label = tk.Label(frame, text="Inertie en x:")
        inertia_label.grid(row=4, column=0, pady=5)
        self.inertia_entry = tk.Entry(frame, textvariable=self.inertia_x)
        self.inertia_entry.grid(row=4, column=1, pady=5)

    def start_simulation(self):
        if not self.running:
            self.running = True
            self.calibrate_pid()
            self.update_robot()

    def stop_simulation(self):
        self.running = False
        self.plot_pid()

    def reset_simulation(self):
        self.running = False
        self.init_variables()
        self.canvas.delete("all")
        self.angle_entry.delete(0, tk.END)
        self.mass_entry.delete(0, tk.END)
        self.height_entry.delete(0, tk.END)
        self.length_entry.delete(0, tk.END)
        self.inertia_entry.delete(0, tk.END)
        self.angle_entry.insert(0, "90")
        self.mass_entry.insert(0, "0.2")
        self.height_entry.insert(0, "0.01")
        self.length_entry.insert(0, "0.005")
        self.inertia_entry.insert(0, "0.1")

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

            L = self.length.get() / 2
            I = (1/12) * self.mass.get() * (self.height.get()**2 + self.length.get()**2)
            alpha = (F * L) / I

            omega = alpha * self.dt
            temp_angle += omega * self.dt
            temp_angle %= 360

            error_sum += abs(error)

        return error_sum

    def update_robot(self):
        if not self.running:
            return

        self.canvas.delete("all")
        x = self.x_position.get()
        y = 500
        angle = math.radians(self.angle.get())
        length = 100

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

        # Limiter l'intégrale
        self.integral += error * self.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        # Calculer la dérivée avec lissage
        self.error_history.append(error)
        if len(self.error_history) > self.derivative_window:
            self.error_history.pop(0)
        if len(self.error_history) > 1:
            derivative = (self.error_history[-1] - self.error_history[-2]) / self.dt
        else:
            derivative = 0.0

        P = self.Kp * error
        I_term = self.Ki * self.integral
        D = self.Kd * derivative
        F = P + I_term + D

        L = l / 2  # Distance du centre de masse au point de pivot
        alpha = (F * L) / I  # Accélération angulaire

        # Enregistrer les données pour affichage
        self.time_data.append(self.time)
        self.angle_data.append(self.angle.get())
        self.F_data.append(F)
        self.P_data.append(P)
        self.I_data.append(I_term)
        self.D_data.append(D)

        # Mettre à jour l'angle
        omega = alpha * self.dt
        new_angle = self.angle.get() + omega * self.dt
        self.angle.set(new_angle % 360)

        # Afficher les informations PID
        print(f"--- Time: {self.time} ---")
        print(f"--- Angle: {self.angle.get()} ---")
        print(f"--- Force PID: {F} ---")
        print(f"--- Proportionnelle (P): {P} ---")
        print(f"--- Intégrale (I): {I_term} ---")
        print(f"--- Dérivée (D): {D} ---")
        print(f"--- Accélération angulaire: {alpha} ---")
        print(f"--- Nouvel angle: {new_angle} ---")
        print(f"--- Tension des moteurs: {F} ---")
        print(f"--- Moment d'inertie : {I} ---")
        print(f"--- Inertie en x: {self.inertia_x.get()} ---")
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
