import tkinter as tk
from functools import partial

class Application(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.creer_widgets()

    def creer_widgets(self):
        self.label = tk.Label(self, text="Bonjour , Où souhaitez-vous aller?")
        self.bouton1 = tk.Button(self, text="salle de robotique", command=partial(self.get_goal,1))
        self.bouton2 = tk.Button(self, text="salle de projet ", command=partial(self.get_goal,2))
        self.bouton2 = tk.Button(self, text="salle de projet ", command=partial(self.get_goal,2))

        self.label.pack()
        self.bouton1.pack()
        self.bouton2.pack()


    def get_goal(self,x) : 
        print('salle n°' , x)

if __name__ == "__main__":
    app = Application()
    app.title("Ma Première App :-)")
    app.mainloop()
