import os

def sweepScenarios(filename):
    T_SIM = "5"
    max_scene = 19
    for i in range(max_scene):
        print("Scenario_B_SD"+str(2*i+2) + ", Progress "+str(round(i/max_scene, 2)*100) + "%%")
        Scenario = "Scenario_B_SD"+str(2*i+2)
        sweepmodes(Scenario, T_SIM, filename)

def sweepmodes(Scenario, T_SIM, filename):
    os.system("python main.py realsensor " + Scenario + " " + T_SIM + " " + filename)
    os.system("python main.py model0 " + Scenario + " " + T_SIM + " " + filename)
    os.system("python main.py model1 " + Scenario + " " + T_SIM + " " + filename)
    os.system("python main.py model2 " + Scenario + " " + T_SIM + " " + filename)
    os.system("python main.py model3 " + Scenario + " " + T_SIM + " " + filename)

if __name__ == '__main__':
    for i in range(10):
        sweepScenarios("Sweep_01deg_no_arg_fetch_"+str(i)+".txt")


    #for i in range(1):
    #    sweepmodes(Scenario="Scenario_B_SD16", T_SIM="5")
    #print("Done")

