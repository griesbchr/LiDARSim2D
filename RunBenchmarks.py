import os
#os.system("python main.py model0 Scenario_debug 1,4")


def sweepScenarios():
    T_SIM = "5"
    for i in range(19):
        Scenario = "Scenario_B_SD"+str(2*i+2)
        sweepmodes(Scenario, T_SIM)

def sweepmodes(Scenario, T_SIM):
    os.system("python main.py realsensor " + Scenario + " " + T_SIM)
    os.system("python main.py model0 " + Scenario + " " + T_SIM)
    os.system("python main.py model1 " + Scenario + " " + T_SIM)
    os.system("python main.py model2 " + Scenario + " " + T_SIM)
    os.system("python main.py model3 " + Scenario + " " + T_SIM)

if __name__ == '__main__':
    #sweepScenarios()
    sweepmodes(Scenario="Scenario_crossing", T_SIM="5")

