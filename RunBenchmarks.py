import os
#os.system("python main.py model0 Scenario_debug 1,4")


def sweepScenarios():
    T_SIM = "5"
    for i in range(10):
        Scenario = "Scenario_B_S"+str(i)
        sweepmodes(Scenario, T_SIM)

def sweepmodes(Scenario, T_SIM):
    os.system("python main.py realsensor " + Scenario + " " + T_SIM)
    os.system("python main.py model0 " + Scenario + " " + T_SIM)
    os.system("python main.py model1 " + Scenario + " " + T_SIM)
    os.system("python main.py model2 " + Scenario + " " + T_SIM)
    os.system("python main.py model3 " + Scenario + " " + T_SIM)

sweepScenarios()